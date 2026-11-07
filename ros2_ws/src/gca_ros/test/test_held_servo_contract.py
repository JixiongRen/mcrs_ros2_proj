"""Installed ROS constructors and separate held-input ARM/JOG ingress; no hardware."""

import json
import time
from threading import Event, RLock
from types import SimpleNamespace

import pytest
import rclpy
from gca.domain.manual import ManualInputQuality, QualityError, host_clock_id
from gca.servo_config import make_servo_settings, servo_settings_document
from gca_ros.conversion import quality_to_ros
from gca_ros.security import AccessError
from gca_ros.servo_node import ServoControlNode
from gca_system_interfaces.msg import ServoJogCommand
from gca_system_interfaces.srv import ManageDevice
from gca_teleop.model import Intent
from gca_teleop.node import TeleopNode
from gca_teleop.policy import TeleopPolicy
from system_coordinator.node import SystemCoordinator
from test_native_servo import settings


def proof(kind):
    now = time.monotonic_ns()
    return ManualInputQuality(
        kind,
        "physical-input",
        "epoch",
        1,
        now,
        "JOYDEV_INIT_ANCHORED_U32_MS",
        1,
        now,
        now + 150_000_000,
        host_clock_id(),
        "UNKNOWN",
        episode_end_ns=0,
    )


def test_same_snapshot_enables_held_ingress_and_arm_proof_cannot_move(tmp_path):
    payload = servo_settings_document(settings())
    payload.pop("sha256")
    payload.update(backend="hardware", manual_input_mode="joydev_hold")
    config = make_servo_settings(payload)
    path = tmp_path / "servo.json"
    path.write_text(json.dumps(servo_settings_document(config)))
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "profile:=servo",
            "-p",
            f"servo_config_file:={path}",
            "-p",
            "coordinator_key:=held-test",
        ]
    )
    servo = coordinator = None
    try:
        servo, coordinator = ServoControlNode(), SystemCoordinator()
        quality = proof("JOYDEV_HELD_STATE")
        assert coordinator._quality.validate(quality, time.monotonic_ns()) == quality.valid_until_ns
        assert (
            servo._create_quality_tracker().validate(quality, time.monotonic_ns())
            == quality.valid_until_ns
        )
        dispatched = []
        servo._dispatch_jog = lambda request, converter: dispatched.append(request)
        for kind in ("JOYDEV_HELD_ARM", "QUALIFIED_STATE_CONTRACT"):
            servo._servo_jog(ServoJogCommand(deadman=True, quality=quality_to_ros(proof(kind))))
        assert not dispatched
        servo._servo_jog(ServoJogCommand(deadman=True, quality=quality_to_ros(quality)))
        assert len(dispatched) == 1
        servo._servo_jog(ServoJogCommand(deadman=False))
        assert len(dispatched) == 2  # STOP retains the common ingress.

        calls = []
        servo.runtime = SimpleNamespace(arm=lambda *args, **kwargs: calls.append(kwargs))
        grant = SimpleNamespace(
            session_id="s",
            generation=1,
            sequence=1,
            deadline=time.monotonic() + 1,
            token="t",
            control_policy="MANUAL_SERVO_VELOCITY",
        )
        neutral = proof("JOYDEV_HELD_ARM")
        request = ManageDevice.Request(
            has_input_quality=True, input_quality=quality_to_ros(neutral)
        )
        servo._arm_runtime(grant, request, 7)
        assert calls[0]["input_deadline_ns"] == neutral.valid_until_ns
        request.input_quality = quality_to_ros(proof("JOYDEV_HELD_STATE"))
        with pytest.raises(AccessError, match="HARDWARE_ARM_INPUT_NOT_QUALIFIED"):
            servo._arm_runtime(grant, request, 7)
        assert len(calls) == 1
    finally:
        if servo is not None:
            servo.runtime = None
            servo.destroy_node()
        if coordinator is not None:
            coordinator.destroy_node()
        rclpy.shutdown()


def test_coordinator_without_held_snapshot_keeps_previous_gate():
    rclpy.init(args=["--ros-args", "-p", "profile:=servo", "-p", "coordinator_key:=default-test"])
    node = None
    try:
        node = SystemCoordinator()
        with pytest.raises(QualityError):
            node._quality.validate(proof("JOYDEV_HELD_STATE"), time.monotonic_ns())
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("held", [True, False])
def test_servo_timer_tick_does_not_skip_for_early_arrival_at_previous_send_deadline(held):
    offered, pumped = [], []
    quality = proof("JOYDEV_HELD_STATE")
    intent = Intent(action="JOG", quality=quality, servo_rpm=1.0)
    node = SimpleNamespace(
        _closing=Event(),
        _policy=TeleopPolicy("servo", 9.847, manual_input_mode="joydev_hold")
        if held
        else TeleopPolicy(),
        _pipeline_lock=RLock(),
        _pipeline=SimpleNamespace(tick=lambda *args, **kwargs: intent),
        _health=None,
        _session=SimpleNamespace(gate=lambda: {}),
        _dispatcher=SimpleNamespace(
            guard=lambda: "guard",
            offer=lambda *args: offered.append(args),
            pump=lambda: pumped.append(True),
        ),
        _accept_intent=lambda value: None,
        # A timer callback may precede the previous callback's now+period.
        _dispatch_after_ns=time.monotonic_ns() + 10_000_000,
        profile=SimpleNamespace(dispatch_hz=25.0),
    )
    TeleopNode._tick(node)
    assert bool(offered) is held and bool(pumped) is held
    if held:
        assert offered[0][0].quality == quality  # No rewritten input deadline.
