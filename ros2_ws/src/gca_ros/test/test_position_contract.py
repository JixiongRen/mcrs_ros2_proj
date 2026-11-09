"""Fixed-mode ROS contracts without starting a runtime or opening devices."""

import asyncio
import json
import time
from dataclasses import replace
from pathlib import Path
from runpy import run_path
from types import SimpleNamespace

import pytest
import rclpy
from gca.domain.servo import FeedRuntimeState, ServoState
from gca.servo_config import make_position_settings, make_servo_settings, servo_settings_document
from gca_ros.conversion import state_from_core
from gca_ros.security import AccessError
from gca_ros.servo_node import ServoControlNode
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import ServoJogCommand
from gca_system_interfaces.srv import ControlSession, GetCapabilities, ManageDevice
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from system_coordinator.authority import SessionError
from system_coordinator.node import SystemCoordinator

ROOT = Path(__file__).resolve().parents[4]


def snapshot(tmp_path, mode, backend="simulation"):
    servo = run_path(str(ROOT / "config/servo_settings.py"))
    runtime = servo["settings_payload"](backend)
    if mode == "position":
        encoder = run_path(str(ROOT / "config/encoder_settings.py"))["settings_payload"]()
        config = make_position_settings(runtime, servo["position_settings_payload"](), encoder)
    else:
        config = make_servo_settings(runtime)
    path = tmp_path / "servo.json"
    path.write_text(json.dumps(servo_settings_document(config)))
    return path, config


@pytest.mark.parametrize(
    "mode,source_mode,policy",
    [
        ("manual", "MANUAL", "MANUAL_SERVO_VELOCITY"),
        ("position", "AUTO", "POSITION_EXTERNAL_ENCODER"),
    ],
)
def test_fixed_modes_reject_other_sources_before_claim_or_jog(tmp_path, mode, source_mode, policy):
    # Hardware snapshots are only parsed: configure is never called in this test.
    path, config = snapshot(tmp_path, mode, "hardware")
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "coordinator_key:=position-contract",
            "-p",
            "profile:=servo",
            "-p",
            f"servo_config_file:={path}",
            "-p",
            f"control_mode:={mode}",
        ]
    )
    node = coordinator = None
    try:
        node, coordinator = ServoControlNode(), SystemCoordinator()
        assert node._snapshot().control_policy == coordinator._state.control_policy == policy
        assert not node._snapshot().armed
        assert not node.set_parameters([Parameter("control_mode", value="manual")])[0].successful
        assert not coordinator.set_parameters([Parameter("control_mode", value="manual")])[
            0
        ].successful
        other_mode = "AUTO" if source_mode == "MANUAL" else "MANUAL"
        denied = asyncio.run(
            coordinator._control(
                "claim_control",
                ControlSession.Request(owner="wrong", mode=other_mode, lease_s=1.0),
                ControlSession.Response(),
            )
        )
        assert not denied.accepted and denied.error_code == "CONTROL_MODE_MISMATCH"
        assert coordinator._authority.current is None
        with pytest.raises(AccessError, match="CONTROL_MODE_MISMATCH"):
            node._validate_execution_policy(
                "POSITION_EXTERNAL_ENCODER" if mode == "manual" else "MANUAL_SERVO_VELOCITY"
            )
        source = coordinator._authority.claim("test", source_mode, 1.0)
        assert coordinator._lease_request(source, 0.5).control_policy == policy
        with pytest.raises(SessionError, match="CONTROL_MODE_MISMATCH"):
            coordinator._validate_source_mode(other_mode)

        arm_calls, jog_calls = [], []
        state = FeedRuntimeState(configured=True, control_policy=policy)
        node.runtime = SimpleNamespace(
            snapshot=lambda: state, arm=lambda *args, **kw: arm_calls.append(kw)
        )
        capabilities = node._capabilities(GetCapabilities.Request(), GetCapabilities.Response())
        assert capabilities.can_servo_velocity is (mode == "manual")
        assert capabilities.can_linear_motion is (mode == "position")
        assert capabilities.can_measure_linear is (mode == "position")
        if mode == "position":
            stopped = FeedRuntimeState(
                boot_id="freeze",
                state_seq=1,
                control_policy=policy,
                stop_progress="CONFIRMED",
                servo_stop_confirmed=True,
                linear_stop_confirmed=True,
                sample_age_s=0.02,
            )
            assert coordinator._update_state(state_from_core(stopped))
            assert coordinator._update_state(
                state_from_core(
                    replace(
                        stopped,
                        sample_age_s=0.3,
                        servo_stop_confirmed=False,
                        linear_stop_confirmed=False,
                    )
                )
            )
            assert coordinator._state.stop.progress == "CONFIRMED"
            assert not coordinator._state.stop.servo_confirmed
            assert not coordinator._state.stop.linear_confirmed
            assert (
                capabilities.limits.max_linear_m_s == config.position.motion.max_linear_mm_s / 1000
            )
            assert (
                capabilities.limits.max_linear_m_s2
                == config.position.motion.max_linear_mm_s2 / 1000
            )
            grant = SimpleNamespace(
                session_id="s",
                generation=1,
                sequence=1,
                deadline=time.monotonic() + 1,
                token="t",
                control_policy=policy,
            )
            node._arm_runtime(grant, ManageDevice.Request(), 7)
            assert arm_calls == [{"expected_stop_serial": 7, "input_deadline_ns": 0}]
            node._dispatch_jog = lambda *args: jog_calls.append(args)
            coordinator._verify = lambda *args, **kwargs: pytest.fail(
                "position Jog reached authorization"
            )
            for deadman in (False, True):
                node._servo_jog(ServoJogCommand(deadman=deadman))
                coordinator._jog(ServoJogCommand(deadman=deadman))
            assert not jog_calls
            for axis, end_policy, rotation in ((3, 1, 0.0), (1, 2, 0.0), (1, 1, 1.0)):
                request = ExecuteMotion.Goal()
                request.target.axis_mask, request.target.mode = axis, 1
                request.target.rotation_rad, request.end_policy = rotation, end_policy
                assert node._goal(request) == GoalResponse.REJECT
                assert node._denials[""] == "POSITION_LINEAR_STOP_REQUIRED"
        else:
            assert node._goal(ExecuteMotion.Goal()) == GoalResponse.REJECT
            assert node._denials[""] == "CONTROL_MODE_MISMATCH"
    finally:
        if node is not None:
            node.runtime = None
            node.destroy_node()
        if coordinator is not None:
            coordinator.destroy_node()
        rclpy.shutdown()


def test_snapshot_mode_mismatch_is_refused_before_runtime_creation(tmp_path):
    path, _ = snapshot(tmp_path, "position")
    for invalid in ("manual", "invalid"):
        values = {
            "servo_config_file": str(path),
            "control_mode": invalid,
            "simulation_slip_ratio": 1.0,
        }
        node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))
        with pytest.raises(ValueError, match="control_mode"):
            ServoControlNode._read_config(node)


def test_position_feedback_and_stop_evidence_are_not_servo_estimates():
    core = FeedRuntimeState(
        control_policy="POSITION_EXTERNAL_ENCODER",
        device_epoch="encoder-epoch",
        sample_seq=23,
        position_mm=123.0,
        velocity_mm_s=-2.0,
        linear_valid=True,
        velocity_valid=True,
        linear_referenced=True,
        reference_id="reference",
        sample_age_s=0.02,
        stop_progress="CONFIRMED",
        servo_stop_confirmed=True,
        linear_stop_confirmed=False,
        servo=ServoState(
            device_epoch="servo-epoch",
            sample_seq=456,
            measured_velocity_rpm=0.0,
            valid=True,
            sample_age_s=0.04,
        ),
    )
    message = state_from_core(core)
    assert (message.linear_position_m, message.linear_velocity_m_s) == (0.123, -0.002)
    assert message.linear_valid and message.linear_velocity_valid and message.linear_reference_valid
    assert message.feedback_age_s == 0.02 and message.servo.sample_age_s == 0.04
    assert message.device_epoch == "encoder-epoch" and message.servo.device_epoch == "servo-epoch"
    assert message.sample_seq == 23 and message.servo.sample_seq == 456
    assert message.stop.servo_confirmed and not message.stop.linear_confirmed
    assert state_from_core(replace(core, linear_stop_confirmed=True)).stop.linear_confirmed
