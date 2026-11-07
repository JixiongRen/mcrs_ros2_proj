"""Servo-only units, capability binding and disabled external position ingress."""

import math
from dataclasses import replace

import rclpy
from gca.domain.servo import FeedRuntimeState, ServoState
from gca_ros.conversion import servo_jog_from_ros, state_from_core
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import CommandContext, GcaState, ServoJogCommand, StopStatus
from rclpy.action import GoalResponse
from system_coordinator.node import SystemCoordinator


def test_servo_si_boundary_never_populates_external_axis_measurements():
    core = FeedRuntimeState(
        state="READY",
        stop_progress="CONFIRMED",
        servo=ServoState(
            raw_position_count=4096,
            shaft_angle_rad=2 * math.pi,
            measured_velocity_rpm=-3.0,
            requested_velocity_rpm=-4.58,
            accepted_velocity_rpm=-4.58,
            valid=True,
            configured=True,
        ),
    )
    msg = state_from_core(core)
    assert msg.mode == "SERVO_ONLY"
    assert msg.control_policy == "MANUAL_SERVO_VELOCITY"
    assert msg.servo.raw_position_count == 4096
    assert math.isclose(msg.servo.measured_velocity_rad_s, -math.pi / 10)
    assert not msg.linear_valid and not msg.rotation_estimate_valid
    assert not msg.linear_reference_valid and not msg.rotation_reference_valid
    assert msg.stop.servo_confirmed and not msg.stop.linear_confirmed
    assert not msg.stop.rotation_confirmed
    stale = state_from_core(replace(core, servo=replace(core.servo, valid=False)))
    assert not stale.servo.valid
    command = servo_jog_from_ros(
        ServoJogCommand(
            context=CommandContext(
                session_id="s", task_id="t", command_id="c", generation=1, sequence=2
            ),
            servo_angular_velocity_rad_s=-math.pi / 10,
            ttl_s=0.15,
            deadman=False,
        )
    )
    assert math.isclose(command.velocity_rpm, -3.0)


def test_coordinator_binds_servo_source_mode_and_rejects_position_without_encoder():
    rclpy.init(
        args=["--ros-args", "-p", "coordinator_key:=servo-contract-key", "-p", "profile:=servo"]
    )
    node = None
    try:
        node = SystemCoordinator()
        source = node._authority.claim("test", "MANUAL", 1.0)
        lease = node._lease_request(source, 0.5)
        assert lease.control_policy == "MANUAL_SERVO_VELOCITY"
        request = ExecuteMotion.Goal()
        request.context = node._domain_context(source)
        request.token = source.token
        # Source authentication is actual, independent of unavailable actuator state.
        from gca_ros.permits import capability_part

        assert capability_part(request.token + ".test") == source.token
        request.token += ".test"
        assert node._goal(request) == GoalResponse.REJECT
        assert node._denials[request.context.command_id] == "AUTO_MODE_REQUIRED"
        node._authority.release()
        source = node._authority.claim("auto", "AUTO", 1.0)
        assert node._lease_request(source, 0.5).control_policy == "POSITION_EXTERNAL_ENCODER"
        request.context = node._domain_context(source)
        request.token = source.token + ".test"
        assert node._goal(request) == GoalResponse.REJECT
        assert node._denials[request.context.command_id] == "EXTERNAL_ENCODER_POSITION_UNSUPPORTED"
        baseline = GcaState(
            boot_id="freeze",
            state_seq=9,
            state="EXECUTING",
            armed=True,
            stop_serial=1,
            stop=StopStatus(progress="CONFIRMED"),
        )
        assert node._update_state(baseline)
        frozen = GcaState(
            boot_id="freeze",
            state_seq=9,
            state="EXECUTING",
            armed=True,
            stop_serial=2,
            stop=StopStatus(progress="REQUESTED"),
        )
        assert node._update_state(frozen)
        assert node._state.stop.progress == "REQUESTED" and node._state.stop_serial == 2
        assert node._state.state == "EXECUTING"  # No invented control progress.
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
