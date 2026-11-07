"""Bounded native-servo runtime checks; never opens physical devices or ROS."""

import runpy
from dataclasses import replace
from math import pi
from pathlib import Path
from threading import Event, Thread
from time import monotonic, monotonic_ns

import pytest
from gca.domain.manual import ManualInputQuality, host_clock_id
from gca.domain.models import CommandContext, DeviceError, Lease, MotionCommand
from gca.domain.servo import MANUAL_SERVO_VELOCITY, ServoJogCommand
from gca.runtime.feed_axis import FeedAxisRuntime
from gca.servo_config import make_servo_settings
from gca.simulation.servo import SimulatedVelocityDrive


def settings(backend="simulation"):
    source = Path(__file__).resolve().parents[4] / "config" / "servo_settings.py"
    return make_servo_settings(runpy.run_path(str(source))["settings_payload"](backend))


def wait_for(predicate, timeout=1.5):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        Event().wait(0.005)
    raise AssertionError("bounded servo condition did not become true")


@pytest.fixture
def runtime():
    runtime = FeedAxisRuntime(settings())
    assert runtime.start().accepted
    assert runtime.activate().accepted
    yield runtime
    runtime.inject_fault("control_freeze", False)
    runtime.drive.fail_read = runtime.drive.fail_write = False
    runtime.drive._watchdog_raw = runtime.config.device.watchdog_raw
    runtime.close()


def arm(runtime, generation=1, policy=MANUAL_SERVO_VELOCITY):
    return runtime.arm(Lease("servo-session", generation, 1, 3.0, "executor", policy))


def test_full_speed_si_roundtrip_is_accepted_but_real_overspeed_is_rejected(runtime):
    assert arm(runtime).accepted
    limit = runtime.config.max_rpm
    rad_s = limit * 2.0 * pi / 60.0
    roundtrip_rpm = rad_s * 60.0 / (2.0 * pi)
    for seq, sign in enumerate((-1, 1), start=10):
        assert runtime.jog(command(sign * roundtrip_rpm, seq)).accepted
        assert runtime._jog.velocity_rpm == sign * limit
    assert runtime.jog(command(limit + 0.001, 12)).code == "INVALID_SERVO_JOG"


def command(rpm, seq, *, source_kind="SCRIPT_SAMPLE"):
    now = monotonic_ns()
    return ServoJogCommand(
        CommandContext("servo-session", "manual", f"servo-{seq}", 1, seq),
        rpm,
        freshness_token="executor",
        ttl_s=0.15,
        quality=ManualInputQuality(
            source_kind,
            "scripted-servo-unit",
            "epoch-1",
            seq,
            now,
            "SCRIPT_MONOTONIC",
            seq,
            now,
            now + 150_000_000,
            host_clock_id(),
            "SYNTHETIC",
        ),
    )


def drive_for(runtime, rpm, duration=0.30, start_seq=10):
    end, seq = monotonic() + duration, start_seq
    while monotonic() < end:
        result = runtime.jog(command(rpm, seq))
        assert result.accepted, result
        seq += 1
        Event().wait(0.03)
    return seq


def test_servo_has_no_external_reference_or_device_dependency(runtime):
    assert not hasattr(runtime, "encoder") and not hasattr(runtime, "pulse")
    assert len(runtime._threads) == 2
    assert not runtime.snapshot().linear_valid
    assert not runtime.snapshot().linear_referenced
    assert not runtime.snapshot().rotation_estimate_valid
    assert not runtime.snapshot().rotation_reference_valid
    assert runtime.set_reference().code == "EXTERNAL_ENCODER_REQUIRED"
    assert (
        runtime.reserve(MotionCommand(CommandContext("s", "t", "c", 1, 1))).code
        == "POSITION_CONTROL_UNAVAILABLE"
    )
    assert arm(runtime).accepted
    assert runtime.snapshot().armed
    assert not runtime.snapshot().linear_referenced


@pytest.mark.parametrize("rpm", [-1.2, 1.2])
def test_signed_native_velocity_drives_independent_shaft_feedback(runtime, rpm):
    assert arm(runtime).accepted
    drive_for(runtime, rpm)
    state = runtime.snapshot()
    assert state.servo.measured_velocity_rpm * rpm > 0
    assert state.servo.raw_position_count * rpm > 0
    assert abs(state.servo.measured_velocity_rpm) < abs(rpm)
    assert not state.linear_valid and state.position_mm == 0
    assert not state.rotation_estimate_valid and state.rotation_deg == 0
    assert runtime.stop("B").accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert not runtime.snapshot().servo.torque_enabled
    assert runtime.snapshot().servo.requested_velocity_rpm == 0.0
    assert runtime.snapshot().servo.accepted_velocity_rpm == 0.0
    assert (
        abs(runtime.snapshot().servo.measured_velocity_rpm) <= runtime.config.velocity_tolerance_rpm
    )
    assert runtime.records()[-1]["stop_reason"] == "B"
    assert "time_ns" in runtime.records()[-1]
    assert runtime.disarm().accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.snapshot().servo.accepted_velocity_rpm == 0.0
    assert runtime.close().accepted
    assert runtime.snapshot().servo.accepted_velocity_rpm == 0.0


def test_io_deadline_stops_motion_when_control_frozen_but_reads_continue(runtime):
    assert arm(runtime).accepted
    drive_for(runtime, 1.2)
    before_seq = runtime.simulation_evidence()["sample"]["seq"]
    assert runtime.inject_fault("control_freeze", True).accepted
    frozen_state_seq = runtime.snapshot().state_seq
    wait_for(lambda: not runtime.simulation_evidence()["sample"]["enabled"])
    wait_for(
        lambda: (
            abs(runtime.simulation_evidence()["sample"]["velocity_rpm"])
            <= runtime.config.velocity_tolerance_rpm
        )
    )
    evidence = runtime.simulation_evidence()
    assert evidence["sample"]["seq"] > before_seq
    assert runtime.snapshot().state_seq == frozen_state_seq
    assert runtime.snapshot().stop_progress == "REQUESTED"
    assert runtime.inject_fault("control_freeze", False).accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert not runtime.snapshot().servo.torque_enabled
    assert runtime.snapshot().servo.accepted_velocity_rpm == 0.0


def test_wrong_policy_and_stop_fence_cannot_reenable_old_commands(runtime):
    assert arm(runtime, policy="LEGACY_DUAL_AXIS").code == "CONTROL_POLICY_UNSUPPORTED"
    assert arm(runtime, policy="POSITION_EXTERNAL_ENCODER").code == "CONTROL_POLICY_UNSUPPORTED"
    assert arm(runtime).accepted
    assert not runtime.renew(
        Lease("servo-session", 1, 2, 3, "executor", "POSITION_EXTERNAL_ENCODER")
    ).accepted
    stale_serial = runtime.stop_serial
    context = CommandContext("servo-session", "manual", "stop-100", 1, 100)
    assert runtime.stop("B", context=context).accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.jog(command(1.0, 99)).code == "STOP_FENCED"
    assert (
        runtime.arm(
            Lease("servo-session", 2, 1, 3, "executor", MANUAL_SERVO_VELOCITY),
            expected_stop_serial=stale_serial,
        ).code
        == "STOP_SUPERSEDED"
    )
    assert not runtime.snapshot().servo.torque_enabled


def test_deadman_release_revokes_output_without_normal_command_queue(runtime):
    assert arm(runtime).accepted
    seq = drive_for(runtime, -1.2)
    assert runtime.inject_fault("control_freeze", True).accepted
    release = replace(command(0, seq + 1), deadman=False, quality=None)
    assert runtime.jog(release).accepted
    wait_for(lambda: not runtime.simulation_evidence()["sample"]["enabled"])
    runtime.inject_fault("control_freeze", False)
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")


def test_zero_write_failure_still_disables_torque_and_preserves_fault(runtime):
    assert arm(runtime).accepted
    drive_for(runtime, 1.2)
    assert runtime.snapshot().servo.accepted_velocity_rpm > 0.0
    runtime.drive.fail_write = True
    assert runtime.stop("B").accepted
    wait_for(lambda: not runtime.simulation_evidence()["sample"]["enabled"])
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.snapshot().state == "FAULT"
    assert runtime.snapshot().error_code
    assert not runtime.snapshot().armed
    assert runtime.snapshot().servo.requested_velocity_rpm == 0.0
    # Torque disable stops the plant, but failed zero writes leave the last
    # accepted goal register unchanged. Do not manufacture an accepted zero.
    assert runtime.snapshot().servo.accepted_velocity_rpm > 0.0


def test_failed_reads_do_not_manufacture_stop_confirmation(runtime):
    assert arm(runtime).accepted
    drive_for(runtime, 1.2)
    last_seq = runtime.simulation_evidence()["sample"]["seq"]
    runtime.drive.fail_read = True
    assert runtime.stop("B").accepted
    wait_for(
        lambda: runtime.snapshot().stop_progress == "UNKNOWN",
        timeout=runtime.config.stop_timeout_s + 0.5,
    )
    assert runtime.snapshot().state == "FAULT"
    assert runtime.snapshot().servo.sample_seq == last_seq
    assert not runtime.snapshot().servo.valid


@pytest.mark.parametrize("leading_stop", [False, True])
def test_pending_fault_reason_survives_later_stop_requests(leading_stop):
    runtime = FeedAxisRuntime(settings())
    if leading_stop:
        runtime._latch_stop("B")
    runtime._latch_stop("DEVICE_TIMEOUT", fault=True)
    runtime._latch_stop("COORDINATOR_STOP", disarm=True)
    reason, disarm, fault, serial = runtime._stop_request
    assert reason == "DEVICE_TIMEOUT" and disarm and fault
    assert serial == (3 if leading_stop else 2)
    runtime._begin_stop(reason, monotonic_ns(), disarm=disarm, fault=fault)
    assert runtime.snapshot().error_code == "DEVICE_TIMEOUT"
    assert runtime._stopping["reason"] == "DEVICE_TIMEOUT"
    assert runtime._last_stop_reason == "DEVICE_TIMEOUT"


def test_active_stop_upgrades_to_first_fault_without_overwriting_its_reason():
    runtime = FeedAxisRuntime(settings())
    now = monotonic_ns()
    runtime._begin_stop("B", now)
    deadline = runtime._stopping["deadline"]
    runtime._begin_stop("DEVICE_TIMEOUT", now + 1, fault=True)
    runtime._begin_stop("COORDINATOR_STOP", now + 2, disarm=True)
    runtime._begin_stop("FEEDBACK_STALE", now + 3, fault=True)
    assert runtime._stopping["fault"] and runtime._stopping["disarm"]
    assert runtime._stopping["deadline"] == deadline
    assert runtime._stopping["reason"] == "DEVICE_TIMEOUT"
    assert runtime._last_stop_reason == "DEVICE_TIMEOUT"


def test_confirmed_fault_reason_survives_management_stop(runtime):
    runtime._latch_stop("DEVICE_TIMEOUT", fault=True)
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.snapshot().state == "FAULT"
    assert runtime.stop("COORDINATOR_STOP", disarm=True).accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.snapshot().error_code == "DEVICE_TIMEOUT"
    assert runtime.records()[-1]["stop_reason"] == "DEVICE_TIMEOUT"
    assert not runtime.snapshot().armed and not runtime.snapshot().servo.torque_enabled


def test_hardware_arm_requires_original_input_deadline_without_real_hardware():
    config = settings("hardware")
    # Explicit injected simulation port exercises the hardware policy gate only.
    runtime = FeedAxisRuntime(config, drive=SimulatedVelocityDrive(config))
    try:
        assert runtime.start().accepted
        assert runtime.activate().accepted
        lease = Lease("servo-session", 1, 1, 1, "executor", MANUAL_SERVO_VELOCITY)
        assert runtime.arm(lease).code == "INPUT_NOT_QUALIFIED"
        assert (
            runtime.arm(lease, input_deadline_ns=monotonic_ns() - 1).code == "INPUT_NOT_QUALIFIED"
        )
        assert not runtime.snapshot().armed
        assert not runtime.snapshot().servo.torque_enabled
    finally:
        runtime.close()


def test_unconnected_and_failed_open_never_claim_stopped_or_attempt_stop_writes():
    class FailedOpenDrive(SimulatedVelocityDrive):
        stop_attempts = 0

        def open_identified(self):
            raise DeviceError("CONNECT_FAILED")

        def stop(self, deadline_ns):
            self.stop_attempts += 1
            super().stop(deadline_ns)

    config = settings()
    drive = FailedOpenDrive(config)
    runtime = FeedAxisRuntime(config, drive=drive)
    assert runtime.snapshot().stop_progress == "UNKNOWN"
    assert runtime.snapshot().stop_evidence == "UNKNOWN"
    assert runtime.start().code == "CONNECT_FAILED"
    assert runtime.snapshot().stop_progress == "UNKNOWN"
    assert runtime.close().accepted
    assert drive.stop_attempts == 0
    assert not runtime._cleanup_failed.is_set()


def test_close_joins_remaining_owner_even_when_start_did_not_complete():
    runtime = FeedAxisRuntime(settings())
    release = Event()
    owner = Thread(target=lambda: release.wait(0.5), daemon=True)
    runtime._threads.append(owner)
    owner.start()
    try:
        assert runtime.close(timeout_s=0.01).code == "SHUTDOWN_TIMEOUT"
        assert owner.is_alive()
    finally:
        release.set()
        assert runtime.close(timeout_s=0.5).accepted
    assert not owner.is_alive()
    assert runtime.snapshot().stop_progress == "UNKNOWN"
