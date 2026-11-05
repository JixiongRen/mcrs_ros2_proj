"""T05/T07–T10/T18: local threaded owners exercised without ROS or hardware."""

from dataclasses import replace
from threading import Event
from time import monotonic, sleep

import pytest
from gca.config import RuntimeConfig
from gca.domain.models import (
    CommandContext,
    EncoderSample,
    JogCommand,
    Lease,
    MotionCommand,
    MotionLimits,
    PulseAbort,
    PulseCompletion,
    PulseSegment,
    ServoSample,
)
from gca.runtime import ControlRuntime


def wait_for(predicate, timeout=3.0):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        sleep(0.01)
    raise AssertionError("condition did not become true before deadline")


@pytest.fixture
def runtime():
    runtime = ControlRuntime()
    assert runtime.start().accepted
    assert runtime.activate().accepted
    wait_for(lambda: runtime.snapshot().velocity_valid)
    assert runtime.set_reference().accepted
    yield runtime
    runtime.close()


def armed(runtime, ttl=5):
    result = runtime.arm(Lease("session", 1, 1, ttl, "token"))
    assert result.accepted
    return result.token


def command(runtime, seq=1, **kwargs):
    return MotionCommand(
        CommandContext("session", "task", f"command-{seq}", 1, seq),
        freshness_token="token",
        end_policy=1,
        reference_id=runtime.snapshot().reference_id,
        **kwargs,
    )


def run_motion(runtime, motion):
    assert runtime.reserve(motion).accepted
    assert runtime.start_motion(motion).accepted


def terminal(runtime, timeout=4):
    events = []

    def collect():
        events.extend(runtime.drain_events())
        return bool(events)

    wait_for(collect, timeout)
    event = events[-1]
    runtime.commit_terminal(event.command_id, event.outcome)
    return event


def test_unarmed_rejected_and_full_closed_loop_uses_plant(runtime):
    motion = command(runtime, linear_mm=3, rotation_deg=8)
    assert runtime.reserve(motion).code == "NOT_ARMED"
    armed(runtime)
    run_motion(runtime, motion)
    assert runtime.snapshot().position_mm < 1
    result = terminal(runtime)
    assert result.outcome == "SUCCEEDED" and result.stop_confirmed
    assert abs(runtime.snapshot().position_mm - 3) <= 0.08
    assert abs(runtime.plant.snapshot().position_mm - 3) <= 0.09


def test_stop_on_disarmed_device_never_creates_arm_permission(runtime):
    assert not runtime.snapshot().armed
    assert runtime.stop().accepted
    wait_for(lambda: runtime.snapshot().stop_progress == "CONFIRMED")
    assert runtime.snapshot().state == "DISARMED"
    assert not runtime.snapshot().armed
    assert runtime.reserve(command(runtime, linear_mm=1)).code == "NOT_ARMED"


def test_relative_command_retries_do_not_move_twice(runtime):
    armed(runtime)
    motion = command(runtime, axis_mask=1, mode=2, linear_mm=-1)
    run_motion(runtime, motion)
    assert not runtime.reserve(motion).accepted
    assert terminal(runtime).outcome == "SUCCEEDED"
    assert not runtime.reserve(motion).accepted
    assert abs(runtime.snapshot().position_mm + 1) <= 0.08


def test_both_axis_validation_prevents_partial_motion(runtime):
    armed(runtime)
    motion = command(runtime, linear_mm=1, rotation_deg=10000)
    assert runtime.reserve(motion).code == "ROTATION_LIMIT"
    sleep(0.05)
    assert runtime.plant.snapshot().position_mm == 0


def test_cancel_waits_for_stop_confirmation(runtime):
    armed(runtime)
    motion = command(runtime, linear_mm=30, rotation_deg=100)
    run_motion(runtime, motion)
    wait_for(lambda: runtime.snapshot().position_mm > 0.1)
    assert runtime.stop("CANCEL", motion.context.command_id).accepted
    result = terminal(runtime)
    assert result.outcome == "CANCELED" and result.stop_confirmed
    position = runtime.snapshot().position_mm
    sleep(0.1)
    assert abs(runtime.snapshot().position_mm - position) < 0.03


def test_feedback_freeze_stops_and_does_not_renew_cached_sample(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().position_mm > 0.1)
    assert runtime.inject_fault("encoder_freeze").accepted
    event = terminal(runtime)
    assert event.outcome == "FAILED"
    assert runtime.snapshot().state == "FAULT"
    assert not runtime.snapshot().linear_valid
    record = runtime.records()[-1]
    assert record["sample_age_s"] > runtime.config.feedback_max_age_s
    assert record["sample_age_s"] == pytest.approx(
        (record["time_ns"] - record["sample_received_ns"]) / 1e9
    )


def test_owner_deadlines_stop_when_control_task_frozen(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().applied_velocity_mm_s > 1)
    runtime.inject_fault("control_freeze", True)
    wait_for(lambda: abs(runtime.plant.advance(runtime.clock()).velocity_mm_s) < 0.01, 1.5)
    runtime.inject_fault("control_freeze", False)
    assert terminal(runtime).outcome == "FAILED"


def test_lease_expiry_does_not_accept_late_renewal(runtime):
    armed(runtime, 0.15)
    sleep(0.18)
    assert not runtime.renew(Lease("session", 1, 2, 1, "token")).accepted
    wait_for(lambda: runtime.snapshot().state == "DISARMED")


def test_hold_has_independent_context_and_feedback_expiry_exits(runtime):
    armed(runtime)
    motion = replace(command(runtime, axis_mask=1, linear_mm=1), end_policy=2)
    run_motion(runtime, motion)
    event = terminal(runtime)
    assert event.outcome == "SUCCEEDED" and event.outputs_held
    assert runtime.snapshot().state == "HOLDING"
    runtime.inject_fault("encoder_disconnect")
    wait_for(lambda: runtime.snapshot().state == "FAULT")


def test_zero_pulse_target_and_single_axis_reference(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=2, rotation_deg=0.001))
    assert terminal(runtime).outcome == "SUCCEEDED"
    assert runtime.snapshot().confirmed_pulses == 0


def test_jog_deadman_and_short_ttl_stop(runtime):
    armed(runtime)
    jog = JogCommand(
        CommandContext("session", "jog", "jog-1", 1, 1),
        linear_velocity_mm_s=2,
        axis_mask=1,
        freshness_token="token",
        ttl_s=0.1,
    )
    assert runtime.jog(jog).accepted
    wait_for(lambda: runtime.snapshot().state == "READY")
    assert runtime.plant.snapshot().velocity_mm_s < 0.15


def test_configuration_transaction_rejects_structural_change(runtime):
    result = runtime.configure(replace(RuntimeConfig(), encoder_mm_per_count=0.1))
    assert result.code == "RECONFIGURE_REQUIRED"
    assert runtime.snapshot().applied_config_revision == "simulation-v1"


def test_unknown_stop_is_failed_even_if_zero_write_was_accepted(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().applied_velocity_mm_s > 2)
    runtime.inject_fault("plant_ignore_zero", True)
    runtime.stop("CANCEL")
    result = terminal(runtime)
    assert result.outcome == "FAILED" and not result.stop_confirmed
    assert runtime.snapshot().stop_progress == "UNKNOWN"
    assert not runtime.reset().accepted
    runtime.inject_fault("plant_ignore_zero", False)


def test_expired_reservation_cannot_start_later(runtime):
    armed(runtime)
    motion = command(runtime, admission_timeout_s=0.02, linear_mm=1)
    assert runtime.reserve(motion).accepted
    sleep(0.04)
    assert not runtime.start_motion(motion).accepted
    assert runtime.plant.snapshot().position_mm == 0


def test_same_id_changed_payload_conflicts(runtime):
    armed(runtime)
    motion = command(runtime, linear_mm=1)
    assert runtime.reserve(motion).accepted
    assert runtime.reserve(replace(motion, linear_mm=2)).code == "ID_CONFLICT"
    runtime.release_reservation(motion.context.command_id)


def test_highwater_survives_bounded_result_eviction():
    runtime = ControlRuntime(RuntimeConfig(history_capacity=2))
    assert runtime.start().accepted
    runtime.activate()
    wait_for(lambda: runtime.snapshot().velocity_valid)
    runtime.set_reference()
    armed(runtime)
    first = command(runtime)
    try:
        for sequence in range(1, 6):
            motion = command(runtime, sequence)
            assert runtime.reserve(motion).accepted
            runtime.release_reservation(motion.context.command_id)
        assert runtime.get_command_status(first.context.command_id).code == "UNKNOWN_COMMAND"
        assert runtime.reserve(first).code == "STALE_SEQUENCE"
    finally:
        runtime.close()


def test_fault_recovery_requires_new_reference_and_explicit_arm(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=20))
    wait_for(lambda: runtime.snapshot().position_mm > 0.1)
    runtime.inject_fault("encoder_disconnect", True)
    assert terminal(runtime).outcome == "FAILED"
    runtime.inject_fault("encoder_disconnect", False)
    wait_for(lambda: runtime.snapshot().linear_valid)
    assert runtime.reset().accepted
    assert runtime.snapshot().state == "DISARMED"
    assert not runtime.snapshot().armed


def test_cancel_before_start_clears_reservation(runtime):
    armed(runtime)
    motion = command(runtime, linear_mm=10)
    assert runtime.reserve(motion).accepted
    runtime.stop("CANCEL", motion.context.command_id)
    assert not runtime.start_motion(motion).accepted
    wait_for(lambda: runtime.snapshot().state == "READY")
    assert runtime.plant.snapshot().position_mm == 0


def test_inactive_runtime_rejects_arm_and_config_change_while_armed(runtime):
    runtime.activate(False)
    assert not runtime.arm(Lease("s", 1, 1, 1)).accepted
    runtime.activate(True)
    armed(runtime)
    assert runtime.configure(replace(RuntimeConfig(), kp=4)).code == "CONFIGURATION_BUSY"


def test_frozen_control_requests_expire_without_late_motion(runtime):
    armed(runtime)
    runtime.inject_fault("control_freeze", True)
    result = runtime.reserve(command(runtime, linear_mm=1))
    assert result.code == "CONTROL_REQUEST_TIMEOUT"
    runtime.inject_fault("control_freeze", False)
    sleep(0.05)
    assert runtime._reservation is None
    assert runtime.plant.snapshot().position_mm == 0


def test_disarm_escalates_an_existing_stop_and_jog_cannot_revive_it(runtime):
    armed(runtime)
    runtime.stop("STOP")
    runtime.stop("DISARM", disarm=True)
    jog = JogCommand(
        CommandContext("session", "task", "jog", 1, 2),
        freshness_token="token",
        axis_mask=1,
        linear_velocity_mm_s=2,
    )
    assert not runtime.jog(jog).accepted
    wait_for(lambda: runtime.snapshot().state == "DISARMED")
    assert not runtime.snapshot().armed


def test_unselected_axis_zero_limits_are_allowed(runtime):
    armed(runtime)
    motion = command(runtime, axis_mask=2, rotation_deg=0, limits=MotionLimits(0, 0, 90, 360))
    run_motion(runtime, motion)
    assert terminal(runtime).outcome == "SUCCEEDED"


def test_config_cannot_claim_new_queue_capacity_was_applied(runtime):
    changed = replace(RuntimeConfig(), revision="new", queue_capacity=2)
    assert runtime.configure(changed).code == "RECONFIGURE_REQUIRED"
    assert runtime.snapshot().applied_config_revision == "simulation-v1"


def test_pulse_receipts_are_bound_to_segment_and_settled_only_once():
    runtime = ControlRuntime()
    old = PulseSegment("old", 5, 0.03, 1, "epoch")
    new = PulseSegment("new", 9, 0.03, 2, "epoch")
    runtime._pulse_segments = {"old": old, "new": new}
    runtime._pending_segment = "new"
    runtime._reserved_pulses = 9
    completion = PulseCompletion("old", 5, 1, "epoch")
    runtime._io_event("pulse_complete", completion)
    runtime._io_event("pulse_complete", completion)
    runtime._consume_io(0)
    assert runtime._confirmed_pulses == 5
    assert runtime._pending_segment == "new"
    assert runtime._reserved_pulses == 9
    runtime._io_event("pulse_aborted", ("new", PulseAbort(3, True)))
    runtime._io_event("pulse_aborted", ("new", PulseAbort(3, True)))
    runtime._consume_io(0)
    assert runtime._confirmed_pulses == 8
    assert runtime._pending_segment is None


def test_wrong_epoch_or_payload_cannot_settle_pulse():
    runtime = ControlRuntime()
    segment = PulseSegment("segment", 5, 0.03, 1, "epoch")
    runtime._pulse_segments["segment"] = segment
    runtime._pending_segment = "segment"
    runtime._io_event("pulse_complete", PulseCompletion("segment", 5, 1, "old-epoch"))
    runtime._consume_io(0)
    assert runtime._confirmed_pulses == 0
    assert runtime._state.state == "FAULT"


def test_bounded_record_ring_reports_eviction():
    runtime = ControlRuntime()
    for _ in range(4002):
        runtime._tick()
    assert len(runtime.records()) == 4000
    assert runtime.snapshot().dropped_records == 2


def test_disarmed_authorization_generation_cannot_be_revived(runtime):
    armed(runtime)
    runtime.stop("DISARM", disarm=True)
    wait_for(lambda: runtime.snapshot().state == "DISARMED")
    assert runtime.arm(Lease("session", 1, 2, 1, "token")).code == "REVOKED_GENERATION"
    assert runtime.arm(Lease("session", 2, 3, 1, "new-token")).accepted


def test_cleanup_failure_cannot_report_successful_close():
    from gca.simulation.devices import simulated_devices

    devices = simulated_devices(RuntimeConfig())
    servo = devices[0]
    original_close = servo.close

    def fail_close():
        original_close()
        raise RuntimeError("simulated cleanup failure")

    servo.close = fail_close
    runtime = ControlRuntime(devices=devices)
    assert runtime.start().accepted
    result = runtime.close()
    assert not result.accepted and result.code == "DEVICE_CLEANUP_FAILED"


def test_reset_during_active_motion_is_rejected(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, linear_mm=20, axis_mask=1))
    assert runtime.reset().code == "RESET_BUSY"
    runtime.stop("CANCEL")
    terminal(runtime)


def test_rotation_only_does_not_require_unselected_encoder_freshness(runtime):
    runtime.inject_fault("encoder_disconnect", True)
    sleep(0.22)
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=2, rotation_deg=2))
    result = terminal(runtime)
    assert result.outcome == "SUCCEEDED"
    assert not runtime.snapshot().linear_valid
    assert runtime.plant.snapshot().position_mm == 0


def test_quantization_cannot_place_edge_outside_soft_limit():
    runtime = ControlRuntime(RuntimeConfig(rotation_min_deg=-0.05, rotation_max_deg=0.05))
    low = runtime._quantizer.angle(runtime._pulse_target(-0.05))
    high = runtime._quantizer.angle(runtime._pulse_target(0.05))
    assert -0.05 <= low <= 0 <= high <= 0.05


def test_saturated_command_queue_cannot_block_stop_latch(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().applied_velocity_mm_s > 1)
    runtime.inject_fault("control_freeze", True)
    # A controlled pause establishes the full-queue condition; no timing guess
    # about which concurrent request happens to win the last slot is needed.
    for _ in range(runtime.config.queue_capacity):
        runtime._commands.put_nowait(("active", True, Event(), []))
    assert runtime._commands.full()
    assert runtime.stop("CANCEL").accepted
    assert runtime.snapshot().stop_progress == "REQUESTED"
    wait_for(lambda: abs(runtime.plant.advance(runtime.clock()).velocity_mm_s) < 0.01, 1.5)
    runtime.inject_fault("control_freeze", False)
    event = terminal(runtime)
    assert event.outcome == "CANCELED" and event.stop_confirmed


def test_critical_io_overflow_latches_fault_and_invalidates_count_evidence():
    runtime = ControlRuntime(RuntimeConfig(queue_capacity=1))
    runtime._rotation_valid = runtime._rotation_referenced = True
    runtime._io_event("acquisition_error", "first")
    runtime._io_event("pulse_complete", PulseCompletion("lost", 5, 1, "epoch"))
    assert runtime._emergency == "IO_EVENT_OVERFLOW"
    assert not runtime._permit
    assert not runtime.snapshot().rotation_estimate_valid
    runtime._tick()
    assert runtime.snapshot().state == "FAULT"
    assert runtime.snapshot().error_code == "IO_EVENT_OVERFLOW"
    assert runtime.drain_events() == []


def test_stop_output_diagnostics_zero_after_ack_without_ack_flood(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().applied_velocity_mm_s > 1)
    zero_receipts = []
    original = runtime._io_event

    def observe(kind, value=None):
        if kind == "zero_applied":
            zero_receipts.append(value)
        original(kind, value)

    runtime._io_event = observe
    assert runtime.stop("CANCEL").accepted
    assert runtime.snapshot().requested_velocity_mm_s == 0
    event = terminal(runtime)
    assert event.stop_confirmed
    assert runtime.snapshot().applied_velocity_mm_s == 0
    assert zero_receipts
    assert all(zero_receipts.count(generation) == 1 for generation in set(zero_receipts))


def test_failed_zero_write_does_not_fabricate_applied_zero(runtime):
    armed(runtime)
    run_motion(runtime, command(runtime, axis_mask=1, linear_mm=30))
    wait_for(lambda: runtime.snapshot().applied_velocity_mm_s > 1)
    runtime.inject_fault("servo_write_error", True)
    event = terminal(runtime)
    assert event.outcome == "FAILED" and not event.stop_confirmed
    state = runtime.snapshot()
    assert state.requested_velocity_mm_s == 0
    assert state.applied_velocity_mm_s > 0
    assert state.stop_progress == "UNKNOWN"


def _fixed_deadline_runtime():
    """Arrange pure control state under a model clock, without worker threads."""
    now = [1_000_000_000]
    runtime = ControlRuntime(clock=lambda: now[0])
    runtime._active_lifecycle = True
    runtime._state = replace(
        runtime._state,
        state="DISARMED",
        configured=True,
        lifecycle_active=True,
        reference_id="reference",
    )
    runtime._estimator.update(EncoderSample(0, 1, now[0], "model-epoch"))
    runtime._estimator.set_reference(0)
    runtime._reference_id = "reference"
    assert runtime._handle("arm", Lease("session", 1, 1, 5, "token"), now[0]).accepted
    motion = MotionCommand(
        CommandContext("session", "task", "fixed-deadline", 1, 1),
        axis_mask=1,
        linear_mm=1,
        end_policy=1,
        freshness_token="token",
        reference_id="reference",
        admission_timeout_s=0.5,
        admission_deadline_ns=1_200_000_000,
    )
    return runtime, now, motion


def test_local_queue_delay_cannot_restart_receiver_admission_ttl():
    runtime, now, motion = _fixed_deadline_runtime()
    done, holder = Event(), []
    runtime._commands.put_nowait(("reserve", motion, done, holder))
    now[0] = 1_210_000_000  # Frame arrived earlier; local queue consumed its budget.
    runtime._tick()
    assert done.is_set()
    assert holder[0].code == "COMMAND_EXPIRED"
    assert runtime._reservation is None and runtime._motion is None
    assert not runtime._permit


def test_reservation_keeps_fixed_deadline_and_late_start_does_not_move():
    runtime, now, motion = _fixed_deadline_runtime()
    now[0] = 1_150_000_000
    assert runtime._handle("reserve", motion, now[0]).accepted
    assert runtime._reservation_deadline == 1_200_000_000  # Not now + 0.5 seconds.
    now[0] = 1_210_000_000
    assert not runtime._handle("motion", motion, now[0]).accepted
    assert runtime._motion is None and not runtime._permit


@pytest.mark.parametrize("invalid", [True, -1, 1.2, "1200000000"])
def test_fixed_admission_deadline_has_strict_monotonic_ns_type(invalid):
    runtime, now, motion = _fixed_deadline_runtime()
    assert (
        runtime._validate_command(replace(motion, admission_deadline_ns=invalid), now[0]).code
        == "INVALID_TIMEOUT"
    )


def test_jog_local_queue_delay_cannot_restart_fixed_deadline():
    runtime, now, motion = _fixed_deadline_runtime()
    jog = JogCommand(
        motion.context,
        linear_velocity_mm_s=1,
        axis_mask=1,
        freshness_token="token",
        ttl_s=0.5,
        admission_deadline_ns=1_200_000_000,
    )
    done, holder = Event(), []
    runtime._commands.put_nowait(("jog", jog, done, holder))
    now[0] = 1_210_000_000
    runtime._tick()
    assert holder[0].code == "COMMAND_EXPIRED"
    assert runtime._jog is None and not runtime._permit


def test_jog_duration_is_capped_by_receiver_fixed_deadline():
    runtime, now, motion = _fixed_deadline_runtime()
    jog = JogCommand(
        motion.context,
        linear_velocity_mm_s=1,
        axis_mask=1,
        freshness_token="token",
        ttl_s=0.5,
        admission_deadline_ns=1_200_000_000,
    )
    now[0] = 1_150_000_000
    assert runtime._handle_jog(jog, now[0]).accepted
    assert runtime._jog_deadline == 1_200_000_000


@pytest.mark.parametrize("invalid", [True, -1, 1.2, "1200000000"])
def test_jog_fixed_deadline_type_is_strict(invalid):
    runtime, now, motion = _fixed_deadline_runtime()
    jog = JogCommand(
        motion.context,
        linear_velocity_mm_s=1,
        axis_mask=1,
        freshness_token="token",
        admission_deadline_ns=invalid,
    )
    assert runtime._handle_jog(jog, now[0]).code == "INVALID_TIMEOUT"


def test_logical_duplicate_ignores_refreshed_receiver_nonce_budgets():
    runtime, now, motion = _fixed_deadline_runtime()
    assert runtime._handle("reserve", motion, now[0]).accepted
    refreshed = replace(motion, admission_deadline_ns=1_300_000_000, admission_timeout_s=0.3)
    assert runtime._handle("reserve", refreshed, now[0]).code == "DUPLICATE_COMMAND"
    assert runtime._motion is None and not runtime._permit


def test_refreshed_nonce_with_changed_logical_target_still_conflicts():
    runtime, now, motion = _fixed_deadline_runtime()
    assert runtime._handle("reserve", motion, now[0]).accepted
    changed = replace(
        motion, linear_mm=2, admission_deadline_ns=1_300_000_000, admission_timeout_s=0.3
    )
    assert runtime._handle("reserve", changed, now[0]).code == "ID_CONFLICT"
    assert runtime._motion is None and not runtime._permit


def test_duplicate_cache_does_not_bypass_actual_permission_validation():
    runtime, now, motion = _fixed_deadline_runtime()
    assert runtime._handle("reserve", motion, now[0]).accepted
    invalid = replace(motion, freshness_token="invalid-token", admission_deadline_ns=1_300_000_000)
    assert runtime._handle("reserve", invalid, now[0]).code == "INVALID_PERMISSION"


def test_stop_between_plan_and_submit_receives_owner_reject_ack(runtime):
    """Pin the queued-but-not-submitted window instead of racing wall sleeps."""
    armed(runtime)
    pause, owner_blocked, release = Event(), Event(), Event()
    original_poll = runtime.pulse.poll
    original_notify = runtime._io_event
    rejected = []
    paused_once = [False]

    def barrier_poll():
        if pause.is_set() and not paused_once[0]:
            paused_once[0] = True
            owner_blocked.set()
            if not release.wait(2.0):
                raise RuntimeError("test pulse-owner barrier timed out")
        return original_poll()

    def observe(kind, value=None):
        if kind == "pulse_rejected":
            rejected.append(value.segment_id)
        original_notify(kind, value)

    runtime.pulse.poll = barrier_poll
    runtime._io_event = observe
    pause.set()
    try:
        assert owner_blocked.wait(0.5)
        motion = command(runtime, axis_mask=2, rotation_deg=20)
        run_motion(runtime, motion)
        wait_for(lambda: runtime._pending_segment is not None)
        segment_id = runtime._pending_segment
        assert runtime._pulse_demand.get()[0].segment_id == segment_id
        assert runtime._reserved_pulses == 0 and runtime._pulse_idle
        assert runtime.stop("CANCEL", motion.context.command_id).accepted
        wait_for(lambda: runtime.snapshot().state == "STOPPING")
        # Stop must retain evidence until the owner has acknowledged rejection.
        assert runtime._pending_segment == segment_id
        assert runtime._pulse_demand.get()[0].segment_id == segment_id
        release.set()
        result = terminal(runtime)
        assert result.outcome == "CANCELED" and result.stop_confirmed
        assert rejected == [segment_id]
        assert runtime._pending_segment is None
        assert runtime.snapshot().confirmed_pulses == 0
        assert runtime.plant.snapshot().mechanical_angle_deg == 0
    finally:
        release.set()


def test_planned_segment_carries_fixed_output_expiry_into_backend():
    runtime, now, _ = _fixed_deadline_runtime()
    runtime._plan_pulse(10, now[0], "finite-window")
    segment, expiry = runtime._pulse_demand.get()
    assert segment.expires_ns == expiry == now[0] + int(runtime.config.output_ttl_s * 1e9)
    assert now[0] + int(segment.duration_s * 1e9) <= segment.expires_ns


@pytest.mark.parametrize("latched_state", ["FAULT", "ESTOP"])
def test_repeated_parking_cannot_clear_confirmed_fault_or_evidence(latched_state):
    runtime = ControlRuntime()
    runtime._state = replace(
        runtime._state,
        state=latched_state,
        armed=False,
        stop_progress="CONFIRMED",
        stop_evidence="MEASURED_ZERO",
        error_code="ORIGINAL_FAULT",
    )
    runtime._begin_stop("DISARM", disarm=True)
    state = runtime.snapshot()
    assert state.state == latched_state and state.error_code == "ORIGINAL_FAULT"
    assert state.stop_progress == "CONFIRMED" and state.stop_evidence == "MEASURED_ZERO"
    assert runtime._stopping is None


def test_stop_can_confirm_output_without_unlatching_fault_until_explicit_reset():
    runtime, now, _ = _fixed_deadline_runtime()
    runtime._state = replace(
        runtime._state,
        state="FAULT",
        armed=False,
        stop_progress="UNKNOWN",
        error_code="ORIGINAL_FAULT",
    )
    runtime._begin_stop("DISARM", disarm=True)
    runtime._servo_sample.put(ServoSample(0, False, now[0], "servo-epoch"))
    runtime._advance_stop(now[0])
    now[0] += 160_000_000
    runtime._advance_stop(now[0])
    assert runtime.snapshot().stop_progress == "CONFIRMED"
    assert runtime.snapshot().state == "FAULT"
    assert runtime.snapshot().error_code == "ORIGINAL_FAULT"
    assert runtime._handle("reset", None, now[0]).accepted
    assert runtime.snapshot().state == "DISARMED"
    assert runtime.snapshot().error_code == ""
