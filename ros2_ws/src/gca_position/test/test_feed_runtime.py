"""Small cross-component runtime checks; only simulated devices are opened."""

import runpy
from dataclasses import replace
from pathlib import Path
from threading import Event
from time import monotonic

import pytest
from gca.domain.models import CommandContext, Lease, MotionCommand, MotionLimits
from gca.domain.servo import MANUAL_SERVO_VELOCITY, POSITION_EXTERNAL_ENCODER
from gca.runtime.feed_axis import FeedAxisRuntime
from gca.servo_config import make_position_settings
from gca_position.assembly import create_position_components


def wait_for(runtime, predicate, timeout=6.0):
    deadline, renew_at = monotonic() + timeout, monotonic() + 0.5
    while monotonic() < deadline:
        if predicate():
            return
        if monotonic() >= renew_at and runtime.snapshot().armed:
            lease = runtime._lease
            assert runtime.renew(replace(lease, sequence=lease.sequence + 1))
            renew_at = monotonic() + 0.5
        Event().wait(0.01)
    raise AssertionError(f"bounded position condition failed: {runtime.snapshot()}")


@pytest.fixture
def runtime():
    root = Path(__file__).resolve().parents[4]
    servo = runpy.run_path(str(root / "config/servo_settings.py"))
    encoder = runpy.run_path(str(root / "config/encoder_settings.py"))
    config = make_position_settings(
        servo["settings_payload"]("simulation"),
        servo["position_settings_payload"](),
        encoder["settings_payload"](),
    )
    drive, position = create_position_components(config, slip_ratio=0.85)
    runtime = FeedAxisRuntime(config, drive=drive, position=position)
    assert runtime.start()
    assert runtime.activate()
    wait_for(runtime, lambda: runtime.snapshot().velocity_valid)
    assert runtime.set_reference(axis_mask=1, linear_mm=12.0)
    wait_for(runtime, lambda: runtime.snapshot().velocity_valid)
    assert (
        runtime.arm(Lease("position-session", 1, 1, 3, "executor", MANUAL_SERVO_VELOCITY)).code
        == "CONTROL_POLICY_UNSUPPORTED"
    )
    assert runtime.arm(Lease("position-session", 1, 1, 3, "executor", POSITION_EXTERNAL_ENCODER))
    yield runtime
    runtime.inject_fault("control_freeze", False)
    runtime.close()


def command(runtime, seq, target, mode=2):
    return MotionCommand(
        CommandContext("position-session", "position", f"motion-{seq}", 1, seq),
        axis_mask=1,
        mode=mode,
        linear_mm=target,
        end_policy=1,
        limits=MotionLimits(max_linear_mm_s=2, max_linear_mm_s2=5),
        freshness_token="executor",
        reference_id=runtime.snapshot().reference_id,
        execution_timeout_s=5,
    )


def begin(runtime, value):
    assert runtime.reserve(value)
    assert runtime.start_motion(value)


def terminal(runtime):
    wait_for(runtime, lambda: runtime._terminal is not None)
    events = runtime.drain_events()
    assert len(events) == 1
    return events[0]


def test_relative_absolute_and_cancel_keep_reference_and_single_task(runtime):
    assert runtime.snapshot().position_mm == pytest.approx(12)
    first = command(runtime, 10, 2)
    begin(runtime, first)
    assert runtime.snapshot().target_linear_mm == pytest.approx(14)
    event = terminal(runtime)
    assert event.outcome == "SUCCEEDED" and event.stop_confirmed and not event.outputs_held
    assert runtime.snapshot().servo_stop_confirmed and runtime.snapshot().linear_stop_confirmed
    assert abs(runtime.snapshot().position_mm - 14) <= 1
    assert runtime.commit_terminal(event.command_id, event.outcome)
    assert runtime.reserve(first).code == "DUPLICATE_COMMAND"
    second = command(runtime, 11, 12, mode=1)
    begin(runtime, second)
    assert runtime.snapshot().target_linear_mm == 12
    event = terminal(runtime)
    assert event.outcome == "SUCCEEDED"
    assert abs(runtime.snapshot().position_mm - 12) <= 1
    # A cancel arriving after success is prepared but before ROS commits it
    # must replace the pending success, rather than leave the Action waiting.
    assert runtime.stop("ACTION_CANCEL", command_id=event.command_id)
    wait_for(runtime, lambda: runtime._terminal.outcome == "CANCELED")
    canceled = runtime.drain_events()
    assert len(canceled) == 1 and canceled[0].stop_confirmed
    assert runtime.commit_terminal(event.command_id, "CANCELED")
    assert runtime.snapshot().state == "READY" and runtime.snapshot().armed


def test_original_encoder_age_and_fault_stop_independent_of_control(runtime):
    begin(runtime, command(runtime, 10, 10))
    wait_for(
        runtime,
        lambda: (
            runtime._demand.get() is not None
            and abs(runtime.simulation_evidence()["sample"]["velocity_rpm"]) > 0.2
        ),
    )
    now, demand = runtime.clock(), runtime._demand.get()
    recent = replace(demand, encoder_ns=now - 90_000_000)
    # Ninety-ms encoder evidence may authorize TX with its own complete 80-ms
    # ACK budget. The actual pre-TX guard still checks the original sample age.
    assert runtime._valid_output(recent, now)
    with runtime._velocity_write_guard(recent, now + 80_000_000):
        pass
    assert not runtime._valid_output(replace(demand, encoder_ns=now - 101_000_000), now)
    for changed in (
        replace(demand, encoder_epoch="previous-encoder"),
        replace(demand, reference_id="previous-reference"),
        replace(demand, generation=demand.generation - 1),
        replace(demand, stop_serial=demand.stop_serial - 1),
    ):
        assert not runtime._valid_output(changed, now)
    assert runtime.inject_fault("control_freeze", True)
    frozen_seq = runtime.snapshot().state_seq
    runtime._position.acquisition.reader.fail_read = True
    wait_for(runtime, lambda: not runtime.simulation_evidence()["sample"]["enabled"], timeout=0.45)
    assert runtime.snapshot().state_seq == frozen_seq
    assert runtime.inject_fault("control_freeze", False)
    event = terminal(runtime)
    assert event.outcome == "FAILED" and event.stop_confirmed
    assert runtime.snapshot().servo_stop_confirmed
    assert not runtime.snapshot().linear_stop_confirmed
    assert runtime.snapshot().state == "FAULT"


def test_relative_target_is_fixed_at_admission(runtime):
    value = command(runtime, 10, 2)
    assert runtime.reserve(value)
    assert runtime._reservation_target == 14
    # Explicitly vary the feedback snapshot between admission and start;
    # the admitted relative goal remains exactly 14 mm.
    assert runtime.inject_fault("control_freeze", True)
    runtime._position_feedback = replace(runtime._position_feedback, position_mm=12.2)
    assert runtime._handle_position("motion", value, runtime.clock())
    assert runtime._motion_target == 14
    assert runtime.inject_fault("control_freeze", False)
    assert runtime.stop("ACTION_CANCEL", command_id=value.context.command_id)
    event = terminal(runtime)
    assert event.outcome == "CANCELED" and event.stop_confirmed
