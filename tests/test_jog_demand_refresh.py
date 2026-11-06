"""Control progress renews accepted permission, never a source sample or PI datum."""

import pytest
from gca.config import RuntimeConfig
from gca.domain.models import CommandContext
from gca.simulation.teleop_report import RuntimeModelRig


def rig():
    return RuntimeModelRig(RuntimeConfig(output_ttl_s=0.04))


def test_no_new_frame_or_feedback_can_hold_a_still_valid_output_without_reintegrating():
    r = rig()
    try:
        assert r.jog(2.0, 0.0).accepted
        r.advance(0.04)
        runtime = r.runtime
        runtime.encoder.frozen = True
        demand = runtime._velocity_demand.get()
        integral = runtime._jog_velocity.integral
        filtered = runtime._jog_velocity.filtered
        last_output = runtime._last_output_ns
        fixed_input, fixed_nonce = runtime._input_deadline, runtime._jog_deadline
        r.advance(0.08)  # Beyond old output TTL, within source/nonce and feedback budgets.
        current = runtime._velocity_demand.get()
        assert runtime.snapshot().state == "EXECUTING" and runtime._permit
        assert current.expires_ns > demand.expires_ns
        assert current.expires_ns <= min(fixed_input, fixed_nonce, runtime._lease_deadline)
        assert (current.velocity, current.seq, current.feedback_ns, current.stop_serial) == (
            demand.velocity,
            demand.seq,
            demand.feedback_ns,
            demand.stop_serial,
        )
        assert runtime._jog_velocity.integral == integral
        assert runtime._jog_velocity.filtered == filtered
        assert runtime._last_output_ns == last_output
        assert runtime.plant.snapshot().accepted_velocity_mm_s == demand.velocity
    finally:
        r.close()


def test_real_encoder_expiry_cannot_be_renewed_by_live_source_frames():
    r = rig()
    try:
        r.run(0.2, lambda _: (2.0, 0.0))
        runtime = r.runtime
        runtime.encoder.frozen = True
        original = runtime._estimator.state.received_ns
        for index in range(50):
            if index % 8 == 0 and runtime.snapshot().state != "FAULT":
                r.jog(2.0, 0.0)
            r.step()
        assert runtime._estimator.state.received_ns == original
        assert runtime.snapshot().state == "FAULT"
        assert runtime.plant.snapshot().accepted_velocity_mm_s == 0.0
    finally:
        r.close()


def test_control_freeze_cannot_refresh_output_or_original_input_deadline():
    r = rig()
    try:
        r.jog(2.0, 0.0)
        r.advance(0.04)
        demand = r.runtime._velocity_demand.get()
        original_input = r.runtime._input_deadline
        r.control_paused = True
        r.advance(0.08)
        assert r.runtime._velocity_demand.get() == demand
        assert r.runtime._input_deadline == original_input
        assert r.runtime.plant.snapshot().accepted_velocity_mm_s == 0.0
    finally:
        r.close()


@pytest.mark.parametrize("desired", [0.0, -2.0, 1.0])
def test_neutral_reverse_or_reduced_intent_cannot_keep_refreshing_old_positive_velocity(desired):
    r = rig()
    try:
        r.run(0.3, lambda _: (2.0, 0.0))
        runtime = r.runtime
        runtime.encoder.frozen = True
        original_sample = runtime._estimator.state.seq
        integral = runtime._jog_velocity.integral
        before = runtime._velocity_demand.get().velocity
        assert before > 0 and r.jog(desired, 0.0).accepted
        r.advance(0.08)  # Includes braking the prior PI-corrected setpoint through zero.
        after = runtime.plant.snapshot().accepted_velocity_mm_s
        assert after == pytest.approx(desired)
        assert abs(after - before) <= runtime.config.max_linear_mm_s2 * 0.08 + 1e-9
        assert runtime._estimator.state.seq == original_sample
        assert runtime._jog_velocity.integral == integral
    finally:
        r.close()


def test_stop_serial_and_revoked_permission_cannot_be_reopened_by_refresh():
    r = rig()
    try:
        r.run(0.2, lambda _: (2.0, 0.0))
        runtime = r.runtime
        old = runtime._velocity_demand.get()
        runtime.stop("B", context=CommandContext("model-session", "manual-model", "B", 1, 1000))
        runtime._velocity_demand.put(old)  # A computed old demand racing the latch.
        runtime._refresh_jog_demand(r.clock())
        assert runtime._velocity_demand.get() == old
        r.advance(0.02)
        assert not runtime._permit
        assert runtime.plant.snapshot().accepted_velocity_mm_s == 0.0
    finally:
        r.close()
