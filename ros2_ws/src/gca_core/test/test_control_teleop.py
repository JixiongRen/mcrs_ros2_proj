"""JF01–07/JT core evidence: real runtime -> accepted simulated I/O -> plant."""

from threading import Event

import pytest
from gca.algorithms.jog_velocity import JogVelocityController
from gca.config import RuntimeConfig
from gca.domain.models import CommandContext, JogCommand, Lease, PulseSegment
from gca.simulation.teleop_report import RuntimeModelRig, scripted_quality


@pytest.mark.parametrize("direction", [1, -1])
@pytest.mark.parametrize("slip", [0.0, 0.1, 0.25])
def test_velocity_pi_tracks_external_plant_slip_and_neutral_has_no_position_debt(direction, slip):
    rig = RuntimeModelRig(RuntimeConfig(simulation_slip=slip))
    try:
        rows = rig.run(5, lambda t: (direction * 2 if t < 3 else 0, 0))
        steady = [r["plant_velocity_mm_s"] for r in rows if 2 <= r["model_s"] <= 2.5]
        assert abs(sum(steady) / len(steady) - direction * 2) <= 0.1
        neutral = [r["plant_position_mm"] for r in rows if r["model_s"] >= 4]
        assert max(neutral) - min(neutral) <= 0.02
        assert rows[-1]["velocity_integral_mm_s"] == 0
        assert rows[-1]["accepted_velocity_mm_s"] == 0
        assert all(r["state"] != "HOLDING" for r in rows)
    finally:
        rig.close()


def test_pi_actual_feedback_dt_once_only_receipt_and_antiwindup():
    c = JogVelocityController(kp=0, ki=2, kaw=3, integral_limit=1, filter_s=0)
    c.shape(2, 100, 0.02)
    request = c.request(0, 0.03)
    c.commit(1, 1, request, 1, 0.03)
    expected = 2 * 2 * 0.03 + 3 * (1 - request) * 0.03
    assert c.integral == pytest.approx(expected)
    assert c.applied(1, 1, 0.5)
    assert c.integral == pytest.approx(expected - 3 * 0.5 * 0.03)
    before = c.integral
    assert not c.applied(1, 1, 0.5)
    c.commit(1, 1, request, 1, 0.03)
    assert c.integral == before
    for i in range(2, 500):
        value = c.request(0, 0.01)
        c.commit(i, 1, value, 0, 0.01)
    assert abs(c.integral) <= 1
    c.request(1, 0.02, neutral=True)
    assert c.integral == 0
    with pytest.raises(ValueError):
        c.request(0, 0)
    with pytest.raises(ValueError):
        c.shape(1, 100, 0.3)


@pytest.mark.parametrize("rate", [0.1, -0.1])
def test_subpulse_rotation_survives_many_segments_within_one_edge(rate):
    rig = RuntimeModelRig()
    try:
        rows = rig.run(10, lambda t: (0, rate))
        rows.extend(rig.run(0.5, lambda t: (0, 0)))
        ideal = rate * 10 / rig.runtime._quantizer.angle(1)
        assert 0 < abs(rows[-1]["confirmed_pulses"])
        assert abs(rows[-1]["confirmed_pulses"] - ideal) <= 1
        assert (
            abs(rows[-1]["independent_mechanical_angle_deg"] - rows[-1]["estimated_angle_deg"])
            < 1e-9
        )
        assert all(abs(r["phase_fraction_pulses"]) <= 0.500001 for r in rows)
    finally:
        rig.close()


@pytest.mark.parametrize("rate", [30, -30])
def test_constant_rotation_does_not_lose_wave_handoff_time(rate):
    rig = RuntimeModelRig()
    try:
        rows = rig.run(4, lambda t: (0, rate))
        window = [r for r in rows if 2 <= r["model_s"] <= 3]
        speed = (window[-1]["estimated_angle_deg"] - window[0]["estimated_angle_deg"]) / (
            window[-1]["model_s"] - window[0]["model_s"]
        )
        assert abs(speed - rate) <= 0.05 * abs(rate)
    finally:
        rig.close()


def test_first_input_and_reentry_start_with_real_dt_not_old_timestamp():
    rig = RuntimeModelRig()
    try:
        assert rig.jog(20, 180).accepted
        rig.advance(0.02)
        assert abs(rig.runtime._jog_velocity.target) <= 2.000001
        assert abs(rig.runtime._jog_rotation.commanded_rate) <= 14.400001
        rig.runtime.stop(
            "B", context=CommandContext("model-session", "manual-model", "stop", 1, 100)
        )
        rig.advance(0.5)
        assert rig.runtime._jog is None and not rig.runtime._permit
        assert rig.runtime.snapshot().stop_progress == "CONFIRMED"
        # A stale command remains fenced after parking; STOP is no automatic resume.
        command = JogCommand(
            CommandContext("model-session", "manual-model", "late", 1, 99),
            1,
            0,
            3,
            "model-token",
            quality=scripted_quality(rig.clock(), 101),
        )
        assert rig.runtime._handle_jog(command, rig.clock()).code == "STOP_FENCED"
    finally:
        rig.close()


def test_missing_quality_and_fixed_source_expiry_fail_closed_with_control_paused():
    rig = RuntimeModelRig()
    try:
        c = JogCommand(
            CommandContext("model-session", "manual-model", "bad", 1, 1), 1, 0, 3, "model-token"
        )
        assert rig.runtime._handle_jog(c, rig.clock()).code == "MANUAL_QUALITY_REQUIRED"
        assert rig.jog(2, 0, quality=scripted_quality(rig.clock(), 2, age_s=0.06)).accepted
        rig.advance(0.04)
        rig.control_paused = True
        rig.advance(0.04)
        assert rig.runtime.plant.snapshot().accepted_velocity_mm_s == 0
        assert not rig.runtime._valid_output(
            rig.clock() + 1_000_000_000,
            rig.runtime._output_generation,
            stop_serial=rig.runtime.stop_serial,
        )
    finally:
        rig.close()


def test_stop_rejects_already_queued_jog_and_arm_without_fifo_dependency():
    rig = RuntimeModelRig()
    try:
        r = rig.runtime
        done, holder = Event(), []
        oldserial = r.stop_serial
        c = JogCommand(
            CommandContext("model-session", "manual-model", "pending", 1, 2),
            2,
            0,
            3,
            "model-token",
            quality=scripted_quality(rig.clock(), 2),
        )
        r._commands.put_nowait(("jog", c, done, holder, oldserial))
        assert r.stop(
            "B", context=CommandContext("model-session", "manual-model", "B", 1, 3)
        ).accepted
        r._tick()
        assert done.is_set() and holder[0].code == "STOP_SUPERSEDED"
        assert r._jog is None and not r._permit
        assert not r._handle("arm", (Lease("new", 1, 1, 1, "new"), oldserial), rig.clock()).accepted
    finally:
        rig.close()


def test_stop_during_quality_validation_cannot_reopen_output_gate():
    rig = RuntimeModelRig()
    try:
        r = rig.runtime
        original = r._quality_tracker.validate

        def stopping_validate(*args, **kwargs):
            deadline = original(*args, **kwargs)
            r.stop("B", context=CommandContext("model-session", "manual-model", "B", 1, 2))
            return deadline

        r._quality_tracker.validate = stopping_validate
        result = rig.jog(2, 0)
        assert result.code == "STOP_SUPERSEDED"
        assert not r._permit and r._jog is None
    finally:
        rig.close()


def test_unknown_abort_invalidates_rotation_reference_and_locks_axis_mask3():
    rig = RuntimeModelRig()
    try:
        assert rig.jog(1, 30).accepted
        rig.advance(0.02)
        assert rig.runtime._pending_segment is not None
        rig.runtime.stop(
            "LT_RELEASE", context=CommandContext("model-session", "manual-model", "LT", 1, 100)
        )
        rig.advance(0.5)
        assert not rig.runtime.snapshot().rotation_reference_valid
        c = JogCommand(
            CommandContext("model-session", "manual-model", "again", 1, 101),
            1,
            0,
            3,
            "model-token",
            quality=scripted_quality(rig.clock(), 101),
        )
        assert rig.runtime._handle_jog(c, rig.clock()).code == "ROTATION_REFERENCE_INVALID"
    finally:
        rig.close()


def test_velocity_step_reverse_and_limits_never_request_historical_position():
    rig = RuntimeModelRig(RuntimeConfig(linear_min_mm=-1, linear_max_mm=1))
    try:
        rows = rig.run(3, lambda t: (2 if t < 1.5 else -2, 0))
        assert max(r["plant_position_mm"] for r in rows) <= 1.02
        assert min(r["plant_position_mm"] for r in rows) >= -1.02
        assert rows[-1]["plant_velocity_mm_s"] < 0
        assert rig.runtime._hold is None
    finally:
        rig.close()


def test_edge_time_oracle_matches_integrated_limited_phase_with_microsecond_budget():
    rig = RuntimeModelRig()
    try:
        profiles = []
        r = rig.runtime
        old = r.pulse.submit

        def collect(segment):
            profiles.append((segment, dict(r._diagnostics)))
            return old(segment)

        r.pulse.submit = collect
        rig.run(1.2, lambda t: (0, 30 if t < 0.5 else -30))
        assert any(s.signed_pulses > 0 for s, _ in profiles)
        assert any(s.signed_pulses < 0 for s, _ in profiles)
        c = rig.config
        motion_s = (
            c.pulse_segment_s
            - (c.direction_setup_us + max(c.direction_hold_us, c.pulse_low_us) + c.pulse_high_us)
            / 1e6
        )
        for segment, d in profiles:
            start, end = d["rotation_profile_start_deg_s"], d["rotation_profile_end_deg_s"]
            assert abs(start) <= c.max_rotation_deg_s and abs(end) <= c.max_rotation_deg_s
            assert abs(end - start) <= c.max_rotation_deg_s2 * motion_s + 1e-9
            assert start * end >= -1e-9
            assert len(segment.edge_offsets_us) == abs(segment.signed_pulses)
            sign = 1 if segment.signed_pulses > 0 else -1
            scale = r._quantizer.pulses_per_deg
            # Independent integrated phase at each emitted integer-us edge.
            for index, us in enumerate(segment.edge_offsets_us):
                t = (us - c.direction_setup_us) / 1e6
                phase = d["rotation_phase_start_pulses"] + scale * (
                    start * t + 0.5 * (end - start) / motion_s * t * t
                )
                assert (
                    abs(phase - sign * (index + 0.5))
                    <= scale * c.max_rotation_deg_s * 0.500001e-6 + 1e-8
                )
    finally:
        rig.close()


def test_duplicate_feedback_does_not_integrate_velocity_pi_twice():
    rig = RuntimeModelRig(RuntimeConfig(simulation_slip=0.25))
    try:
        rig.run(0.5, lambda t: (2, 0))
        r = rig.runtime
        before = r._jog_velocity.integral
        sequence = r._estimator.state.seq
        # Reuse the identical mailbox sample while changing no sample timestamp.
        rig.clock.ns += 1_000_000
        r._tick()
        assert r._estimator.state.seq == sequence
        assert r._jog_velocity.integral == before
    finally:
        rig.close()


def test_missed_steps_are_independent_truth_not_detected_by_open_loop_reference():
    rig = RuntimeModelRig()
    try:
        rig.runtime.plant.missed_step_fraction = 0.25
        rows = rig.run(1, lambda t: (0, 30))
        rows.extend(rig.run(0.5, lambda t: (0, 0)))
        s = rig.runtime.snapshot()
        assert s.rotation_estimate_valid and s.rotation_reference_valid
        assert rows[-1]["independent_mechanical_angle_deg"] == pytest.approx(0.75 * s.rotation_deg)
        assert abs(s.rotation_deg - rows[-1]["independent_mechanical_angle_deg"]) > 1
    finally:
        rig.close()


def test_simulated_profile_consumes_only_actual_rising_edges_not_uniform_time_fraction():
    rig = RuntimeModelRig()
    try:
        p = rig.runtime.pulse
        p.abort()
        start = rig.clock()
        s = PulseSegment("edge-oracle", 3, 0.03, 1, p.epoch, edge_offsets_us=(1000, 12000, 26000))
        p.submit(s)
        for delta, count in (
            (999_000, 0),
            (1_000_000, 1),
            (11_999_000, 1),
            (12_000_000, 2),
            (26_000_000, 3),
        ):
            rig.clock.ns = start + delta
            assert p.poll() == []
            assert rig.runtime.plant.snapshot().mechanical_angle_deg == pytest.approx(
                count * rig.runtime._quantizer.angle(1)
            )
        rig.clock.ns = start + 30_000_000
        assert p.poll()[0].signed_pulses == 3
        assert p.poll() == []
    finally:
        rig.close()


def test_jog_reentry_after_long_idle_has_fresh_initial_budget():
    rig = RuntimeModelRig()
    try:
        rig.advance(2)
        assert rig.jog(20, 180).accepted
        rig.advance(0.02)
        assert rig.runtime._jog_velocity.target <= 2.000001
        assert rig.runtime._jog_rotation.commanded_rate <= 14.400001
    finally:
        rig.close()


def test_reverse_discards_old_whole_pulse_debt_but_keeps_bounded_deceleration():
    rig = RuntimeModelRig()
    try:
        rig.run(0.4, lambda t: (0, 30))
        p = rig.runtime._jog_rotation
        p.reference_phase += 1000  # Controlled late-wave debt, never physical motion.
        prior = p.wave_rate
        assert rig.jog(0, -30).accepted
        rig.advance(0.04)
        assert p.wave_rate <= prior
        assert abs(p.reference_phase - rig.runtime._confirmed_pulses) < 100
        rows = rig.run(0.5, lambda t: (0, -30))
        assert rows[-1]["profile_rate_deg_s"] < 0
    finally:
        rig.close()


def test_same_owner_explicit_reference_and_new_arm_restores_after_unknown_abort():
    rig = RuntimeModelRig()
    try:
        r = rig.runtime
        assert rig.jog(0, 30).accepted
        rig.advance(0.02)
        r.stop("LT", context=CommandContext("model-session", "manual-model", "LT", 1, 20))
        rig.advance(0.5)
        assert not r.snapshot().rotation_reference_valid
        assert r.stop("DISARM", disarm=True).accepted
        rig.advance(0.3)
        assert r.snapshot().state == "DISARMED"
        assert r._handle("reference", (3, 0.0, 0.0), rig.clock()).accepted
        assert (
            r._handle("arm", Lease("model-session", 1, 2, 5, "model-token"), rig.clock()).code
            == "REVOKED_GENERATION"
        )
        assert r._handle(
            "arm", (Lease("model-session", 2, 1, 5, "new-token"), r.stop_serial), rig.clock()
        ).accepted
        c = JogCommand(
            CommandContext("model-session", "manual-model", "new-LT", 2, 21),
            0,
            1,
            3,
            "new-token",
            quality=scripted_quality(rig.clock(), 21, epoch="new-reference"),
        )
        assert r._handle_jog(c, rig.clock()).accepted
        rig.advance(0.04)
        assert r.snapshot().state == "EXECUTING"
    finally:
        rig.close()


def test_fixed_seed_repeats_model_trace_without_wall_clock_or_uuid_dependence():
    traces = []
    for _ in range(2):
        rig = RuntimeModelRig(
            RuntimeConfig(simulation_noise_mm=0.005, simulation_encoder_delay_s=0.02)
        )
        try:
            traces.append(rig.run(2, lambda t: (2 if t < 1 else -1, 0.1)))
        finally:
            rig.close()
    assert traces[0] == traces[1]


@pytest.mark.parametrize("target", [-0.25, 0.25])
@pytest.mark.parametrize("deadband", [0.01, 0.5])
def test_jt13_independent_plant_deadband_crossing_uses_feedback(target, deadband):
    from gca.simulation.teleop_report import deadband_case

    rows, metrics, _ = deadband_case(target, deadband)
    assert metrics["passed"], metrics
    window = [r for r in rows if 2 <= r["model_s"] <= 2.5]
    assert abs(sum(r["plant_velocity_mm_s"] for r in window) / len(window) - target) <= 0.04
    assert all(r["plant_deadband_mm_s"] == deadband for r in rows)
    if deadband > abs(target):
        assert any(abs(r["accepted_velocity_mm_s"]) > deadband for r in rows)
        assert abs(rows[-1]["plant_position_mm"]) > 0.1
    assert metrics["neutral_drift_mm"] <= 0.02
    assert rows[-1]["velocity_integral_mm_s"] == 0


@pytest.mark.parametrize("target", [-2.0, 2.0])
def test_jt14_actual_actuator_saturation_then_release_unwinds_velocity_pi(target):
    from gca.simulation.teleop_report import saturation_case

    rows, metrics, _ = saturation_case(target)
    assert metrics["passed"], metrics
    capped = [r for r in rows if 0.4 <= r["model_s"] < 1]
    assert all(abs(r["accepted_velocity_mm_s"]) <= 1 for r in capped)
    assert all(r["plant_velocity_limit_mm_s"] == 1 for r in capped)
    assert any(abs(r["plant_velocity_mm_s"]) > 0.9 for r in capped)
    recovered = [r for r in rows if 3 <= r["model_s"] <= 3.5]
    assert abs(sum(r["plant_velocity_mm_s"] for r in recovered) / len(recovered) - target) <= 0.1
    assert all(r["plant_velocity_limit_mm_s"] == 30 for r in recovered)
    assert all(abs(r["velocity_integral_mm_s"]) <= 0.1 for r in recovered)
    assert max(abs(r["velocity_integral_mm_s"]) for r in rows) <= 10
    assert metrics["neutral_drift_mm"] <= 0.02


@pytest.mark.parametrize(
    "rate,acceleration", [(0.0, 0.0), (0.0, 10.0), (0.0, -10.0), (4.0, 0.0), (-4.0, 0.0)]
)
def test_half_phase_exact_start_is_earliest_crossing(rate, acceleration):
    from gca.algorithms.jog_rotation import JogRotationPlanner

    assert JogRotationPlanner._crossing(0.5, 0.5, rate, acceleration, 0.03) == 0


def test_stationary_phase_cannot_invent_a_different_crossing():
    from gca.algorithms.jog_rotation import JogRotationPlanner

    with pytest.raises(ValueError, match="stationary"):
        JogRotationPlanner._crossing(0.5, 0.0, 0.0, 0.0, 0.03)


@pytest.mark.parametrize("direction", [-1, 1])
def test_real_subthreshold_acceleration_and_phase_are_not_treated_as_zero(direction):
    from gca.algorithms.jog_rotation import JogRotationPlanner

    target, acceleration = direction * 1e-25, direction * 1e-20
    t = JogRotationPlanner._crossing(target, 0.0, 0.0, acceleration, 0.03)
    assert 0 < t < 0.03
    assert 0.5 * acceleration * t * t == pytest.approx(target, rel=1e-12, abs=0)


@pytest.mark.parametrize("fraction", [-0.5, 0.5])
@pytest.mark.parametrize("direction", [-1, 1])
def test_runtime_half_phase_from_rest_produces_ordered_edges_with_independent_oracle(
    fraction, direction
):
    rig = RuntimeModelRig()
    try:
        r, captures = rig.runtime, []
        original_submit = r.pulse.submit

        def capture(segment):
            captures.append((segment, dict(r._diagnostics)))
            return original_submit(segment)

        r.pulse.submit = capture
        assert rig.jog(0, direction * 10).accepted
        # Reachable planner state: a previous signed rounding leaves exactly
        # half an edge; no count was reserved or declared confirmed by this.
        p = r._jog_rotation
        p.fraction = p.reference_phase = fraction
        assert p.wave_rate == 0 and r._pending_segment is None
        rig.advance(0.04)
        assert captures
        segment, d = captures[0]
        edges = segment.edge_offsets_us
        assert len(edges) >= 2
        assert segment.signed_pulses * direction > 0
        assert all(b > a for a, b in zip(edges, edges[1:]))
        if fraction * direction == 0.5:
            assert edges[0] == rig.config.direction_setup_us
        else:
            assert edges[0] > rig.config.direction_setup_us
        c = rig.config
        motion_s = (
            c.pulse_segment_s
            - (c.direction_setup_us + max(c.direction_hold_us, c.pulse_low_us) + c.pulse_high_us)
            / 1e6
        )
        start, end = d["rotation_profile_start_deg_s"], d["rotation_profile_end_deg_s"]
        assert start == 0
        for index, us in enumerate(edges):
            t = (us - c.direction_setup_us) / 1e6
            phase = fraction + r._quantizer.pulses_per_deg * (
                start * t + 0.5 * (end - start) / motion_s * t * t
            )
            assert (
                abs(phase - direction * (index + 0.5))
                <= r._quantizer.pulses_per_deg * c.max_rotation_deg_s * 0.500001e-6 + 1e-8
            )
        assert r.snapshot().state == "EXECUTING"
    finally:
        rig.close()


def test_fine_rotation_passes_only_against_independently_integrated_limited_rate():
    from gca.simulation.teleop_report import DEFAULT_CRITERIA, fine_rotation_metrics

    rows = [
        {"model_s": 0.0, "limited_rotation_deg_s": 0.1, "confirmed_pulses": 0},
        {"model_s": 10.0, "limited_rotation_deg_s": 0.1, "confirmed_pulses": 1},
    ]
    metrics = fine_rotation_metrics(rows, 10.0, 10.0, 1.0, DEFAULT_CRITERIA)
    assert metrics["independent_limited_integral_edges"] == 1
    assert metrics["ideal_input_integral_edges"] == 100
    assert metrics["count_error_edges"] == 0 and metrics["passed"]
    rows[-1]["confirmed_pulses"] = 0
    assert not fine_rotation_metrics(rows, 0.1, 10.0, 1.0, DEFAULT_CRITERIA)["passed"]


@pytest.fixture(scope="module")
def short_v3_long_model():
    from gca.simulation.teleop_report import long_sampling_case

    return long_sampling_case(60.0)


def test_v3_two_short_periodic_runs_validate_recorded_hash_drift_and_truth(short_v3_long_model):
    from gca.simulation.teleop_report import numerical_trace_hash

    runs, metrics = short_v3_long_model
    assert metrics["passed"], metrics
    assert metrics["full_30_minutes"] is False
    assert metrics["repeat_runs"] == len(runs) == 2
    assert metrics["sampled_trace_sha256"][0] == metrics["sampled_trace_sha256"][1]
    for (rows, _), analysis, digest in zip(
        runs, metrics["run_metrics"], metrics["sampled_trace_sha256"]
    ):
        assert numerical_trace_hash(rows) == digest
        assert analysis["actual_stimulus_model_duration_s"] == 60
        assert analysis["actual_observed_model_duration_s"] == 61.5
        assert analysis["retained_rows"] == analysis["expected_recorded_rows"] == len(rows)
        assert analysis["sample_stride_s"] == 0.1
        assert [p["model_s"] for p in analysis["steady_period_endpoints"]] == [30, 60]
        assert analysis["period_interval_count"] == 1
        assert all(analysis["checks"].values())
        assert analysis["neutral_linear_drift_mm"] <= 0.02
        assert analysis["max_completed_count_truth_error_pulses"] <= 1


def test_long_acceptance_detects_real_cycle_drift_instead_of_only_small_fraction(
    short_v3_long_model,
):
    from gca.simulation.teleop_report import DEFAULT_CRITERIA, long_trajectory_metrics

    rows = [dict(row) for row in short_v3_long_model[0][0][0]]
    for row in rows:
        if row["model_s"] == 60:
            row["plant_position_mm"] += 0.05
    metrics = long_trajectory_metrics(rows, 60.0, RuntimeConfig(), DEFAULT_CRITERIA)
    assert not metrics["checks"]["per_cycle_linear_drift"]
    assert not metrics["checks"]["cumulative_linear_drift"]
    assert not metrics["passed"]


def test_trace_hash_ignores_external_identity_but_includes_actual_plant_motion(short_v3_long_model):
    from gca.simulation.teleop_report import numerical_trace_hash

    rows = [dict(row) for row in short_v3_long_model[0][0][0]]
    original = numerical_trace_hash(rows)
    rows[0].update(boot_id="another-host", segment_id="another-UUID", host_wall_s=12345)
    assert numerical_trace_hash(rows) == original
    rows[-1]["plant_position_mm"] += 0.001
    assert numerical_trace_hash(rows) != original


def test_model_junit_keeps_clock_and_actual_completion_separate_from_wall_time(tmp_path):
    import xml.etree.ElementTree as ET

    from gca.simulation.teleop_report import DEFAULT_CRITERIA, write_model_junit

    cases = [
        {"scenario": "one", "passed": True},
        {
            "scenario": "long_sampling",
            "passed": False,
            "full_30_minutes": True,
            "repeat_runs": 2,
            "error": "drift threshold failed",
        },
    ]
    write_model_junit(tmp_path, cases, DEFAULT_CRITERIA, 1800.0)
    root = ET.parse(tmp_path / "junit.xml").getroot()
    properties = {p.attrib["name"]: p.attrib["value"] for p in root.find("properties")}
    assert (
        root.attrib["tests"] == "2"
        and root.attrib["failures"] == "1"
        and root.attrib["skipped"] == "0"
    )
    assert (
        properties["clock"] == "model_time"
        and properties["criterion_version"] == "teleop-controllability-v3"
    )
    assert properties["full_duration"] == "true"
    assert all("time" not in case.attrib for case in root.findall("testcase"))
    assert root.findall("testcase")[1].find("failure") is not None


def test_model_exception_records_all_required_cases_failed_not_old_pass(tmp_path, monkeypatch):
    import json
    import xml.etree.ElementTree as ET

    from gca.simulation import teleop_report

    def fail(*args, **kwargs):
        raise RuntimeError("controlled model failure")

    monkeypatch.setattr(teleop_report, "_run_report", fail)
    result = teleop_report.run(tmp_path)
    assert not result["passed"] and not result["full_acceptance_model_duration"]
    assert len(result["scenarios"]) == 23
    root = ET.parse(tmp_path / "junit.xml").getroot()
    assert root.attrib["failures"] == "23" and root.attrib["skipped"] == "0"
    assert all(case.find("failure") is not None for case in root.findall("testcase"))
    properties = {p.attrib["name"]: p.attrib["value"] for p in root.find("properties")}
    assert properties["full_duration"] == "false"
    assert json.loads((tmp_path / "summary.json").read_text())["passed"] is False


@pytest.mark.parametrize("direction", [-1, 1])
def test_jog_rotation_quantized_candidate_respects_boundary_reference(direction):
    """A monotonic one-edge candidate cannot reserve across a sub-edge margin."""
    config = RuntimeConfig(rotation_min_deg=-0.1, rotation_max_deg=0.1)
    rig = RuntimeModelRig(config)
    try:
        r = rig.runtime
        # A real reference transform places the current stationary coordinate
        # near the software limit; independent physical angle is never assigned.
        edge_deg = 360 * config.catheter_per_motor_ratio / config.pulses_per_motor_rev
        reference_deg = direction * (0.1 - edge_deg / 8)
        assert r.stop("BOUNDARY_REFERENCE_SETUP", disarm=True).accepted
        rig.advance(0.3)
        assert r.snapshot().state == "DISARMED"
        assert r._handle("reference", (3, 0.0, reference_deg), rig.clock()).accepted
        assert r._handle(
            "arm", Lease("model-session", 2, 1, 5, "boundary-token"), rig.clock()
        ).accepted
        command = JogCommand(
            CommandContext("model-session", "boundary", "candidate", 2, 1),
            0,
            direction * 2,
            3,
            "boundary-token",
            quality=scripted_quality(rig.clock(), 1, epoch="boundary-input"),
        )
        assert r._handle_jog(command, rig.clock()).accepted
        p = r._jog_rotation
        p.commanded_rate = p.requested_rate = direction * 2
        assert p.wave_rate == 0 and p.fraction == 0
        motion_s = (
            config.pulse_segment_s
            - (
                config.direction_setup_us
                + max(config.direction_hold_us, config.pulse_low_us)
                + config.pulse_high_us
            )
            / 1e6
        )
        # Independent integral: ramp 0->2deg/s over this finite interval.
        displacement_deg = 0.5 * 2 * motion_s
        assert edge_deg / 2 < displacement_deg < 1.5 * edge_deg
        assert direction * (reference_deg + direction * edge_deg) > 0.1
        before = rig.runtime.plant.snapshot().mechanical_angle_deg
        with pytest.raises(ValueError, match="soft limit"):
            r._plan_jog_pulse(rig.clock(), command.context.command_id)
        assert r._pending_segment is None and r._pulse_demand.get() is None
        assert r._confirmed_pulses == r._reserved_pulses == 0
        assert rig.runtime.plant.snapshot().mechanical_angle_deg == before == 0
    finally:
        rig.close()


@pytest.mark.parametrize("direction", [-1, 1])
def test_jog_rotation_unsettled_reservation_blocks_new_plan_even_when_engine_idle(direction):
    """Finished physical wave != a settled ledger: keep one reservation until ACK."""
    config = RuntimeConfig(rotation_min_deg=-0.1, rotation_max_deg=0.1)
    rig = RuntimeModelRig(config)
    try:
        r = rig.runtime
        assert rig.jog(0, direction * 2).accepted
        rig.advance(0.02)
        pending = r._pending_segment
        assert pending is not None and r._reserved_pulses * direction > 0
        reserved = r._reserved_pulses
        assert r._confirmed_pulses == 0
        # Controlled acknowledgement barrier: I/O completes the real simulated
        # waveform, but the single control writer has not consumed its receipt.
        rig.control_paused = True
        rig.advance(0.03)
        assert r._pulse_idle and r.pulse.is_idle()
        assert r._pending_segment == pending and r._reserved_pulses == reserved
        edge_deg = 360 * config.catheter_per_motor_ratio / config.pulses_per_motor_rev
        truth = rig.runtime.plant.snapshot().mechanical_angle_deg
        assert truth == pytest.approx(reserved * edge_deg)
        assert config.rotation_min_deg <= truth <= config.rotation_max_deg
        mailbox = r._pulse_demand.get()
        fraction = r._jog_rotation.fraction
        assert rig.clock() >= r._jog_rotation.cursor_ns
        r._plan_jog_pulse(rig.clock(), "must-not-reserve-again")
        assert r._pending_segment == pending and r._pulse_demand.get() == mailbox
        assert r._reserved_pulses == reserved and r._confirmed_pulses == 0
        assert r._jog_rotation.fraction == fraction
        r._consume_io(rig.clock())
        assert r._pending_segment is None and r._reserved_pulses == 0
        assert r._confirmed_pulses == reserved
        # Re-reading no completion cannot count the same finite segment again.
        r._consume_io(rig.clock())
        assert r._confirmed_pulses == reserved
    finally:
        rig.close()
