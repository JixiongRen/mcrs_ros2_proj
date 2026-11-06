"""Test the acceptance analyser's failure sensitivity; DDS runs remain real."""

import importlib
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
suite = importlib.import_module("teleop_suite")
ROOT = Path(__file__).resolve().parents[1]
CRITERIA = yaml.safe_load(
    (ROOT / "ros2_ws/src/system_bringup/config/teleop_acceptance.yaml").read_text()
)
RUNTIME = yaml.safe_load((ROOT / "ros2_ws/src/system_bringup/config/simulation.yaml").read_text())[
    "runtime"
]


def io_row(time_ns, *, velocity=0.0):
    return dict(
        available=True,
        evidence_kind="SIMULATED_ACCEPTED_IO_AND_PLANT",
        sampled_ns=time_ns,
        received_monotonic_ns=time_ns,
        plant_updated_ns=time_ns,
        plant_position_mm=0.5,
        plant_velocity_mm_s=velocity,
        plant_angle_deg=0.0,
        accepted_velocity_mm_s=velocity,
        servo_enabled=True,
        pulse_idle=True,
        pulse_active_expires_ns=0,
        demand_expires_ns=100_000_000,
        demand_seq=1,
        input_valid_until_ns=150_000_000,
        jog_valid_until_ns=150_000_000,
        control_state_seq=7,
        control_paused=True,
        stop_serial=3,
    )


@pytest.mark.parametrize("scenario", suite.SCENARIOS)
def test_every_scenario_fails_if_observations_missing(scenario):
    checks = suite.scenario_checks(scenario, [], [], [], [], [], 0, CRITERIA, RUNTIME, {})
    assert checks and not all(check["passed"] for check in checks)
    assert not next(
        check for check in checks if check["name"] == "independent_simulated_io_observed"
    )["passed"]


def test_frozen_cached_control_state_cannot_hide_nonzero_simulated_io_after_deadline():
    rows = [io_row(t, velocity=1.0) for t in (160_000_000, 260_000_000, 360_000_000)]
    checks = suite.scenario_checks(
        "control_freeze",
        [],
        [],
        rows,
        [],
        [],
        0,
        CRITERIA,
        RUNTIME,
        dict(freeze_started_ns=100_000_000, freeze_cleared_ns=400_000_000),
    )
    check = next(
        c for c in checks if c["name"] == "original_input_deadline_not_extended_by_live_source"
    )
    assert not check["passed"]
    assert check["details"]["original_input_deadline_ns"] == 150_000_000


def test_reader_disconnect_checks_actual_io_not_only_confirmed_status():
    rows = [io_row(t, velocity=1.0) for t in (2_500_000_000, 3_500_000_000)]
    checks = suite.scenario_checks("disconnect", [], [], rows, [], [], 0, CRITERIA, RUNTIME, {})
    assert not next(c for c in checks if c["name"] == "input_failure_stops_without_cached_resume")[
        "passed"
    ]


def test_stop_timeline_binds_sequence_boot_and_serial_and_preserves_distinct_clocks():
    records = [
        dict(
            kind="input",
            monotonic_ns=1_001_000_000,
            frame=dict(raw_event_ns=900_000_000, observed_ns=950_000_000),
            intent=dict(action="STOP", reason="B_STOP"),
        ),
        dict(
            kind="stop_request",
            monotonic_ns=1_000_000_000,
            sequence=8,
            generation=1,
            reason="B_STOP",
        ),
        dict(
            kind="stop_accepted", monotonic_ns=1_100_000_000, sequence=8, stop_serial=3, boot_id="b"
        ),
    ]
    core = [dict(time_ns=1_050_000_000, stop_serial=3, boot_id="b")]
    io = [io_row(1_200_000_000)]
    states = [
        dict(
            received_monotonic_ns=1_300_000_000,
            stop_serial=3,
            boot_id="b",
            stop=dict(progress="CONFIRMED"),
        )
    ]
    timeline = suite.stop_timelines(records, core, io, states, 0)
    assert timeline["samples"][0]["source_event_ns"] == 900_000_000
    assert timeline["samples"][0]["reader_observed_ns"] == 950_000_000
    assert timeline["summaries"]["owner_accepted"]["p50_ms"] == 100.0
    assert timeline["summaries"]["control_first_record"]["p95_ms"] == 50.0
    assert timeline["summaries"]["independent_io_first_observed"]["max_ms"] == 200.0
    assert timeline["summaries"]["confirmation_first_dds_observed"]["max_ms"] == 300.0


def test_p95_is_interpolated_and_empty_sample_never_claims_zero_latency():
    assert suite.percentile([0.0, 10.0], 95) == 9.5
    assert suite.latency_stats([]) == dict(count=0, p50_ms=None, p95_ms=None, max_ms=None)


def test_recovery_script_releases_lt_without_centering_or_auto_repressing():
    script = suite.script_for("lt_release_restore")
    release = next(e for e in script["events"] if e["at_s"] == 1.5)
    assert release["axes"] == {"2": -32768}
    assert not any(
        e["at_s"] > 1.5 and e.get("axes", {}).get("2") == 32767 for e in script["events"]
    )
