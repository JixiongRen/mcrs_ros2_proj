"""Braking default selection and boundary use without ROS or hardware."""

import json
import subprocess
import sys
from copy import deepcopy
from pathlib import Path

import pytest
from gca.domain.position import resolve_braking_parameters


def test_unknown_values_use_explicit_defaults_without_mutating_the_source():
    motion = {"braking_deceleration_mm_s2": None, "braking_delay_s": None}
    defaults = {"braking_deceleration_mm_s2": 5.0, "braking_delay_s": 0.5}
    before = deepcopy((motion, defaults))
    result = resolve_braking_parameters(motion, defaults)
    assert (result.deceleration_mm_s2, result.delay_s) == (5.0, 0.5)
    assert result.deceleration_source == result.delay_source == "default"
    assert (motion, defaults) == before


def test_each_supplied_value_overrides_only_its_own_default_including_zero_delay():
    defaults = {"braking_deceleration_mm_s2": 5.0, "braking_delay_s": 0.5}
    result = resolve_braking_parameters(
        {"braking_deceleration_mm_s2": 3.0, "braking_delay_s": None}, defaults
    )
    assert (result.deceleration_mm_s2, result.delay_s) == (3.0, 0.5)
    assert (result.deceleration_source, result.delay_source) == ("configured", "default")
    result = resolve_braking_parameters(
        {"braking_deceleration_mm_s2": None, "braking_delay_s": 0.0}, defaults
    )
    assert (result.deceleration_mm_s2, result.delay_s) == (5.0, 0.0)
    assert (result.deceleration_source, result.delay_source) == ("default", "configured")


def test_invalid_values_are_rejected_instead_of_disguised_as_unknown():
    defaults = {"braking_deceleration_mm_s2": 5.0, "braking_delay_s": 0.5}
    for name, value in (
        ("braking_deceleration_mm_s2", 0.0),
        ("braking_delay_s", -0.1),
        ("braking_delay_s", False),
        ("braking_deceleration_mm_s2", float("nan")),
        ("braking_delay_s", float("inf")),
    ):
        motion = {"braking_deceleration_mm_s2": None, "braking_delay_s": None}
        motion[name] = value
        with pytest.raises(ValueError):
            resolve_braking_parameters(motion, defaults)
    with pytest.raises(ValueError):
        resolve_braking_parameters(
            {"braking_deceleration_mm_s2": None, "braking_delay_s": None},
            {"braking_deceleration_mm_s2": None, "braking_delay_s": 0.5},
        )


def test_resolved_values_drive_symmetric_boundary_braking():
    result = resolve_braking_parameters(
        {"braking_deceleration_mm_s2": None, "braking_delay_s": None},
        {"braking_deceleration_mm_s2": 5.0, "braking_delay_s": 0.5},
    )
    # With 5 mm remaining before the reserved edge, 5 mm/s consumes
    # 2.5 mm in delay and 2.5 mm while decelerating at 5 mm/s².
    assert result.limit_velocity(10.0, 985.0, -50.0, 1000.0, 10.0) == pytest.approx(5.0)
    assert result.limit_velocity(-10.0, -35.0, -50.0, 1000.0, 10.0) == pytest.approx(-5.0)
    assert result.limit_velocity(10.0, 990.0, -50.0, 1000.0, 10.0) == 0.0
    assert result.limit_velocity(-10.0, -40.0, -50.0, 1000.0, 10.0) == 0.0


def test_position_export_keeps_unknown_measurements_and_resolved_sources(tmp_path):
    root = Path(__file__).resolve().parents[4]
    script = root / "config/servo_settings.py"
    target = tmp_path / "position.json"
    subprocess.run(
        [
            sys.executable,
            "-I",
            "-S",
            "-B",
            str(script),
            "export",
            str(target),
            "--section",
            "position",
        ],
        cwd=tmp_path,
        check=True,
        capture_output=True,
        text=True,
    )
    document = json.loads(target.read_text())
    assert document["schema_version"] == 2
    assert document["motion"]["braking_deceleration_mm_s2"] is None
    assert document["motion"]["braking_delay_s"] is None
    expected = resolve_braking_parameters(document["motion"], document["braking_defaults"])
    assert document["effective_braking"] == {
        "deceleration_mm_s2": expected.deceleration_mm_s2,
        "delay_s": expected.delay_s,
        "deceleration_source": "default",
        "delay_source": "default",
    }
