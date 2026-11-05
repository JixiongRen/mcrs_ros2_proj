"""Independent model/calibration reports are repeatable and reject bad evidence."""

import importlib.util
import json
from pathlib import Path

import pytest


def load_tool(name):
    path = Path(__file__).resolve().parents[1] / "tools" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_fixed_seed_model_rows_repeat_and_slip_is_not_target_echo():
    model = load_tool("model_report")
    first_rows, first_metrics = model.advance_scenario("slip", slip=0.25)
    second_rows, second_metrics = model.advance_scenario("slip", slip=0.25)
    assert json.dumps(first_rows) == json.dumps(second_rows)
    assert first_metrics == second_metrics
    assert first_metrics["passed"]
    assert first_rows[0]["plant_mm"] == 0
    assert any(0 < row["plant_mm"] < row["target_mm"] for row in first_rows)
    _, fault_metrics = model.advance_scenario("zero_failure", zero_failure=True)
    assert fault_metrics["passed"] and not fault_metrics["stop_confirmed_from_model_velocity"]
    _, rotation = model.rotation_scenario("missed", missed_fraction=0.2)
    assert rotation["passed"]


def test_offline_calibration_uses_independent_signed_displacement(tmp_path):
    tool = load_tool("calibrate_from_measurements")
    source = tmp_path / "measurements.csv"
    source.write_text(
        "count_start,count_end,measured_displacement_mm\n0,100,1\n100,0,-1\n0,200,2\n200,0,-2\n"
    )
    rows, result = tool.calibrate(source)
    assert len(rows) == 4
    assert result["signed_mm_per_count"] == 0.01
    assert result["residual_rmse_mm"] == 0
    assert result["hardware_verified_by_this_tool"] is False


@pytest.mark.parametrize("row", ["", "0,0,1\n", "0,1,nan\n", "0,1,1\n1,0,1\n"])
def test_calibration_rejects_invalid_or_inconsistent_evidence(tmp_path, row):
    tool = load_tool("calibrate_from_measurements")
    source = tmp_path / "bad.csv"
    source.write_text("count_start,count_end,measured_displacement_mm\n" + row)
    with pytest.raises(ValueError):
        tool.calibrate(source)
