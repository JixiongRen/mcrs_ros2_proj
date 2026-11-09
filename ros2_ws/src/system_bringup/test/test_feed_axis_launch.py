"""Mode selection is checked before any Node action can connect to a device."""

import json
from pathlib import Path
from runpy import run_path
from types import SimpleNamespace

import pytest
from gca.servo_config import make_position_settings, servo_settings_document
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from system_bringup import servo_launch


def context(tmp_path, **overrides):
    root = Path(__file__).resolve().parents[4]
    servo = run_path(str(root / "config/servo_settings.py"))
    encoder = run_path(str(root / "config/encoder_settings.py"))
    settings = make_position_settings(
        servo["settings_payload"]("simulation"),
        servo["position_settings_payload"](),
        encoder["settings_payload"](),
    )
    snapshot = tmp_path / "position.json"
    snapshot.write_text(json.dumps(servo_settings_document(settings)))
    result = LaunchContext()
    result.launch_configurations.update(
        {
            "servo_config_file": str(snapshot),
            "control_mode": "position",
            "backend": "simulation",
            "discovery_range": "LOCALHOST",
            "simulation_slip_ratio": "0.85",
            "trusted_local_control": "false",
            "record_directory": "",
            **overrides,
        }
    )
    return result


def assembly(description):
    return next(entity for entity in description.entities if isinstance(entity, OpaqueFunction))


def test_unified_launch_requires_explicit_selectors_and_only_starts_two_position_nodes(
    tmp_path, monkeypatch
):
    constructed = []
    monkeypatch.setattr(
        servo_launch, "Node", lambda **kwargs: constructed.append(kwargs) or SimpleNamespace()
    )
    description = servo_launch.description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments["control_mode"].default_value is None
    assert arguments["backend"].default_value is None
    assembly(description).execute(context(tmp_path))
    assert [entry["package"] for entry in constructed] == ["gca_ros", "system_coordinator"]
    assert all(entry["parameters"][0]["control_mode"] == "position" for entry in constructed)


@pytest.mark.parametrize(
    "overrides",
    [
        {"control_mode": "manual"},
        {"control_mode": "automatic"},
        {"backend": "hardware"},
        {"discovery_range": "invalid"},
        {"simulation_slip_ratio": "nan"},
    ],
)
def test_bad_selection_never_constructs_device_nodes(tmp_path, monkeypatch, overrides):
    monkeypatch.setattr(
        servo_launch, "Node", lambda **kwargs: pytest.fail("invalid launch reached Node creation")
    )
    with pytest.raises(ValueError):
        assembly(servo_launch.description()).execute(context(tmp_path, **overrides))


def test_old_manual_wrapper_cannot_accept_position_snapshot(tmp_path, monkeypatch):
    monkeypatch.setattr(
        servo_launch, "Node", lambda **kwargs: pytest.fail("legacy wrapper accepted position mode")
    )
    with pytest.raises(ValueError, match="match"):
        assembly(servo_launch.description("simulation")).execute(context(tmp_path))
