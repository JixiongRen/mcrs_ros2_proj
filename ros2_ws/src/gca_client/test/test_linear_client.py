"""Verify the actual ROS wire contract without starting an executor or any device."""

import threading
from types import SimpleNamespace

import pytest
from gca_client.client import Credentials, GcaClient
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import GcaState
from gca_system_interfaces.srv import GetCapabilities
from rclpy.serialization import deserialize_message, serialize_message


def client():
    instance = object.__new__(GcaClient)
    instance._lock = threading.RLock()
    instance._latest = GcaState(reference_id="fixed-reference")
    instance.credentials = Credentials("auto", 1, "private")
    caps = GetCapabilities.Response(can_linear_motion=True, can_measure_linear=True)
    caps.limits.max_linear_m_s = 0.01
    caps.limits.max_linear_m_s2 = 0.01
    instance.capabilities = lambda: caps
    instance.motion_permit = lambda context, ttl: SimpleNamespace(token="permit")
    return instance


@pytest.mark.parametrize("relative", [False, True])
def test_explicit_linear_goal_serializes_units_and_reference(relative):
    goal = client().make_linear_goal(
        -5, relative=relative, speed_mm_s=2, acceleration_mm_s2=5, timeout_s=10
    )
    wire = deserialize_message(serialize_message(goal), ExecuteMotion.Goal)
    assert wire.target.axis_mask == 1
    assert wire.target.mode == (2 if relative else 1)
    assert wire.target.linear_m == -0.005
    assert wire.target.rotation_rad == 0.0
    assert wire.target.reference_id == "fixed-reference"
    assert wire.limits.max_linear_m_s == 0.002
    assert wire.limits.max_linear_m_s2 == 0.005
    assert wire.end_policy == 1
    assert wire.execution_timeout_s == 10.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed_mm_s": 10.1},
        {"acceleration_mm_s2": 10.1},
        {"timeout_s": 0},
        {"speed_mm_s": float("nan")},
    ],
)
def test_invalid_request_never_acquires_motion_permit(kwargs):
    instance = client()
    instance.motion_permit = lambda *args: pytest.fail("invalid request acquired a permit")
    with pytest.raises(ValueError):
        instance.make_linear_goal(5, **kwargs)


def test_manual_capability_cannot_enter_position_path():
    instance = client()
    instance.capabilities = GetCapabilities.Response
    with pytest.raises(RuntimeError, match="unavailable"):
        instance.make_linear_goal(5)


def test_legacy_imports_forward_the_same_implementation():
    from gca_ros.cli import GcaClient as LegacyClient

    assert LegacyClient is GcaClient
