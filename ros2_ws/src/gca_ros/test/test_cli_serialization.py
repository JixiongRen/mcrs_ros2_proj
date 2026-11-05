"""Exercise the client boundary through real generated C serialization support."""

import json
import stat
import threading

import pytest
from gca_ros.cli import Credentials, GcaClient, finite_float, save_credentials
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import CommandContext, GcaState
from gca_system_interfaces.srv import ControlSession
from rclpy.serialization import deserialize_message, serialize_message


def test_integer_user_inputs_are_normalized_before_ros_wire_serialization():
    # Isolate service transport, while preserving the actual client methods and
    # the generated ROS messages/C serialization that previously corrupted int.
    client = object.__new__(GcaClient)
    client._lock = threading.RLock()
    client._latest = GcaState(reference_id="reference")
    captured = {}

    def transport(service_type, name, request):
        wire = deserialize_message(serialize_message(request), ControlSession.Request)
        captured[name] = wire
        return ControlSession.Response(
            accepted=True,
            context=CommandContext(session_id="session", generation=1, sequence=1),
            token="source-cap" if name.endswith("claim_control") else "source-cap.nonce",
            granted_lease_s=4.0,
        )

    client.call = transport
    client.claim(lease_s=4, auto_renew=False)
    assert captured["/system/claim_control"].lease_s == 4.0
    goal = client.make_goal(
        linear_mm=5, rotation_deg=20, timeout_s=30, max_linear_mm_s=8, max_rotation_deg_s=60
    )
    roundtrip = deserialize_message(serialize_message(goal), ExecuteMotion.Goal)
    assert roundtrip.execution_timeout_s == 30.0
    assert roundtrip.target.linear_m == 0.005
    assert roundtrip.limits.max_linear_m_s == 0.008
    assert roundtrip.token == "source-cap.nonce"
    assert captured["/system/get_motion_permit"].lease_s == 0.5


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True])
def test_cli_rejects_nonfinite_or_boolean_motion_numbers(value):
    with pytest.raises(ValueError):
        finite_float(value, "motion_value")


def test_token_file_is_private_on_creation_and_atomic_replacement(tmp_path):
    path = tmp_path / "session.json"
    credentials = Credentials("session", 1, "private-token")
    save_credentials(path, credentials)
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert json.loads(path.read_text())["token"] == "private-token"
    path.chmod(0o644)
    save_credentials(path, Credentials("new-session", 2, "replacement-token"))
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert json.loads(path.read_text())["token"] == "replacement-token"
    assert list(tmp_path.glob(".gca-session-*")) == []
