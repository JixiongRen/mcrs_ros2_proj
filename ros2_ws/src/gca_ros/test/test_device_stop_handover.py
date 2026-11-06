"""Real simulated runtime plus explicitly ordered concurrent ROS ingress calls."""

import threading

import pytest
import rclpy
from gca_ros.node import GcaControlNode
from gca_system_interfaces.msg import CommandContext
from gca_system_interfaces.srv import ManageDevice
from rclpy.lifecycle import TransitionCallbackReturn


@pytest.fixture
def device():
    rclpy.init(args=["--ros-args", "-p", "coordinator_key:=device-stop-test-key"])
    node = GcaControlNode()
    assert node.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert node.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert node.runtime.set_reference().accepted
    grant = node._authority.issue("device-stop-test-key", "source", 1, 1, 4.0)
    yield node, grant
    node._shutdown_runtime()
    node.destroy_node()
    rclpy.shutdown()


def ctx(seq):
    return CommandContext(
        session_id="source",
        generation=1,
        sequence=seq,
        task_id="manual",
        command_id="command" + str(seq),
    )


def test_stop_latches_before_releasing_slow_arm_and_prevents_enable(device):
    node, grant = device
    entered = threading.Event()
    phases, result = [], []

    def phase(name, context):
        phases.append(name)
        if name == "WAITING":
            entered.set()

    node._publish_arm_phase = phase
    node._arm_release.clear()
    thread = threading.Thread(
        target=lambda: result.append(
            node._manage(
                "arm",
                ManageDevice.Request(context=ctx(2), token=grant.token),
                ManageDevice.Response(),
            )
        )
    )
    thread.start()
    try:
        assert entered.wait(1.0)
        before = node.runtime.stop_serial
        stopped = node._coordinator_stop(
            ManageDevice.Request(context=ctx(3), token="device-stop-test-key"),
            ManageDevice.Response(),
            disarm=True,
        )
        assert stopped.accepted
        assert node.runtime.stop_serial > before
        thread.join(1.0)
        assert not thread.is_alive()
        assert len(result) == 1 and not result[0].accepted
        assert result[0].error_code == "STOP_FENCED"
        assert not node.runtime.snapshot().armed
        assert phases == ["WAITING", "RELEASED"]
        assert node.stop_group is not node.management_group
        assert node.fault_group is not node.management_group
    finally:
        node._arm_release.set()
        thread.join(2.0)


def test_arm_pause_timeout_rejects_without_enabling_and_fences_old_rpc(device):
    node, grant = device
    phases = []
    node._publish_arm_phase = lambda phase, context: phases.append(phase)
    node._arm_release.clear()
    request = ManageDevice.Request(context=ctx(2), token=grant.token)
    response = node._manage("arm", request, ManageDevice.Response())
    assert not response.accepted and response.error_code == "ARM_PAUSE_TIMEOUT"
    assert not node.runtime.snapshot().armed
    assert phases == ["WAITING", "TIMEOUT"]
    node._arm_release.set()
    repeated = node._manage("arm", request, ManageDevice.Response())
    assert not repeated.accepted and repeated.error_code == "STOP_FENCED"


def test_new_device_capability_stop_latches_when_core_still_has_old_lease(device):
    from gca.domain.models import Lease
    from gca_ros.conversion import context_from_ros

    node, old = device
    assert node.runtime.arm(Lease("source", 1, 1, 4.0, old.token)).accepted
    node.runtime.stop("EXPLICIT_DISARM", disarm=True)
    node._revoke_authority()
    new = node._authority.issue("device-stop-test-key", "source", 2, 2, 4.0)
    incoming = ctx(4)
    incoming.generation = 2
    # Core permission matching is intentionally strict. The trusted adapter
    # must first latch an unconditional stop during this handover window.
    assert not node.runtime.stop(context=context_from_ros(incoming)).accepted
    before = node.runtime.stop_serial
    response = node._coordinator_stop(
        ManageDevice.Request(context=incoming, token="device-stop-test-key"),
        ManageDevice.Response(),
        disarm=True,
    )
    assert response.accepted
    assert node.runtime.stop_serial > before
    denied = node.runtime.arm(Lease("source", 2, 2, 4.0, new.token), expected_stop_serial=before)
    assert not denied.accepted


def test_shutdown_latches_before_releasing_arm_pause(device):
    node, grant = device
    entered = threading.Event()
    responses, errors = [], []
    original_phase = node._publish_arm_phase

    def phase(name, context):
        original_phase(name, context)
        if name == "WAITING":
            entered.set()

    def arm():
        try:
            responses.append(
                node._manage(
                    "arm",
                    ManageDevice.Request(context=ctx(2), token=grant.token),
                    ManageDevice.Response(),
                )
            )
        except BaseException as exc:
            errors.append(exc)

    node._publish_arm_phase = phase
    node._arm_release.clear()
    thread = threading.Thread(target=arm)
    thread.start()
    try:
        assert entered.wait(1.0)
        assert node._shutdown_runtime() == TransitionCallbackReturn.SUCCESS
        thread.join(1.0)
        assert not thread.is_alive()
        assert not errors and len(responses) == 1
        assert not responses[0].accepted
        assert node.runtime is None
    finally:
        node._arm_release.set()
        thread.join(2.0)
