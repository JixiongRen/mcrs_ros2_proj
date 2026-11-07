"""A waiting management callback must not serialize manual motion permits."""

import asyncio
from threading import Event, Thread

import pytest
import rclpy
from gca_ros.node import GcaControlNode
from gca_system_interfaces.msg import CommandContext
from gca_system_interfaces.srv import ControlSession, ExecutionLease, ManageDevice
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from system_coordinator.node import SystemCoordinator


@pytest.mark.parametrize("host", ["device", "coordinator"])
def test_motion_permit_completes_while_management_waits(host):
    key = "test-permit-scheduling"
    rclpy.init(args=["--ros-args", "-p", f"coordinator_key:={key}"])
    node = GcaControlNode() if host == "device" else SystemCoordinator()
    client_node = Node("permit_scheduling_client")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    entered, release, replied = Event(), Event(), Event()

    def waiting_management(request, response):
        entered.set()
        release.wait(2.0)
        response.accepted = True
        return response

    node.create_service(
        ManageDevice,
        "/test/waiting_management",
        waiting_management,
        callback_group=node.management_group,
    )
    manage = client_node.create_client(ManageDevice, "/test/waiting_management")
    topic = "/gca/get_motion_permit" if host == "device" else "/system/get_motion_permit"
    permit = client_node.create_client(ControlSession, topic)
    worker = Thread(target=executor.spin)
    worker.start()
    try:
        assert manage.wait_for_service(timeout_sec=2.0)
        assert permit.wait_for_service(timeout_sec=2.0)
        if host == "device":
            grant = node._authority.issue(key, "unit-owner", 1, 1, 3.0)
        else:
            grant = node._authority.claim("unit-owner", "MANUAL", 3.0)
        context = CommandContext(
            session_id=grant.session_id,
            generation=grant.generation,
            sequence=2,
            task_id="manual",
            command_id="fresh-jog",
        )
        management = manage.call_async(ManageDevice.Request())
        assert entered.wait(1.0)
        future = permit.call_async(
            ControlSession.Request(context=context, token=grant.token, lease_s=0.15)
        )
        future.add_done_callback(lambda _: replied.set())
        # This bound is only a test rendezvous: no production deadline changes.
        assert replied.wait(0.4), "motion permit queued behind waiting management"
        assert not management.done() and not release.is_set()
        response = future.result()
        assert response.accepted and 0 < response.granted_lease_s <= 0.15
        deadline = node._motion_permits.consume(response.token, context)
        assert deadline <= grant.deadline
        with pytest.raises(ValueError, match="INVALID_OR_USED_PERMIT"):
            node._motion_permits.consume(response.token, context)
    finally:
        release.set()
        executor.shutdown(timeout_sec=2.0)
        worker.join(timeout=2.0)
        client_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("host", ["device", "coordinator"])
def test_revocation_cannot_leave_permit_registered_after_retirement(host, monkeypatch):
    key = "test-permit-retirement"
    rclpy.init(args=["--ros-args", "-p", f"coordinator_key:={key}"])
    node = GcaControlNode() if host == "device" else SystemCoordinator()
    entered, release, revoked = Event(), Event(), Event()
    workers, errors, responses = [], [], []
    original_issue = node._motion_permits.issue

    def paused_issue(*args):
        entered.set()
        assert release.wait(2.0)
        return original_issue(*args)

    def guarded(operation):
        try:
            operation()
        except BaseException as exc:
            errors.append(exc)

    async def accept_release(client, request):
        assert client is node._lease_client and request.release
        return ExecutionLease.Response(accepted=True, error_code="OK")

    try:
        if host == "device":
            grant = node._authority.issue(key, "unit-owner", 1, 1, 3.0)
            authority_lock = node._authority.transaction_lock
        else:
            grant = node._authority.claim("unit-owner", "MANUAL", 3.0)
            authority_lock = node._lock
            monkeypatch.setattr(node, "_call", accept_release)
        context = CommandContext(
            session_id=grant.session_id,
            generation=grant.generation,
            sequence=2,
            task_id="manual",
            command_id="pending-permit",
        )
        stop_context = CommandContext(
            session_id=grant.session_id,
            generation=grant.generation,
            sequence=3,
            task_id="manual",
            command_id="release-owner",
        )
        monkeypatch.setattr(node._motion_permits, "issue", paused_issue)

        def issue():
            responses.append(
                node._motion_permit(
                    ControlSession.Request(context=context, token=grant.token, lease_s=0.15),
                    ControlSession.Response(),
                )
            )

        def revoke():
            if host == "device":
                node._revoke_authority()
            else:
                response = asyncio.run(
                    node._control(
                        "release_control",
                        ControlSession.Request(context=stop_context, token=grant.token),
                        ControlSession.Response(),
                    )
                )
                assert response.accepted
            revoked.set()

        issuer = Thread(target=lambda: guarded(issue))
        workers.append(issuer)
        issuer.start()
        assert entered.wait(1.0)
        # With no other worker yet, this probe deterministically distinguishes
        # an atomic authenticate/issue transaction from the vulnerable gap.
        # If retirement can proceed, complete it before allowing late issue.
        retirement_can_proceed = authority_lock.acquire(blocking=False)
        if retirement_can_proceed:
            authority_lock.release()
        revoker = Thread(target=lambda: guarded(revoke))
        workers.append(revoker)
        revoker.start()
        if retirement_can_proceed:
            assert revoked.wait(1.0), "retirement did not reach its controlled rendezvous"
        release.set()
        for worker in workers:
            worker.join(timeout=2.0)
        assert all(not worker.is_alive() for worker in workers)
        assert not errors, errors
        assert revoked.is_set() and responses[0].accepted
        current = node._authority.grant if host == "device" else node._authority.current
        assert current is None
        binding = (context.session_id, context.generation)
        assert binding not in node._motion_permits._seen
        assert not any(
            permit.context[:2] == binding for permit in node._motion_permits._pending.values()
        )
    finally:
        release.set()
        for worker in workers:
            worker.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
