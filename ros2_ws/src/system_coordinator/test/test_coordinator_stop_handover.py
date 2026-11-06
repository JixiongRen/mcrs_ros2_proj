"""Controlled coroutine interleavings using real ROS service message types."""

import asyncio
import copy
import time

import pytest
import rclpy
from gca.domain.manual import ManualInputQuality, host_clock_id
from gca_ros.conversion import quality_to_ros
from gca_system_interfaces.msg import CommandContext, GcaState, JogCommand, StopStatus
from gca_system_interfaces.srv import ControlSession, ExecutionLease, ManageDevice
from rclpy.executors import SingleThreadedExecutor
from system_coordinator.node import SystemCoordinator


@pytest.fixture
def coordinator():
    rclpy.init(args=["--ros-args", "-p", "coordinator_key:=stop-handover-test-key"])
    node = SystemCoordinator()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def context(source, sequence):
    return CommandContext(
        session_id=source.session_id,
        generation=source.generation,
        sequence=sequence,
        task_id="manual",
        command_id="command" + str(sequence),
    )


@pytest.mark.parametrize("phase", ["before_rotate", "after_rotate", "arm_response"])
def test_stop_during_arm_cannot_be_lost_at_generation_handover(coordinator, phase):
    node = coordinator
    source = node._authority.claim("operator", "MANUAL", 4.0)
    source.needs_reauthorization = True
    source.executor_token = "old-device-token"
    old_context = context(source, 2)
    stop_context = context(source, 3)
    node._state = GcaState(state="DISARMED", stop=StopStatus(progress="CONFIRMED"))
    calls = []

    def stop_now():
        verified = node._stop_verify(stop_context, source.token)
        assert verified is source
        node._fence_stop(stop_context, source)

    async def transport(client, request):
        calls.append((client, copy.deepcopy(request)))
        if client is node._lease_client:
            if request.release:
                if phase == "before_rotate":
                    stop_now()
                return ExecutionLease.Response(accepted=True, error_code="OK")
            if phase == "after_rotate":
                assert source.generation != old_context.generation
                stop_now()
            return ExecutionLease.Response(accepted=True, token="new-device-token")
        if client is node._manage_clients["arm"]:
            assert phase == "arm_response"
            stop_now()
            return ManageDevice.Response(accepted=True, state=GcaState(armed=True))
        raise AssertionError("unexpected call")

    node._call = transport
    response = asyncio.run(
        node._manage(
            "arm",
            ManageDevice.Request(context=old_context, token=source.token),
            ManageDevice.Response(),
        )
    )
    assert not response.accepted
    assert response.error_code == "ARM_INTERRUPTED_BY_STOP"
    assert response.state.generation == source.generation
    assert source.needs_reauthorization
    assert sum(client is node._manage_clients["arm"] for client, _ in calls) == (
        phase == "arm_response"
    )


def test_stop_fences_source_and_transition_generation_and_invalidates_pending(coordinator):
    node = coordinator
    source = node._authority.claim("operator", "MANUAL", 4.0)
    old = context(source, 8)
    node._authority.rotate_generation(source)
    node._pending_jog = object()
    with pytest.raises(ValueError):
        node._restricted_verify(old, source.token)  # No alias for release/fault clear.
    assert node._stop_verify(old, source.token) is source
    node._fence_stop(old, source)
    assert node._pending_jog is None
    for generation in (old.generation, source.generation):
        stale = copy.deepcopy(old)
        stale.generation = generation
        with pytest.raises(ValueError, match="STOP_FENCED"):
            node._motion_permits.issue(source.token, stale, 0.1, source.deadline)


def test_handover_ack_and_timeout_are_explicit(coordinator):
    node = coordinator
    source = node._authority.claim("operator", "MANUAL", 4.0)
    node._authority.rotate_generation(source)
    assert source.stop_alias_generation
    response = node._acknowledge(
        ControlSession.Request(context=context(source, 3), token=source.token),
        ControlSession.Response(),
    )
    assert response.accepted and source.stop_alias_generation == 0
    node._authority.rotate_generation(source)
    source.handover_deadline = time.monotonic() - 0.01
    captured = []
    node._expire_source = lambda reason: captured.append(reason)
    node._watchdog()
    assert captured == ["SOURCE_EXPIRED"]


def test_stop_service_groups_are_independent_from_management(coordinator):
    assert coordinator.stop_group is not coordinator.management_group


@pytest.mark.parametrize("operation", ["request_stop", "disarm", "deadman_false"])
@pytest.mark.parametrize("authenticated_state", ["current", "retired"])
def test_authenticated_old_stop_cannot_mutate_replacement_owner(
    coordinator, monkeypatch, operation, authenticated_state
):
    """Select preemption after STOP authentication, before its second lock."""
    node = coordinator
    old = node._authority.claim("old-owner", "MANUAL", 4.0)
    request_context = context(old, 3)
    if authenticated_state == "retired":
        node._authority.release()
    verify = node._stop_verify
    replacement = {}
    calls = []
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    monkeypatch.setattr(executor, "create_task", lambda *args: calls.append(args))

    async def transport(client, request):
        calls.append((client, request))
        return ManageDevice.Response(accepted=True, state=GcaState())

    monkeypatch.setattr(node, "_call", transport)

    def authenticate_then_replace(ctx, token):
        verified = verify(ctx, token)
        # The STOP callback is in stop_group. While it is preempted here,
        # management_group can finish releasing the old source and claiming a
        # replacement. No await inside release_control is being bypassed.
        with node._lock:
            if node._authority.current is not None:
                node._authority.release()
            node._motion_permits.retire_session(old.session_id)
            new = node._authority.claim("replacement-owner", "MANUAL", 4.0)
            pending = object()
            node._pending_jog = pending
            permit_context = context(new, 2)
            permit, deadline = node._motion_permits.issue(
                new.token, permit_context, 0.5, new.deadline
            )
            replacement.update(
                source=new,
                pending=pending,
                stop_epoch=node._stop_epoch,
                jog_epoch=node._jog_epoch,
                permit=permit,
                permit_context=permit_context,
                deadline=deadline,
            )
        return verified

    monkeypatch.setattr(node, "_stop_verify", authenticate_then_replace)
    response = None
    try:
        if operation == "deadman_false":
            node._jog(JogCommand(context=request_context, token=old.token, deadman=False))
        else:
            response = asyncio.run(
                node._manage(
                    operation,
                    ManageDevice.Request(context=request_context, token=old.token),
                    ManageDevice.Response(),
                )
            )
        # Inspect all effects before asserting, including the replacement's
        # already issued one-use permit, which must remain usable.
        permit_deadline = node._motion_permits.consume(
            replacement["permit"], replacement["permit_context"]
        )
        observed = dict(
            current_unchanged=node._authority.current is replacement["source"],
            pending_unchanged=node._pending_jog is replacement["pending"],
            stop_epoch_unchanged=node._stop_epoch == replacement["stop_epoch"],
            jog_epoch_unchanged=node._jog_epoch == replacement["jog_epoch"],
            permit_unchanged=permit_deadline == replacement["deadline"],
            no_stop_worker=not node._jog_stopping,
            no_forwarded_stop=not calls,
        )
        assert all(observed.values()), observed
        if response is not None:
            assert not response.accepted
            assert response.error_code == "INVALID_RETIRED_CAPABILITY"
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)


@pytest.mark.parametrize("authority_state", ["same_owner_alias", "retired"])
def test_stop_fence_revalidation_preserves_bounded_existing_stop_authority(
    coordinator, authority_state
):
    node = coordinator
    source = node._authority.claim("operator", "MANUAL", 4.0)
    stop = context(source, 8)
    if authority_state == "same_owner_alias":
        node._authority.rotate_generation(source)
    else:
        node._authority.release()
    verified = node._stop_verify(stop, source.token)
    node._pending_jog = object()
    node._fence_stop(stop, verified)
    assert node._pending_jog is None
    assert node._stop_epoch == 1 and node._jog_epoch == 1
    for generation in {stop.generation, verified.generation}:
        bounded = copy.deepcopy(stop)
        bounded.generation = generation
        with pytest.raises(ValueError, match="STOP_FENCED"):
            node._motion_permits.check_fence(bounded)


@pytest.mark.parametrize("transition", ["new_owner", "same_owner_new_generation"])
def test_previous_session_generation_stop_cannot_block_or_clear_current_stop(
    coordinator, monkeypatch, transition
):
    node = coordinator
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    created = []

    async def interleaving():
        old = node._authority.claim("old-owner", "MANUAL", 4.0)
        old_identity = (old.session_id, old.generation)
        old_entered, old_release, new_entered, new_release = (asyncio.Event() for _ in range(4))

        async def transport(client, request):
            if (request.context.session_id, request.context.generation) == old_identity:
                old_entered.set()
                await old_release.wait()
            else:
                new_entered.set()
                await new_release.wait()
            return ManageDevice.Response(accepted=True, state=GcaState())

        def create(function, *args):
            task = asyncio.create_task(function(*args))
            created.append(task)
            return task

        monkeypatch.setattr(node, "_call", transport)
        monkeypatch.setattr(executor, "create_task", create)
        try:
            node._jog(JogCommand(context=context(old, 3), token=old.token, deadman=False))
            await asyncio.wait_for(old_entered.wait(), timeout=1.0)
            with node._lock:
                if transition == "new_owner":
                    node._authority.release()
                    new = node._authority.claim("new-owner", "MANUAL", 4.0)
                else:
                    node._authority.rotate_generation(old)
                    new = old
            node._jog(JogCommand(context=context(new, 3), token=new.token, deadman=False))
            assert len(created) == 2, "old session/generation RPC must not suppress current STOP"
            await asyncio.wait_for(new_entered.wait(), timeout=1.0)
            old_release.set()
            await asyncio.wait_for(created[0], timeout=1.0)
            assert node._jog_stopping, "old finally must not clear the new owner's stop work"
            new_release.set()
            await asyncio.wait_for(created[1], timeout=1.0)
            assert not node._jog_stopping
        finally:
            old_release.set()
            new_release.set()
            if created:
                await asyncio.wait_for(asyncio.gather(*created), timeout=1.0)

    try:
        asyncio.run(interleaving())
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)


@pytest.mark.parametrize("interruption", ["replacement_owner", "same_owner_stop"])
def test_jog_final_mailbox_commit_rechecks_owner_and_stop_barrier(
    coordinator, monkeypatch, interruption
):
    node = coordinator
    source = node._authority.claim("operator", "MANUAL", 4.0)
    incoming = context(source, 3)
    token, _ = node._motion_permits.issue(source.token, incoming, 0.5, source.deadline)
    now = time.monotonic_ns()
    quality = ManualInputQuality(
        "SCRIPT_SAMPLE",
        "fixture",
        "epoch",
        1,
        now,
        "MONOTONIC",
        1,
        now,
        now + 150_000_000,
        host_clock_id(),
        "SYNTHETIC",
    )
    validate = node._quality.validate
    protected = object()
    created = []
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    monkeypatch.setattr(executor, "create_task", lambda *args: created.append(args))

    def validate_then_interrupt(*args, **kwargs):
        deadline = validate(*args, **kwargs)
        with node._lock:
            if interruption == "replacement_owner":
                node._authority.release()
                node._motion_permits.retire_session(source.session_id)
                node._authority.claim("replacement-owner", "MANUAL", 4.0)
            else:
                node._fence_stop(context(source, 4), source)
            # A newer source/sequence owns the pending slot by this point.
            node._pending_jog = protected
        return deadline

    monkeypatch.setattr(node._quality, "validate", validate_then_interrupt)
    try:
        node._jog(
            JogCommand(
                context=incoming,
                token=token,
                deadman=True,
                axis_mask=3,
                ttl_s=0.15,
                quality=quality_to_ros(quality),
            )
        )
        assert node._pending_jog is protected
        assert not created and not node._jog_forwarding
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
