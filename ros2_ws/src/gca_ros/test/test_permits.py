from types import SimpleNamespace

import pytest
from gca_ros.permits import MotionPermits, PermitError, capability_part


def context(command="command", sequence=1):
    return SimpleNamespace(
        session_id="session", generation=2, command_id=command, sequence=sequence, task_id="task"
    )


def test_lease_renewal_cannot_extend_already_issued_nonce():
    clock = [10.0]
    permits = MotionPermits(clock=lambda: clock[0])
    envelope, deadline = permits.issue("renewable-capability", context(), 0.2, 11.0)
    assert deadline == 10.2
    clock[0] = 10.3
    # Even if the same capability has since been renewed to a later lease,
    # consumption checks the receiver's original nonce deadline.
    with pytest.raises(PermitError, match="EXPIRED"):
        permits.consume(envelope, context())


def test_single_use_and_exact_context_binding():
    permits = MotionPermits()
    envelope, _ = permits.issue("capability", context(), 0.5, permits.clock() + 1)
    assert capability_part(envelope) == "capability"
    with pytest.raises(PermitError, match="CONTEXT_MISMATCH"):
        permits.consume(envelope, context("different"))
    with pytest.raises(PermitError, match="USED_PERMIT"):
        permits.consume(envelope, context())


def test_system_and_device_nonce_domains_do_not_accept_each_other():
    source = MotionPermits()
    device = MotionPermits()
    upstream, _ = source.issue("source-cap", context(), 0.5, source.clock() + 1)
    downstream, _ = device.issue("executor-cap", context(), 0.5, device.clock() + 1)
    with pytest.raises(PermitError, match="USED_PERMIT"):
        device.consume(upstream, context())
    assert source.consume(upstream, context()) > source.clock()
    assert device.consume(downstream, context()) > device.clock()


def test_capacity_is_bounded_and_expired_entries_are_reclaimed():
    clock = [0.0]
    permits = MotionPermits(capacity=1, clock=lambda: clock[0])
    permits.issue("cap", context(), 0.2, 2.0)
    with pytest.raises(PermitError, match="CAPACITY"):
        permits.issue("cap", context("second"), 0.2, 2.0)
    clock[0] = 0.3
    permits.issue("cap", context("second"), 0.2, 2.0)
    assert len(permits._pending) == 1


def test_logical_goal_identity_ignores_authentication_and_admission_budget():
    import copy

    from gca_ros.permits import same_motion_payload
    from gca_system_interfaces.action import ExecuteMotion

    goal = ExecuteMotion.Goal(
        token="source.old-nonce", admission_timeout_s=0.5, execution_timeout_s=20.0, end_policy=1
    )
    goal.target.linear_m = 0.005
    fresh = copy.deepcopy(goal)
    fresh.token = "source.new-nonce"
    fresh.admission_timeout_s = 0.1
    assert same_motion_payload(goal, fresh)
    fresh.target.linear_m = 0.006
    assert not same_motion_payload(goal, fresh)
    fresh.target.linear_m = goal.target.linear_m
    fresh.execution_timeout_s = 21.0
    assert not same_motion_payload(goal, fresh)


def test_stop_fence_revokes_unused_and_already_issued_nonces():
    from types import SimpleNamespace

    import pytest
    from gca_ros.permits import MotionPermits, PermitError

    p = MotionPermits(clock=lambda: 1.0)
    context = SimpleNamespace(session_id="s", generation=1, sequence=4, command_id="c", task_id="t")
    nonce, _ = p.issue("cap", context, 0.1, 2.0)
    stop = SimpleNamespace(session_id="s", generation=1, sequence=5)
    p.fence(stop)
    with pytest.raises(PermitError, match="STOP_FENCED"):
        p.consume(nonce, context)
    with pytest.raises(PermitError, match="STOP_FENCED"):
        p.issue("cap", context, 0.1, 2.0)
    context.sequence = 6
    assert p.issue("cap", context, 0.1, 2.0)[0]
