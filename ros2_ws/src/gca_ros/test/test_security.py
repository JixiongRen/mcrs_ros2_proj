import pytest
from gca_ros.security import AccessError, LeaseAuthority


def test_expired_renewal_cannot_resurrect_output():
    clock = [0.0]
    authority = LeaseAuthority("private", clock=lambda: clock[0])
    grant = authority.issue("private", "session", 1, 1, 1.0)
    clock[0] = 1.1
    with pytest.raises(AccessError, match="SESSION_EXPIRED"):
        authority.issue("private", "session", 1, 2, 1.0, grant.token)
    assert authority.verify(grant.token, "session", 1, allow_expired=True) == grant


def test_wrong_coordinator_and_stale_sequence_fail():
    authority = LeaseAuthority("private")
    with pytest.raises(AccessError, match="UNAUTHORIZED"):
        authority.issue("intruder", "session", 1, 1, 1.0)
    grant = authority.issue("private", "session", 1, 1, 1.0)
    with pytest.raises(AccessError, match="STALE_SEQUENCE"):
        authority.issue("private", "session", 1, 1, 1.0, grant.token)
    with pytest.raises(AccessError, match="STALE_GENERATION"):
        authority.verify(grant.token, "session", 2)


def test_replay_cannot_extend_deadline_and_stop_can_verify_expired_capability():
    clock = [10.0]
    authority = LeaseAuthority("private", clock=lambda: clock[0])
    original = authority.issue("private", "session", 4, 10, 1.0)
    clock[0] = 10.5
    with pytest.raises(AccessError, match="STALE_SEQUENCE"):
        authority.issue("private", "session", 4, 9, 5.0, original.token)
    assert authority.grant.deadline == 11.0
    renewed = authority.issue("private", "session", 4, 11, 1.0, original.token)
    assert renewed.token == original.token and renewed.deadline == 11.5
    clock[0] = 12.0
    with pytest.raises(AccessError, match="SESSION_EXPIRED"):
        authority.verify(original.token, "session", 4)
    authority.verify(original.token, "session", 4, allow_expired=True)


def test_retired_executor_capability_only_exists_without_a_new_owner_and_expires():
    clock = [0.0]
    authority = LeaseAuthority("private", clock=lambda: clock[0])
    old = authority.issue("private", "old", 1, 1, 1.0)
    clock[0] = 1.1
    authority.revoke()
    assert authority.verify_retired(old.token, "old", 1) == old
    with pytest.raises(AccessError, match="INVALID_TOKEN"):
        authority.verify(old.token, "old", 1)
    new = authority.issue("private", "new", 2, 1, 1.0)
    with pytest.raises(AccessError, match="INVALID_RETIRED"):
        authority.verify_retired(old.token, "old", 1)
    authority.revoke()
    clock[0] = 7.0
    with pytest.raises(AccessError, match="INVALID_RETIRED"):
        authority.verify_retired(new.token, "new", 2)


def test_privileged_stop_bridge_spans_same_session_handover_but_not_new_owner():
    import pytest
    from gca_ros.security import AccessError, LeaseAuthority

    a = LeaseAuthority("coordinator-key", clock=lambda: 1.0)
    old = a.issue("coordinator-key", "first", 1, 1, 1.0)
    a.revoke()
    assert a.coordinator_stop("coordinator-key", "first") == old
    new = a.issue("coordinator-key", "first", 2, 2, 1.0)
    assert a.coordinator_stop("coordinator-key", "first") == new
    with pytest.raises(AccessError, match="UNAUTHORIZED_COORDINATOR"):
        a.coordinator_stop("wrong-key", "first")
    a.revoke()
    a.issue("coordinator-key", "second", 3, 1, 1.0)
    with pytest.raises(AccessError, match="STOP_SESSION_MISMATCH"):
        a.coordinator_stop("coordinator-key", "first")
