import pytest
from system_coordinator.authority import SessionError, SourceAuthority


def test_unique_owner_expiry_and_stale_generation():
    now = [0.0]
    authority = SourceAuthority(clock=lambda: now[0])
    source = authority.claim("operator", "MANUAL", 1.0)
    with pytest.raises(SessionError, match="CONTROL_BUSY"):
        authority.claim("other", "AUTO", 1.0)
    with pytest.raises(SessionError, match="STALE_GENERATION"):
        authority.verify(source.session_id, source.generation + 1, source.token)
    now[0] = 2.0
    with pytest.raises(SessionError, match="SESSION_EXPIRED"):
        authority.verify(source.session_id, source.generation, source.token)
    authority.verify(source.session_id, source.generation, source.token, allow_expired=True)
    authority.release()
    replacement = authority.claim("other", "AUTO", 1.0)
    assert replacement.generation > source.generation
    assert replacement.token != source.token


def test_explicit_reauthorization_rotates_generation_without_extending_source_lease():
    now = [20.0]
    authority = SourceAuthority(clock=lambda: now[0])
    source = authority.claim("operator", "AUTO", 2.0)
    old_generation = source.generation
    old_deadline = source.deadline
    source.executor_token = "old-device-capability"
    authority.rotate_generation(source)
    assert source.generation > old_generation
    assert source.deadline == old_deadline
    assert source.executor_token == ""
    with pytest.raises(SessionError, match="STALE_GENERATION"):
        authority.verify(source.session_id, old_generation, source.token)
    authority.verify(source.session_id, source.generation, source.token)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0, -1.0, True])
def test_authorization_and_rpc_budgets_reject_unbounded_values(value):
    from system_coordinator.authority import bounded_positive_seconds

    with pytest.raises(ValueError):
        SourceAuthority(maximum_s=value)
    with pytest.raises(ValueError):
        bounded_positive_seconds(value, "rpc_timeout_s")


def test_retired_source_can_be_recognized_without_resurrecting_control_or_affecting_new_owner():
    clock = [0.0]
    authority = SourceAuthority(clock=lambda: clock[0])
    old = authority.claim("operator", "AUTO", 1.0)
    authority.release()
    assert (
        authority.verify_retired(old.session_id, old.generation, old.token).session_id
        == old.session_id
    )
    with pytest.raises(SessionError, match="INVALID_TOKEN"):
        authority.verify(old.session_id, old.generation, old.token)
    new = authority.claim("replacement", "AUTO", 1.0)
    with pytest.raises(SessionError, match="INVALID_RETIRED"):
        authority.verify_retired(old.session_id, old.generation, old.token)
    authority.release(remember=False)
    assert (
        authority.verify_retired(old.session_id, old.generation, old.token).session_id
        == old.session_id
    )
    clock[0] = 6.0
    with pytest.raises(SessionError, match="INVALID_RETIRED"):
        authority.verify_retired(old.session_id, old.generation, old.token)
    assert new.session_id != old.session_id
