from dataclasses import replace

import pytest
from gca.domain.manual import ManualInputQuality, QualityError, QualityTracker


def sample(kind="SCRIPT_SAMPLE", **kw):
    return replace(
        ManualInputQuality(
            kind,
            "reader",
            "epoch",
            1,
            1_000_000_000,
            "MONOTONIC",
            1,
            1_000_000_000,
            1_100_000_000,
            "boot",
            "UNKNOWN",
        ),
        **kw,
    )


def tracker(**kw):
    return QualityTracker(clock_domain_id="boot", **kw)


def test_deadline_not_reset_by_reception_or_renewal():
    t = tracker()
    assert t.validate(sample(), 1_050_000_000) == 1_100_000_000
    with pytest.raises(QualityError, match="INPUT_EXPIRED"):
        t.validate(sample(), 1_100_000_000)


def test_event_age_reader_heartbeat_cannot_extend_event_deadline():
    t = tracker()
    q = sample("EVENT_AGE")
    t.validate(q, q.event_ns)
    with pytest.raises(QualityError, match="INPUT_DEADLINE_REWRITTEN"):
        t.validate(
            replace(
                q,
                observation_seq=2,
                observation_ns=q.event_ns + 10_000_000,
                valid_until_ns=q.valid_until_ns + 10_000_000,
            ),
            q.event_ns + 10_000_000,
        )


def test_same_qualified_event_new_observation_is_not_radio_event():
    t = tracker()
    q = sample(
        "QUALIFIED_STATE_CONTRACT",
        qualification_id="evidence",
        link_evidence="QUALIFIED_STATE_CONTRACT",
    )
    t.validate(q, q.event_ns)
    newer = replace(
        q,
        observation_seq=2,
        observation_ns=q.event_ns + 100_000_000,
        valid_until_ns=q.valid_until_ns + 100_000_000,
    )
    assert t.validate(newer, newer.observation_ns) == newer.valid_until_ns
    assert newer.event_ns == q.event_ns


def test_program_episode_deadline_is_receiver_pinned():
    t = tracker()
    q = sample("PROGRAMMED_JOG", episode_end_ns=2_000_000_000)
    t.validate(q, q.event_ns)
    with pytest.raises(QualityError, match="PROGRAMMED_EPISODE_REWRITTEN"):
        t.validate(
            replace(
                q,
                observation_seq=2,
                observation_ns=q.event_ns + 10_000_000,
                episode_end_ns=3_000_000_000,
            ),
            q.event_ns + 10_000_000,
        )


def test_epoch_cannot_return_and_history_capacity_never_evicts():
    t = tracker(capacity=2)
    q = sample()
    t.validate(q, q.event_ns)
    t.validate(replace(q, input_epoch="new"), q.event_ns)
    with pytest.raises(QualityError, match="INPUT_EPOCH_REPLAY"):
        t.validate(q, q.event_ns)
    with pytest.raises(QualityError, match="INPUT_HISTORY_CAPACITY"):
        t.validate(replace(q, input_epoch="third"), q.event_ns)


@pytest.mark.parametrize(
    "updates,code",
    [
        ({"clock_domain_id": "other"}, "INPUT_CLOCK_MISMATCH"),
        ({"observation_ns": 2_000_000_000}, "INPUT_TIME_INVALID"),
        ({"valid_until_ns": 1_900_000_000}, "INPUT_DEADLINE_INVALID"),
        ({"event_seq": True}, "INVALID_MANUAL_QUALITY"),
        ({"event_ns": -1}, "INVALID_MANUAL_QUALITY"),
    ],
)
def test_invalid_metadata(updates, code):
    with pytest.raises(QualityError, match=code):
        tracker().validate(sample(**updates), 1_000_000_000)


def test_hardware_rejects_script_and_unqualified_physical():
    for kind in ("SCRIPT_SAMPLE", "EVENT_AGE"):
        with pytest.raises(QualityError):
            tracker(backend="hardware").validate(sample(kind), 1_000_000_000)
