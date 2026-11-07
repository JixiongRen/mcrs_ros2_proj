from concurrent.futures import Future
from threading import Event, Thread
from types import SimpleNamespace

import pytest
from gca_teleop.dispatch import LatestDispatcher


class Clock:
    now = 1_000_000_000

    def __call__(self):
        return self.now


class Session:
    def __init__(self):
        self.stop_epoch = 0
        self.sequence = 0
        self.requests = []
        self.current = ("owner", 1)

    def identity(self):
        return self.current

    def context(self):
        self.sequence += 1
        return SimpleNamespace(session_id="owner", generation=1, sequence=self.sequence), "private"

    def permit_async(self, context, token, ttl):
        future = Future()
        self.requests.append((context, ttl, future))
        return future


def intent(clock, observation=1, velocity=2.0):
    return SimpleNamespace(
        action="JOG",
        deadman=True,
        linear_mm_s=velocity,
        rotation_deg_s=0.0,
        control_policy="LEGACY_DUAL_AXIS",
        servo_rpm=0.0,
        quality=SimpleNamespace(
            valid_until_ns=clock.now + 150_000_000, input_epoch="input", observation_seq=observation
        ),
    )


def test_latest_slot_does_not_starve_valid_inflight_when_observation_or_renewal_advances():
    clock, session, sent = Clock(), Session(), []
    dispatcher = LatestDispatcher(
        session, lambda i, c, t: sent.append((i, c.sequence)), clock=clock
    )
    first = intent(clock)
    dispatcher.offer(first)
    dispatcher.pump()
    for sample in range(2, 20):
        dispatcher.offer(intent(clock, sample, sample / 10))
        session.context()  # Source renewal has a newer global sequence.
        dispatcher.pump()
    assert len(session.requests) == 1
    assert dispatcher.pending_count == dispatcher.inflight_count == 1
    session.requests[0][2].set_result(SimpleNamespace(accepted=True, token="nonce"))
    assert sent == [(first, 1)]
    dispatcher.pump()
    assert len(session.requests) == 2
    session.requests[1][2].set_result(SimpleNamespace(accepted=True, token="nonce2"))
    assert sent[-1][0].linear_mm_s == 1.9
    assert dispatcher.sent_count == 2


def test_stop_or_owner_change_drops_late_reply_without_publishing():
    for trigger in ("stop", "owner", "invalidate"):
        clock, session, sent = Clock(), Session(), []
        dispatcher = LatestDispatcher(session, lambda *args: sent.append(args), clock=clock)
        dispatcher.offer(intent(clock))
        dispatcher.pump()
        future = session.requests[0][2]
        if trigger == "stop":
            session.stop_epoch += 1
        elif trigger == "owner":
            session.current = ("new-owner", 2)
        else:
            dispatcher.invalidate()
        if not future.cancelled():
            future.set_result(SimpleNamespace(accepted=True, token="old"))
        assert not sent
        assert dispatcher.inflight_count == 0


@pytest.mark.parametrize("max_ttl_s", (0.15, 0.30))
def test_original_input_deadline_is_not_extended_by_slow_permit(max_ttl_s):
    clock, session, sent, failures = Clock(), Session(), [], []
    dispatcher = LatestDispatcher(
        session,
        lambda *args: sent.append(args),
        max_ttl_s=max_ttl_s,
        clock=clock,
        failure=failures.append,
    )
    first = intent(clock)
    clock.now += 100_000_000
    dispatcher.offer(first)
    dispatcher.pump()
    assert abs(session.requests[0][1] - 0.05) < 1e-12
    clock.now += 50_000_000
    dispatcher.pump()
    assert failures == ["PERMIT_INPUT_EXPIRED"]
    assert not sent and dispatcher.inflight_count == 0


@pytest.mark.parametrize("options,expected_ttl", (({}, 0.15), ({"max_ttl_s": 0.30}, 0.25)))
def test_permit_ttl_uses_configured_limit_and_original_sample_deadline(options, expected_ttl):
    clock, session, sent = Clock(), Session(), []
    dispatcher = LatestDispatcher(
        session, lambda *args: sent.append(args), clock=clock, **options
    )
    first = intent(clock)
    deadline = clock.now + 300_000_000
    first.quality.valid_until_ns = deadline
    clock.now += 50_000_000
    dispatcher.offer(first)
    dispatcher.pump()
    assert session.requests[0][1] == pytest.approx(expected_ttl)
    clock.now += 50_000_000
    session.requests[0][2].set_result(SimpleNamespace(accepted=True, token="nonce"))
    assert len(sent) == 1 and sent[0][0] is first
    assert sent[0][0].quality.valid_until_ns == deadline


def test_rejected_permit_requests_stop_and_never_publishes():
    clock, session, failures = Clock(), Session(), []
    dispatcher = LatestDispatcher(
        session,
        lambda *args: (_ for _ in ()).throw(AssertionError("publish")),
        clock=clock,
        failure=failures.append,
    )
    dispatcher.offer(intent(clock))
    dispatcher.pump()
    session.requests[0][2].set_result(SimpleNamespace(accepted=False, error_code="FENCED"))
    assert failures == ["PERMIT_REPLY:FENCED"]


def test_slow_publish_keeps_one_slot_but_cannot_hold_stop_invalidation_lock():
    entered, release, stopped = Event(), Event(), Event()
    clock, session = Clock(), Session()

    def publish(*args):
        entered.set()
        assert release.wait(2)

    dispatcher = LatestDispatcher(session, publish, clock=clock)
    dispatcher.offer(intent(clock))
    dispatcher.pump()
    thread = Thread(
        target=lambda: session.requests[0][2].set_result(
            SimpleNamespace(accepted=True, token="nonce")
        )
    )
    thread.start()
    try:
        assert entered.wait(1)
        session.stop_epoch += 1

        def stop():
            dispatcher.invalidate()
            stopped.set()

        stopper = Thread(target=stop)
        stopper.start()
        assert stopped.wait(0.5), "STOP must not wait for DDS publication"
        stopper.join(1)
        for i in range(10):
            dispatcher.offer(intent(clock, i + 2))
            dispatcher.pump()
        assert len(session.requests) == 1
        assert dispatcher.inflight_count == dispatcher.pending_count == 1
    finally:
        release.set()
        thread.join(2)
    assert dispatcher.inflight_count == 0


def test_stop_between_computation_offer_or_context_cannot_mint_fresh_sequence_for_old_intent():
    for stage in ("before_offer", "pending", "context", "permit"):
        clock, session, sent = Clock(), Session(), []
        dispatcher = LatestDispatcher(session, lambda *args: sent.append(args), clock=clock)
        guard = dispatcher.guard()
        old = intent(clock)
        if stage == "before_offer":
            session.stop_epoch += 1
        dispatcher.offer(old, guard)
        if stage == "pending":
            session.stop_epoch += 1
        if stage == "context":
            original_context = session.context

            def racing_context():
                session.stop_epoch += 1
                return original_context()

            session.context = racing_context
        if stage == "permit":
            original_permit = session.permit_async

            def racing_permit(*args):
                session.stop_epoch += 1
                return original_permit(*args)

            session.permit_async = racing_permit
        dispatcher.pump()
        for _, _, future in session.requests:
            future.set_result(SimpleNamespace(accepted=True, token="old"))
        assert not sent, stage
        assert dispatcher.pending_count == dispatcher.inflight_count == 0
        if stage != "permit":
            assert not session.requests


def test_already_completed_permit_publishes_outside_pump_mutex():
    clock, session = Clock(), Session()
    entered, release, stopped = Event(), Event(), Event()

    def immediate_permit(*args):
        future = Future()
        future.set_result(SimpleNamespace(accepted=True, token="nonce"))
        return future

    def publish(*args):
        entered.set()
        assert release.wait(2)

    session.permit_async = immediate_permit
    dispatcher = LatestDispatcher(session, publish, clock=clock)
    dispatcher.offer(intent(clock))
    pump = Thread(target=dispatcher.pump)
    pump.start()
    try:
        assert entered.wait(1)

        def stop():
            dispatcher.invalidate()
            stopped.set()

        stopper = Thread(target=stop)
        stopper.start()
        assert stopped.wait(0.5), "synchronous callback must not hold pump mutex"
        stopper.join(1)
    finally:
        release.set()
        pump.join(2)
    assert dispatcher.inflight_count == 0
