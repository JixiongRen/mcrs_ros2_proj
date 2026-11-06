"""A tick must date the health snapshot it actually reads under the input lock."""

from dataclasses import replace
from threading import Event, RLock
from types import SimpleNamespace

import pytest
from gca_teleop.node import TeleopNode
from gca_teleop.state_machine import TeleopPipeline
from test_input_pipeline import Frames, activate, synthetic


def tick_fixture(monkeypatch):
    frames = Frames()
    profile = synthetic()
    pipeline = TeleopPipeline(profile)
    activate(pipeline, frames, {1: -16000})
    clock = [frames.now + 1_000_000]
    monkeypatch.setattr("gca_teleop.node.time.monotonic_ns", lambda: clock[0])
    accepted, offered = [], []
    dispatcher = SimpleNamespace(
        guard=lambda: "original-guard",
        offer=lambda intent, guard: offered.append((intent, guard)),
        pump=lambda: None,
    )
    node = SimpleNamespace(
        _closing=Event(),
        _dispatcher=dispatcher,
        _session=SimpleNamespace(
            backend="simulation",
            gate=lambda: dict(armed=True, ready=True, reference_id="r1", session_key="s1"),
        ),
        _pipeline_lock=RLock(),
        _pipeline=pipeline,
        _health=frames.health(),
        _accept_intent=accepted.append,
        _dispatch_after_ns=0,
        profile=profile,
    )
    return node, clock, accepted, offered


@pytest.mark.parametrize("interleaving", ["during_guard", "before_input_lock"])
def test_tick_new_reader_heartbeat_after_callback_entry_is_not_a_future_clock_fault(
    monkeypatch, interleaving
):
    node, clock, accepted, offered = tick_fixture(monkeypatch)

    def advance_reader():
        # The reader progressed 200 us while this callback was obtaining other
        # snapshots/locks. This heartbeat is current, not a future device time.
        clock[0] += 200_000
        node._health = replace(node._health, reader_heartbeat_ns=clock[0])

    if interleaving == "during_guard":

        def guard():
            advance_reader()
            return "original-guard"

        node._dispatcher.guard = guard
    else:

        class ReaderWinsLock:
            def __enter__(self):
                advance_reader()

            def __exit__(self, *args):
                return False

        node._pipeline_lock = ReaderWinsLock()

    TeleopNode._tick(node)
    assert accepted[-1].action == "JOG", accepted[-1].reason
    assert accepted[-1].linear_mm_s > 0
    assert offered == [(accepted[-1], "original-guard")]


@pytest.mark.parametrize("health_age_ns", [-1, 100_000_000])
def test_tick_still_rejects_genuinely_future_or_expired_reader_heartbeat(
    monkeypatch, health_age_ns
):
    node, clock, accepted, offered = tick_fixture(monkeypatch)
    node._health = replace(node._health, reader_heartbeat_ns=clock[0] - health_age_ns)
    TeleopNode._tick(node)
    assert accepted[-1].action == "STOP"
    assert accepted[-1].reason == "READER_STALE"
    assert not offered
