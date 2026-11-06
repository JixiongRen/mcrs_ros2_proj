from threading import Event, RLock
from types import SimpleNamespace

from gca_teleop.dispatch import LatestDispatcher
from gca_teleop.node import TeleopNode


def test_stop_after_gate_snapshot_cannot_pair_old_ready_with_new_stop_epoch():
    session = SimpleNamespace(stop_epoch=0, backend="simulation", identity=lambda: ("owner", 1))
    requests = []
    session.context = lambda: requests.append("unexpected context")

    def gate_then_stop():
        old = {"armed": True, "ready": True}
        session.stop_epoch += 1  # Renewal failure after taking the old snapshot.
        return old

    session.gate = gate_then_stop
    dispatcher = LatestDispatcher(session, lambda *args: None)
    intent = SimpleNamespace(
        action="JOG",
        deadman=True,
        quality=SimpleNamespace(source_kind="EVENT_AGE", valid_until_ns=2**63),
    )
    node = SimpleNamespace(
        _closing=Event(),
        _dispatcher=dispatcher,
        _session=session,
        _pipeline_lock=RLock(),
        _pipeline=SimpleNamespace(tick=lambda *a, **kw: intent),
        _health=None,
        _accept_intent=lambda value: None,
        _dispatch_after_ns=0,
        profile=SimpleNamespace(dispatch_hz=25),
    )
    TeleopNode._tick(node)
    assert not requests
    assert dispatcher.pending_count == dispatcher.inflight_count == 0
    assert dispatcher.dropped_count == 1
