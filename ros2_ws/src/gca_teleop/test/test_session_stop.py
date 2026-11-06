"""Order stop receipts, stale DDS, and timeouts without a device or fake ROS API."""

import copy
import threading
from concurrent.futures import Future

import pytest
import rclpy
from gca_system_interfaces.msg import GcaState, StopStatus
from gca_system_interfaces.srv import ControlSession, GetCapabilities, ManageDevice
from gca_teleop.dispatch import LatestDispatcher
from gca_teleop.session import ManualSession, _Credentials
from rclpy.node import Node
from rclpy.task import Future as RosFuture


class ControlledClient:
    """Explicit transport port; the session still uses real generated messages."""

    def __init__(self, future_type=Future):
        self.requests = []
        self.pending = set()
        self.future_type = future_type

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = self.future_type()
        self.requests.append((request, future))
        self.pending.add(future)
        return future

    def remove_pending_request(self, future):
        self.pending.discard(future)


@pytest.fixture
def session():
    rclpy.init()
    node = Node("teleop_session_stop_test")
    s = ManualSession(node, timeout_s=0.05)
    s._clients = {name: ControlledClient() for name in s._clients}
    s._credentials = _Credentials("owner", 1, "private", 1)
    s._receive_state(state())
    s._backend = "simulation"
    yield s
    s._credentials = None
    s.close()
    node.destroy_node()
    rclpy.shutdown()


def state(
    *,
    sequence=10,
    serial=5,
    progress="CONFIRMED",
    name="READY",
    armed=True,
    generation=1,
    boot="boot",
):
    return GcaState(
        boot_id=boot,
        state_seq=sequence,
        stop_serial=serial,
        session_id="owner",
        generation=generation,
        state=name,
        armed=armed,
        lifecycle_active=True,
        linear_reference_valid=True,
        rotation_reference_valid=True,
        rotation_estimate_valid=True,
        reference_id="reference",
        requested_config_revision="revision",
        applied_config_revision="revision",
        stop=StopStatus(progress=progress),
    )


def reply_stop(s, snapshot):
    future = s._clients["request_stop"].requests[-1][1]
    future.set_result(ManageDevice.Response(accepted=True, state=snapshot))


def test_gate_requires_latest_stop_receipt_and_its_serial_confirmation(session):
    s = session
    assert s.gate()["ready"]
    s.request_stop("B")
    assert not s.latest_stop_confirmed
    assert not s.gate()["ready"]  # Even before STOP RPC reaches the receiver.
    reply_stop(s, state(serial=6, progress="REQUESTED"))
    assert not s.gate()["ready"]
    s._receive_state(state())  # Old CONFIRMED from before B cannot satisfy it.
    assert not s.gate()["ready"]
    s._receive_state(state(sequence=11, serial=6, progress="IN_PROGRESS"))
    assert not s.gate()["ready"]
    s._receive_state(state(sequence=12, serial=6))
    assert s.latest_stop_confirmed
    assert s.gate()["ready"]


def test_new_stop_queued_behind_inflight_needs_its_own_receipt(session):
    s = session
    s.request_stop("B")
    s.request_stop("LT_RELEASE")
    reply_stop(s, state(sequence=11, serial=6))
    assert len(s._clients["request_stop"].requests) == 2
    assert not s.gate()["ready"]
    reply_stop(s, state(sequence=12, serial=7, progress="REQUESTED"))
    s._receive_state(state(sequence=13, serial=6))
    assert not s.gate()["ready"]
    s._receive_state(state(sequence=14, serial=7))
    assert s.gate()["ready"]


def test_arm_reply_updates_cache_and_same_tick_old_dds_cannot_undo_it(session):
    s = session
    s._receive_state(state(sequence=20, name="DISARMED", armed=False))
    returned = state(sequence=20, generation=2)

    def call(name, request):
        if name == "arm":
            return ManageDevice.Response(accepted=True, state=returned)
        assert name == "acknowledge_generation"
        return ControlSession.Response(accepted=True)

    s._call = call
    s.manage("arm")
    assert s.gate()["ready"]
    s._receive_state(state(sequence=20, name="DISARMED", armed=False))
    assert s.state.armed and s.gate()["ready"]
    s._receive_state(state(sequence=21, generation=2))
    assert s.gate()["ready"]


def test_failed_arm_generation_ack_waits_for_new_stop_confirmation(session):
    s = session
    s.request_stop("ARM_FAILED")
    polled, unconfirmed, acknowledged = threading.Event(), threading.Event(), threading.Event()
    original_settled = s._stop_settled_locked
    acks = []

    def settled():
        polled.set()
        if s._stop_receipt is not None and s.state.stop.progress == "REQUESTED":
            unconfirmed.set()
        return original_settled()

    def call(name, request):
        assert name == "acknowledge_generation"
        acks.append(copy.deepcopy(request))
        acknowledged.set()
        return ControlSession.Response(accepted=True)

    s._stop_settled_locked = settled
    s._call = call
    thread = threading.Thread(target=s._ack_failed_arm_receipt)
    thread.start()
    try:
        assert polled.wait(0.5) and not acks
        reply_stop(
            s, state(sequence=11, serial=6, progress="REQUESTED", name="DISARMED", armed=False)
        )
        assert unconfirmed.wait(0.5) and not acks
        s._receive_state(state(sequence=12, serial=6, name="DISARMED", armed=False))
        assert acknowledged.wait(0.5)
        thread.join(0.5)
        assert not thread.is_alive() and len(acks) == 1
        assert not s.gate()["ready"]  # Receipt ACK never performs ARM.
    finally:
        s._closing.set()
        thread.join(1.0)


def test_management_timeouts_remove_client_pending_requests_and_cancel(session):
    s = session
    client = s._clients["arm"]
    for _ in range(3):
        with pytest.raises(TimeoutError, match="MANAGEMENT_RPC_TIMEOUT"):
            s._call("arm", ManageDevice.Request())
        assert not client.pending and not s._owned_futures
        assert client.requests[-1][1].cancelled()


def test_stop_timeout_is_fail_closed_and_does_not_reuse_cached_confirmation(session):
    s = session
    s.request_stop("B")
    future = s._stop_future
    with s._lock:
        abandoned = s._supervise_pending_locked(s._stop_started_ns + 100_000_000)
    for expired in abandoned:
        s._abandon(expired)
    assert future.cancelled()
    assert not s._clients["request_stop"].pending
    assert s.error == "STOP_TIMEOUT"
    assert not s.gate()["ready"]
    assert s._stop_confirmed_epoch < s._stop_required_epoch


@pytest.mark.parametrize("kind", ("stop", "capabilities", "renew"))
def test_timeout_cancellation_callbacks_cannot_reverse_session_dispatch_locks(session, kind):
    """A real unbound rclpy Future invokes cancellation callbacks inline.

    The timed acquisition observes the same session lock that guard() reads,
    while releasing the dispatcher lock on failure so this regression never
    leaves deadlocked threads behind, even against the faulty implementation.
    """
    s = session
    name = {"stop": "request_stop", "capabilities": "capabilities", "renew": "renew_control"}[kind]
    s._clients[name] = ControlledClient(RosFuture)
    s._management_active = True  # Isolate cancellation from starting a renewal.
    if kind == "stop":
        s.request_stop("B")
    else:
        request = GetCapabilities.Request() if kind == "capabilities" else ControlSession.Request()
        setattr(s, "_" + kind + "_future", s._send(name, request))
    setattr(s, "_" + kind + "_started_ns", 0)
    future = getattr(s, "_" + kind + "_future")
    dispatcher = LatestDispatcher(s, lambda *args: None)
    dispatcher_held, callback_entered = threading.Event(), threading.Event()
    observations, failures = [], []

    def canceled(done):
        callback_entered.set()
        dispatcher.guard()

    future.add_done_callback(canceled)

    def dispatcher_reader():
        try:
            with dispatcher._lock:
                dispatcher_held.set()
                if not callback_entered.wait(0.3):
                    raise AssertionError("timeout did not enter cancellation callback")
                acquired = s._lock.acquire(timeout=0.2)
                observations.append(acquired)
                if acquired:
                    try:
                        dispatcher.guard()
                    finally:
                        s._lock.release()
        except BaseException as exc:
            failures.append(exc)

    reader = threading.Thread(target=dispatcher_reader)
    reader.start()
    assert dispatcher_held.wait(0.1)
    supervisor = threading.Thread(target=s._renew_tick)
    supervisor.start()
    # Each blocked edge has a bounded release path; never detach leaked workers.
    reader.join(0.6)
    supervisor.join(0.3)
    assert not reader.is_alive() and not supervisor.is_alive()
    assert not failures, failures
    assert observations == [True], "Future.cancel held session lock while callback needed dispatch"
    assert future.cancelled()
    assert future not in s._clients[name].pending


def test_close_reclaims_every_owned_future_including_permits_and_capabilities(session):
    s = session
    permit = s._send("get_motion_permit", ControlSession.Request())
    caps = s._send("capabilities", GetCapabilities.Request())
    renewal = s._send("renew_control", ControlSession.Request())
    s._credentials = None
    s.close()
    assert not s._owned_futures
    assert all(future.cancelled() for future in (permit, caps, renewal))
    assert all(not client.pending for client in s._clients.values())


def test_empty_or_retired_boot_cannot_erase_authoritative_state(session):
    s = session
    before = s.state
    with s._lock:
        assert not s._merge_state_locked(GcaState(), authoritative=True)
    assert s.state == before
    s._receive_state(state(sequence=1, serial=1, boot="new-boot"))
    s._receive_state(state(sequence=100, serial=100))
    assert s.state.boot_id == "new-boot"
    assert not s.gate()["ready"]  # Actual backend must be reconfirmed after reboot.


def test_closing_unblocks_management_wait_and_cleans_transport_pending(session):
    s = session
    sent, finished = threading.Event(), threading.Event()
    errors = []
    client = s._clients["arm"]
    original = client.call_async

    def call(request):
        future = original(request)
        sent.set()
        return future

    def wait_management():
        try:
            s._call("arm", ManageDevice.Request())
        except Exception as exc:
            errors.append(exc)
        finally:
            finished.set()

    client.call_async = call
    thread = threading.Thread(target=wait_management)
    thread.start()
    try:
        assert sent.wait(0.5)
        s._closing.set()
        assert finished.wait(0.5)
        thread.join(0.5)
        assert len(errors) == 1
        assert not client.pending and not s._owned_futures
    finally:
        thread.join(1.0)


def test_stop_receipt_from_another_boot_does_not_confirm_cached_state(session):
    s = session
    s.request_stop("B")
    reply_stop(s, state(sequence=1, serial=1, progress="REQUESTED", boot="new-boot"))
    s._receive_state(state(sequence=100, serial=100))
    assert s._stop_confirmed_epoch < s._stop_required_epoch
    assert not s.gate()["ready"]
    s._receive_state(state(sequence=2, serial=1, boot="new-boot"))
    assert s._stop_confirmed_epoch == s._stop_required_epoch


def test_explicit_release_with_stale_generation_reports_failure_but_drops_local_authority(session):
    s = session

    def call(name, request):
        assert name == "release_control"
        return ControlSession.Response(accepted=False, error_code="INVALID_RETIRED_CAPABILITY")

    s._call = call
    with pytest.raises(RuntimeError, match="INVALID_RETIRED_CAPABILITY"):
        s.manage("release")
    assert s.identity() == ("", 0)
    assert s.error.startswith("RELEASE_FAILED:")
    assert not s._clients["request_stop"].pending
    assert not s._owned_futures
    assert not s.gate()["ready"]
