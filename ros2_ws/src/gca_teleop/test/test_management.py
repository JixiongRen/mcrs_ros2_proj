import os
import socket
import threading

import pytest
from gca_teleop.management import ManagementServer, call, stop_socket


def message(op, request_id="request"):
    return {"op": op, "request_id": request_id, "expected": {}, "args": {}}


def test_slow_management_cannot_block_dedicated_stop_and_request_retry_is_idempotent(tmp_path):
    entered, release, stop_seen = threading.Event(), threading.Event(), threading.Event()
    calls = []

    def handler(request):
        calls.append(request["op"])
        if request["op"] == "arm":
            entered.set()
            assert release.wait(2)
        if request["op"] == "stop":
            stop_seen.set()
        return {"accepted": True}

    path = tmp_path / "private" / "control.sock"
    server = ManagementServer(path, handler)
    server.start()
    responses = []
    thread = threading.Thread(target=lambda: responses.append(call(path, message("arm"))))
    try:
        thread.start()
        assert entered.wait(1)
        assert call(path, message("stop", "stop"), timeout_s=0.5)["accepted"]
        assert stop_seen.is_set() and thread.is_alive()
        assert call(path, message("arm"))["error"] == "REQUEST_IN_PROGRESS"
        release.set()
        thread.join(1)
        assert responses == [{"accepted": True}]
        assert call(path, message("arm"))["accepted"]
        assert calls.count("arm") == 1
        assert os.stat(path).st_mode & 0o777 == 0o600
        assert os.stat(path.parent).st_mode & 0o777 == 0o700
    finally:
        release.set()
        thread.join(2)
        assert server.close()
    assert not path.exists() and not stop_socket(path).exists()


def test_ipc_rejects_unknown_operations_conflicting_ids_and_oversized_frames(tmp_path):
    path = tmp_path / "private" / "control.sock"
    server = ManagementServer(path, lambda req: {"accepted": True}, timeout_s=0.2)
    server.start()
    try:
        assert not call(path, message("arbitrary-shell"))["accepted"]
        assert call(path, message("status"))["accepted"]
        assert call(path, message("arm"))["error"] == "REQUEST_ID_CONFLICT"
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as peer:
            peer.settimeout(1)
            peer.connect(str(path))
            peer.sendall(b"x" * 4097)
            assert b"MESSAGE_TOO_LARGE" in peer.recv(1024)
        assert call(path, message("status", "alive"))["accepted"]
        assert not call(path, message("arm", "wrong-lane"), priority=True)["accepted"]
    finally:
        assert server.close()


def test_socket_owner_lock_and_filesystem_validation_never_replace_user_file(tmp_path):
    path = tmp_path / "private" / "control.sock"
    server = ManagementServer(path, lambda req: {"accepted": True})
    server.start()
    second = ManagementServer(path, lambda req: {})
    try:
        with pytest.raises(BlockingIOError):
            second.start()
        assert call(path, message("status"))["accepted"]
    finally:
        assert server.close()
    path.write_text("user file")
    with pytest.raises(ValueError, match="SOCKET_PATH_OCCUPIED"):
        ManagementServer(path, lambda req: {}).start()
    assert path.read_text() == "user file"
    public = tmp_path / "public"
    public.mkdir(mode=0o755)
    public.chmod(0o755)
    with pytest.raises(ValueError, match="PRIVATE_SOCKET_DIRECTORY_REQUIRED"):
        ManagementServer(public / "socket", lambda req: {}).start()


def test_closing_ipc_bounds_an_unresponsive_administrative_handler(tmp_path):
    entered, release = threading.Event(), threading.Event()

    def handler(request):
        entered.set()
        release.wait(2)
        return {"accepted": True}

    path = tmp_path / "private" / "control.sock"
    server = ManagementServer(path, handler)
    server.start()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as peer:
        peer.connect(str(path))
        peer.sendall(b'{"op":"status","request_id":"blocking"}\n')
        assert entered.wait(1)
        try:
            assert server.close(timeout_s=0.02) is False
        finally:
            release.set()
    assert server.close(timeout_s=1)


def test_message_receive_has_whole_frame_deadline_not_a_new_budget_per_byte(monkeypatch):
    import gca_teleop.management as module

    clock = [10.0]

    class SlowPeer:
        def gettimeout(self):
            return 0.05

        def settimeout(self, remaining):
            assert 0 < remaining <= 0.05 + 1e-9

        def recv(self, count):
            clock[0] += 0.02
            return b" "  # Each byte arrives within a socket timeout.

    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    with pytest.raises(TimeoutError, match="MESSAGE_DEADLINE_EXPIRED"):
        module._receive(SlowPeer(), 4096)
    assert clock[0] < 10.1


def test_deep_json_cannot_kill_ipc_workers(tmp_path):
    path = tmp_path / "private" / "control.sock"
    server = ManagementServer(path, lambda req: {"accepted": True})
    server.start()
    try:
        for _ in range(4):
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as peer:
                peer.settimeout(1)
                peer.connect(str(path))
                peer.sendall(b"[" * 1500 + b"]" * 1500 + b"\n")
                assert b"JSON_NESTING_TOO_DEEP" in peer.recv(1024)
        assert call(path, message("status", "still-alive"))["accepted"]
    finally:
        assert server.close()
