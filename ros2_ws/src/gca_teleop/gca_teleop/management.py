"""Bounded local management IPC. Credentials never leave the session owner.

The STOP socket has its own acceptor and workers; slow ordinary management
cannot occupy them. Socket peers must have the process uid. The filesystem
directory, socket and ownership lock are private, and existing files are never
blindly removed. This is a local same-user interface, not network authentication.
"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import socket
import stat
import struct
import threading
import time
from collections import OrderedDict
from pathlib import Path
from queue import Empty, Full, Queue
from typing import Callable

MAX_MESSAGE = 4096
MAX_RESPONSE = 32768
OPERATIONS = frozenset(
    {
        "status",
        "configure",
        "activate",
        "claim",
        "reference",
        "arm",
        "disarm",
        "reset",
        "stop",
        "release",
        "input-config",
        "script-start",
        "script-stop",
        "simulation-fault",
    }
)


def default_socket() -> Path:
    return Path(f"/tmp/gca-teleop-{os.getuid()}") / "control.sock"


def stop_socket(path: Path) -> Path:
    return path.with_name(path.name + ".stop")


def _invalid_number(value):
    raise ValueError("NON_FINITE_JSON")


def _receive(connection: socket.socket, limit: int, deadline=None) -> dict:
    deadline = (
        deadline if deadline is not None else time.monotonic() + (connection.gettimeout() or 6.0)
    )
    data = bytearray()
    while b"\n" not in data:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("MESSAGE_DEADLINE_EXPIRED")
        connection.settimeout(remaining)
        block = connection.recv(min(1024, limit + 1 - len(data)))
        if not block:
            raise ValueError("INCOMPLETE_MESSAGE")
        data.extend(block)
        if len(data) > limit:
            raise ValueError("MESSAGE_TOO_LARGE")
    line, extra = bytes(data).split(b"\n", 1)
    if extra:
        raise ValueError("ONE_REQUEST_PER_CONNECTION")
    depth, quoted, escaped = 0, False, False
    for byte in line:
        if quoted:
            if escaped:
                escaped = False
            elif byte == 92:
                escaped = True
            elif byte == 34:
                quoted = False
        elif byte == 34:
            quoted = True
        elif byte in (91, 123):
            depth += 1
            if depth > 16:
                raise ValueError("JSON_NESTING_TOO_DEEP")
        elif byte in (93, 125):
            depth -= 1
    try:
        result = json.loads(line, parse_constant=_invalid_number)
    except RecursionError:
        raise ValueError("JSON_NESTING_TOO_DEEP") from None
    if not isinstance(result, dict):
        raise ValueError("OBJECT_REQUIRED")
    return result


def _encoded(value: dict, limit: int) -> bytes:
    data = json.dumps(value, allow_nan=False, separators=(",", ":")).encode() + b"\n"
    if len(data) > limit:
        raise ValueError("MESSAGE_TOO_LARGE")
    return data


def call(path: Path, request: dict, timeout_s: float = 6.0, *, priority=False) -> dict:
    deadline = time.monotonic() + timeout_s
    endpoint = stop_socket(path) if priority or request.get("op") == "stop" else path
    # STOP can fetch a status on the dedicated endpoint without ordinary RPCs.
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout_s)
        connection.connect(str(endpoint))
        connection.sendall(_encoded(request, MAX_MESSAGE))
        return _receive(connection, MAX_RESPONSE, deadline)


class ManagementServer:
    def __init__(self, path: Path, handler: Callable[[dict], dict], timeout_s: float = 6.0):
        self.path, self.handler, self.timeout_s = Path(path), handler, timeout_s
        self._closing = threading.Event()
        self._history = OrderedDict()
        self._lock = threading.Lock()
        self._listeners = []
        self._threads = []
        self._queues = []
        self._connections = set()
        self._ownership = None
        self._socket_inodes = {}

    def start(self):
        parent = self.path.parent
        parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        info = parent.lstat()
        if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError("PRIVATE_SOCKET_DIRECTORY_REQUIRED")
        lock_path = self.path.with_name(self.path.name + ".lock")
        descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
                raise ValueError("PRIVATE_LOCK_REQUIRED")
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(descriptor)
            raise
        self._ownership = descriptor
        try:
            for path, is_stop in ((self.path, False), (stop_socket(self.path), True)):
                if path.exists() or path.is_symlink():
                    info = path.lstat()
                    if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
                        raise ValueError("SOCKET_PATH_OCCUPIED")
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                        probe.settimeout(0.1)
                        try:
                            probe.connect(str(path))
                        except ConnectionRefusedError:
                            path.unlink()
                        else:
                            raise ValueError("SOCKET_ALREADY_LIVE")
                listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                listener.bind(str(path))
                os.chmod(path, 0o600)
                self._socket_inodes[path] = path.lstat().st_ino
                listener.listen(4)
                listener.settimeout(0.1)
                self._listeners.append(listener)
                queue = Queue(4)
                self._queues.append(queue)
                for _ in range(2):
                    worker = threading.Thread(
                        target=self._worker, args=(queue, is_stop), daemon=True
                    )
                    self._threads.append(worker)
                    worker.start()
                thread = threading.Thread(target=self._accept, args=(listener, queue), daemon=True)
                self._threads.append(thread)
                thread.start()
        except BaseException:
            self.close()
            raise

    def _accept(self, listener, queue):
        while not self._closing.is_set():
            try:
                connection, _ = listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            try:
                queue.put_nowait((connection, time.monotonic() + min(0.5, self.timeout_s)))
            except Full:
                connection.close()

    def _worker(self, queue, is_stop):
        while not self._closing.is_set():
            try:
                connection, deadline = queue.get(timeout=0.05)
            except Empty:
                continue
            with self._lock:
                self._connections.add(connection)
            try:
                self._serve(connection, is_stop, deadline)
            finally:
                with self._lock:
                    self._connections.discard(connection)

    def _serve(self, connection, is_stop, deadline):
        try:
            with connection:
                connection.settimeout(self.timeout_s)
                peer = connection.getsockopt(
                    socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")
                )
                _, uid, _ = struct.unpack("3i", peer)
                if uid != os.getuid():
                    raise ValueError("PEER_UID_REJECTED")
                try:
                    request = _receive(connection, MAX_MESSAGE, deadline)
                    if time.monotonic() >= deadline:
                        raise TimeoutError("MESSAGE_DEADLINE_EXPIRED")
                    response = self._dispatch(request, is_stop)
                except (ValueError, TypeError, KeyError, TimeoutError, RuntimeError) as exc:
                    response = {"accepted": False, "error": str(exc)[:512]}
                connection.sendall(_encoded(response, MAX_RESPONSE))
        except (OSError, ValueError):
            pass  # Malformed/closed local connection cannot kill the acceptor.

    def _dispatch(self, request, is_stop):
        if set(request) - {"request_id", "op", "expected", "args"}:
            raise ValueError("UNKNOWN_REQUEST_FIELD")
        op, request_id = request.get("op"), request.get("request_id")
        if op not in OPERATIONS or (is_stop and op not in ("stop", "status")):
            raise ValueError("OPERATION_NOT_ALLOWED")
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise ValueError("REQUEST_ID_REQUIRED")
        if not isinstance(request.get("args", {}), dict):
            raise ValueError("ARGS_OBJECT_REQUIRED")
        if op != "status" and not isinstance(request.get("expected"), dict):
            raise ValueError("EXPECTED_STATE_REQUIRED")
        fingerprint = hashlib.sha256(_encoded(request, MAX_MESSAGE)).hexdigest()
        with self._lock:
            old = self._history.get(request_id)
            if old is not None:
                if old[0] != fingerprint:
                    raise ValueError("REQUEST_ID_CONFLICT")
                return old[1] or {"accepted": False, "error": "REQUEST_IN_PROGRESS"}
            if len(self._history) >= 256:
                # Do not evict in-flight entries, which would permit duplicates.
                removable = next(
                    (key for key, entry in self._history.items() if entry[1] is not None), None
                )
                if removable is None:
                    raise ValueError("REQUEST_CAPACITY_EXCEEDED")
                del self._history[removable]
            self._history[request_id] = (fingerprint, None)
        try:
            response = self.handler(request)
        except Exception as exc:
            response = {"accepted": False, "error": f"{type(exc).__name__}: {exc}"[:512]}
        with self._lock:
            self._history[request_id] = (fingerprint, response)
        return response

    def close(self, timeout_s=1.0):
        self._closing.set()
        for listener in self._listeners:
            listener.close()
        with self._lock:
            for connection in self._connections:
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        for queue in self._queues:
            while True:
                try:
                    queue.get_nowait()[0].close()
                except Empty:
                    break
        deadline = time.monotonic() + timeout_s
        for thread in self._threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        # An acceptor already past accept() may have enqueued during shutdown.
        for queue in self._queues:
            while True:
                try:
                    queue.get_nowait()[0].close()
                except Empty:
                    break
        for path, inode in self._socket_inodes.items():
            try:
                if path.lstat().st_ino == inode and stat.S_ISSOCK(path.lstat().st_mode):
                    path.unlink()
            except FileNotFoundError:
                pass
        if self._ownership is not None:
            os.close(self._ownership)
            self._ownership = None
        return not any(thread.is_alive() for thread in self._threads)
