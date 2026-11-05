"""Bounded, rotating JSON-lines sink with explicit loss/error counters.

The producer only serializes an immutable record and does a nonblocking enqueue.
Only the writer thread opens files. A slow or failed filesystem cannot delay the
motion task. ``close`` has a wall deadline; an unresponsive sink is reported and
the daemon writer is left isolated, never waited on indefinitely.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from typing import TextIO


@dataclass(frozen=True)
class RecorderStatus:
    queued: int
    written: int
    dropped: int
    errors: int
    last_error: str


class AsyncRecorder:
    """One producer or multiple ROS observers, one isolated disk writer.

    Each record must contain only observations, never session credentials. A
    record larger than ``max_record_bytes`` is dropped with a counter. At most
    ``file_count * max_file_bytes`` bytes plus one bounded record are retained.
    An injectable opener permits deterministic slow/full disk tests.
    """

    def __init__(
        self,
        directory: str | Path,
        capacity: int = 1024,
        max_file_bytes: int = 2_000_000,
        file_count: int = 4,
        max_record_bytes: int = 16_384,
        opener: Callable[[Path], TextIO] | None = None,
    ):
        if min(capacity, max_file_bytes, file_count, max_record_bytes) <= 0:
            raise ValueError("Recorder bounds must be positive")
        self.directory = Path(directory)
        self.max_file_bytes = max_file_bytes
        self.file_count = file_count
        self.max_record_bytes = max_record_bytes
        self._open = opener or (lambda p: p.open("w", encoding="utf-8"))
        self._queue: Queue[str] = Queue(capacity)
        self._stop = Event()
        self._lock = Lock()
        self._written = self._dropped = self._errors = 0
        self._last_error = ""
        self._thread = Thread(target=self._writer, name="gca-log-writer", daemon=True)
        self._thread.start()

    def submit(self, record: Mapping) -> bool:
        """Nonblocking best-effort observation; a false result is counted loss."""
        try:
            if self._stop.is_set():
                raise ValueError("recorder is closing")
            if self._has_credentials(record):
                raise ValueError("credentials cannot be recorded")
            text = json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n"
            if len(text.encode("utf-8")) > min(self.max_record_bytes, self.max_file_bytes):
                raise ValueError("record exceeds bounded size")
            self._queue.put_nowait(text)
            return True
        except (Full, TypeError, ValueError):
            with self._lock:
                self._dropped += 1
            return False

    @staticmethod
    def _has_credentials(value) -> bool:
        if isinstance(value, Mapping):
            return any(
                key in ("token", "coordinator_key", "credentials")
                or AsyncRecorder._has_credentials(child)
                for key, child in value.items()
            )
        if isinstance(value, (list, tuple)):
            return any(AsyncRecorder._has_credentials(child) for child in value)
        return False

    def status(self) -> RecorderStatus:
        with self._lock:
            return RecorderStatus(
                self._queue.qsize(), self._written, self._dropped, self._errors, self._last_error
            )

    def close(self, timeout_s: float = 1.0) -> bool:
        """Drain within a wall deadline and report whether writer terminated."""
        self._stop.set()
        self._thread.join(max(0, timeout_s))
        return not self._thread.is_alive()

    def _writer(self) -> None:
        stream = None
        size = 0
        index = -1
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    text = self._queue.get(timeout=0.05)
                except Empty:
                    continue
                try:
                    byte_count = len(text.encode("utf-8"))
                    if stream is None or size + byte_count > self.max_file_bytes:
                        if stream is not None:
                            stream.close()
                        self.directory.mkdir(parents=True, exist_ok=True)
                        index = (index + 1) % self.file_count
                        stream = self._open(self.directory / f"control-{index:02d}.jsonl")
                        size = 0
                    stream.write(text)
                    stream.flush()
                    size += byte_count
                    with self._lock:
                        self._written += 1
                except (OSError, ValueError) as exc:
                    with self._lock:
                        self._errors += 1
                        self._dropped += 1
                        self._last_error = type(exc).__name__ + ": " + str(exc)[:200]
                    if stream is not None:
                        try:
                            stream.close()
                        except OSError as close_error:
                            with self._lock:
                                self._last_error += "; close: " + str(close_error)[:100]
                        stream = None
        finally:
            if stream is not None:
                try:
                    stream.close()
                except OSError as exc:
                    with self._lock:
                        self._errors += 1
                        self._last_error = "close: " + str(exc)[:200]
