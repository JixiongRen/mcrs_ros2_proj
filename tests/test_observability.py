"""Disk backpressure cannot reach the control thread (T12)."""

import io
import json
import time
from threading import Event

from gca.observability import AsyncRecorder


def test_slow_sink_queue_full_is_nonblocking_and_close_is_bounded(tmp_path):
    entered, release = Event(), Event()

    class Slow(io.StringIO):
        def write(self, value):
            entered.set()
            assert release.wait(2)
            return super().write(value)

    recorder = AsyncRecorder(tmp_path, capacity=1, opener=lambda _: Slow())
    try:
        assert recorder.submit({"seq": 1})
        assert entered.wait(1)
        assert recorder.submit({"seq": 2})
        start = time.monotonic()
        assert not recorder.submit({"seq": 3})
        assert time.monotonic() - start < 0.1
        assert recorder.status().dropped == 1
        assert not recorder.close(0.01)
    finally:
        release.set()
        assert recorder.close(1)


def test_full_disk_error_is_visible_and_not_raised_in_producer(tmp_path):
    def full(_):
        raise OSError(28, "No space left on device")

    recorder = AsyncRecorder(tmp_path, opener=full)
    assert recorder.submit({"sample_seq": 1})
    assert recorder.close(1)
    assert recorder.status().errors == 1
    assert recorder.status().dropped == 1


def test_records_rotate_with_bounded_size_and_reject_secrets(tmp_path):
    recorder = AsyncRecorder(tmp_path, max_file_bytes=50, file_count=2)
    for seq in range(20):
        assert recorder.submit({"seq": seq})
    assert not recorder.submit({"token": "secret"})
    assert not recorder.submit({"context": {"token": "secret"}})
    assert not recorder.submit({"value": "x" * 100})
    assert recorder.close(1)
    files = list(tmp_path.glob("*.jsonl"))
    assert len(files) == 2
    assert all(p.stat().st_size <= 50 for p in files)
    for path in files:
        for line in path.read_text().splitlines():
            assert set(json.loads(line)) == {"seq"}
    assert recorder.status().written == 20
