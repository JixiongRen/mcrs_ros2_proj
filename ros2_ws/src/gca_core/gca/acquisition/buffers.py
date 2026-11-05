"""Short-lock latest-value mailbox and bounded event transport."""

from collections import deque
from threading import Lock


class Latest:
    """Capacity-one immutable-value exchange; no I/O occurs under its lock."""

    def __init__(self, initial=None):
        self._value, self._lock = initial, Lock()

    def put(self, value):
        with self._lock:
            self._value = value

    def get(self):
        with self._lock:
            return self._value


class BoundedEvents:
    """Reliable events reject overflow; callers must fault rather than drop it."""

    def __init__(self, capacity):
        self._queue, self._lock, self.capacity = deque(), Lock(), capacity

    def put(self, item):
        with self._lock:
            if len(self._queue) >= self.capacity:
                raise OverflowError("reliable event queue full")
            self._queue.append(item)

    def drain(self):
        with self._lock:
            items = list(self._queue)
            self._queue.clear()
            return items
