"""Read-only Linux joydev reader with bounded work and explicit epochs.

INIT records are baselines, never presses. Linux joydev timestamps are u32
jiffies-derived milliseconds, not Unix time. Initial read-time anchors are
conservative by a configured uncertainty allowance. Later queued timestamps
retain their age; repeated INIT (including joydev overflow recovery) revokes
the epoch. Reading a cached state does not establish radio freshness.
"""

from __future__ import annotations

import array
import errno
import fcntl
import os
import platform
import select
import struct
from pathlib import Path
from time import monotonic_ns
from uuid import uuid4

from gca.domain.manual import host_clock_id

from ..model import InputError, InputFrame, InputHealth, InputIdentity

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
_EVENT = struct.Struct("=IhBB")


def _text(path, default=""):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def _hex(path):
    value = _text(path)
    return int(value, 16) if value else 0


def inspect_device(
    path,
    *,
    opener=os.open,
    ioctl=fcntl.ioctl,
    closer=os.close,
    sysfs_root="/sys/class/input",
    device_mode="",
):
    """Query capabilities without reading events, changing calibration or writing."""
    path = str(Path(path).resolve())
    node = Path(path).name
    if not node.startswith("js") or not node[2:].isdigit():
        raise InputError("NOT_JOYDEV")
    fd = opener(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        axes, buttons = bytearray(1), bytearray(1)
        ioctl(fd, 0x80016A11, axes, True)
        ioctl(fd, 0x80016A12, buttons, True)
        if not axes[0] or not buttons[0]:
            raise InputError("EMPTY_CAPABILITIES")
        name = bytearray(256)
        ioctl(fd, 0x81006A13, name, True)
        axis_map, button_map = bytearray(64), array.array("H", [0] * 512)
        ioctl(fd, 0x80406A32, axis_map, True)
        ioctl(fd, 0x84006A34, button_map, True)
        base = Path(sysfs_root) / node / "device"
        driver_link = base.resolve()
        driver = ""
        for parent in (driver_link, *driver_link.parents):
            if (parent / "driver").is_symlink():
                driver = (parent / "driver").resolve().name
                break
        return InputIdentity(
            path,
            bytes(name).split(b"\0", 1)[0].decode("utf8", "replace"),
            _hex(base / "id/vendor"),
            _hex(base / "id/product"),
            _hex(base / "id/bustype"),
            _hex(base / "id/version"),
            tuple(axis_map[: axes[0]]),
            tuple(button_map[: buttons[0]]),
            _text(base / "phys"),
            _text(base / "uniq"),
            driver,
            platform.release(),
            device_mode,
        )
    finally:
        closer(fd)


def discover(
    selector, *, sysfs_root="/sys/class/input", dev_root="/dev/input", inspect=inspect_device
):
    """Resolve by complete configured predicates, refusing ambiguous matches."""
    paths = (
        [selector.path]
        if selector.path
        else [str(Path(dev_root) / p.name) for p in sorted(Path(sysfs_root).glob("js*"))]
    )
    matches, failures = [], []
    for path in paths:
        try:
            identity = inspect(path, device_mode=selector.device_mode)
        except (OSError, ValueError, InputError) as exc:
            failures.append(str(exc))
            continue
        if all(
            getattr(selector, name) is None or getattr(selector, name) == getattr(identity, name)
            for name in ("vendor", "product", "name", "axis_codes", "button_codes")
        ):
            matches.append(identity)
    if len(matches) != 1:
        raise InputError(
            "INPUT_AMBIGUOUS" if len(matches) > 1 else "INPUT_NOT_FOUND",
            "; ".join(failures) or "No unique matching joydev device",
        )
    return matches[0]


class NativeJoydev:
    def __init__(
        self,
        profile,
        *,
        clock=monotonic_ns,
        discovery=None,
        opener=os.open,
        reader=os.read,
        ioctl=fcntl.ioctl,
        poll_factory=select.poll,
        closer=os.close,
    ):
        if profile.source != "joydev":
            raise ValueError("joydev profile required")
        self.profile, self.clock = profile, clock
        self.discovery = discovery or discover
        self.opener, self.reader, self.ioctl = opener, reader, ioctl
        self.poll_factory, self.closer = poll_factory, closer
        self.fd = None
        self.identity = None
        self._connected = False
        self._reason = "NOT_OPEN"
        self._heartbeat = 0
        self._epoch = ""
        self._source = ""
        self._poller = None
        self._reset()

    def _reset(self):
        self._epoch = uuid4().hex
        self._event_seq = 0
        self._observation_seq = 0
        self._raw_ns = 0
        self._anchor_ms = None
        self._anchor_ns = 0
        self._last_ms = None
        self._initialized = False
        self._axes = [None] * (len(self.identity.axis_codes) if self.identity else 0)
        self._buttons = [None] * (len(self.identity.button_codes) if self.identity else 0)
        self._fragment = b""
        self._fragment_started_ns = 0

    def open(self):
        if self.fd is not None:
            raise InputError("ALREADY_OPEN")
        self.identity = self.discovery(self.profile.device)
        # Reinspect the opened FD rather than trusting a device number discovered
        # before a hotplug. Full identity inspection happens again on every open.
        self.fd = self.opener(self.identity.path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        try:
            axes, buttons = bytearray(1), bytearray(1)
            self.ioctl(self.fd, 0x80016A11, axes, True)
            self.ioctl(self.fd, 0x80016A12, buttons, True)
            amap, bmap = bytearray(64), array.array("H", [0] * 512)
            self.ioctl(self.fd, 0x80406A32, amap, True)
            self.ioctl(self.fd, 0x84006A34, bmap, True)
            name = bytearray(256)
            self.ioctl(self.fd, 0x81006A13, name, True)
            if (
                tuple(amap[: axes[0]]) != self.identity.axis_codes
                or tuple(bmap[: buttons[0]]) != self.identity.button_codes
                or bytes(name).split(b"\0", 1)[0].decode("utf8", "replace") != self.identity.name
            ):
                raise InputError("IDENTITY_CHANGED_DURING_OPEN")
            self._poller = self.poll_factory()
            self._poller.register(self.fd, select.POLLIN | select.POLLHUP | select.POLLERR)
            self._reset()
            self._source = self.identity.fingerprint[:32]
            self._connected, self._reason = True, "BASELINE_PENDING"
            self._heartbeat = self.clock()
            return self.identity
        except BaseException:
            self.close()
            raise

    def _frame(self, now, *, axes=(), buttons=(), resync=False, kernel_ms=None):
        self._observation_seq += 1
        return InputFrame(
            self._source,
            self._epoch,
            self._event_seq,
            self._raw_ns,
            self._observation_seq,
            now,
            host_clock_id(),
            tuple(self._axes),
            tuple(self._buttons),
            axes,
            buttons,
            self._initialized,
            False,
            resync,
            self.identity,
            kernel_ms,
        )

    def _fail(self, reason):
        self._connected, self._initialized, self._reason = False, False, reason

    def poll(self, timeout_s=None):
        if self.fd is None or not self._connected:
            return ()
        timeout = self.profile.poll_timeout_s if timeout_s is None else timeout_s
        if type(timeout) not in (int, float) or not 0 <= timeout <= self.profile.poll_timeout_s:
            raise ValueError("poll timeout exceeds bounded policy")
        start = self.clock()
        if start - self._heartbeat >= int(self.profile.reader_timeout_s * 1e9):
            self._fail("READER_STALLED_REOPEN_REQUIRED")
            return ()
        try:
            ready = self._poller.poll(int(timeout * 1000))
            now = self.clock()
            if now - start >= int(self.profile.reader_timeout_s * 1e9):
                self._fail("READER_STALLED_REOPEN_REQUIRED")
                return ()
            self._heartbeat = now
            if any(
                flags & (select.POLLHUP | select.POLLERR | select.POLLNVAL) for _, flags in ready
            ):
                self._fail("DISCONNECTED")
                return ()
            frames = []
            if ready:
                try:
                    chunk = self.reader(
                        self.fd, min(self.profile.max_poll_bytes, self.profile.max_poll_events * 8)
                    )
                except BlockingIOError:
                    chunk = None
                now = self.clock()
                self._heartbeat = now
                if now - start >= int(self.profile.reader_timeout_s * 1e9):
                    self._fail("READER_STALLED_REOPEN_REQUIRED")
                    return ()
                if chunk == b"":
                    self._fail("DISCONNECTED")
                    return ()
                if chunk:
                    data_start = self._fragment_started_ns if self._fragment else start
                    data = self._fragment + chunk
                    self._fragment = data[len(data) // 8 * 8 :]
                    self._fragment_started_ns = start if len(data) >= 8 else data_start
                    if len(data) // 8 > self.profile.max_poll_events:
                        self._fail("EVENT_BUDGET_EXCEEDED")
                        return ()
                    for offset in range(0, len(data) // 8 * 8, 8):
                        stamp, value, kind, index = _EVENT.unpack_from(data, offset)
                        init, kind = bool(kind & JS_EVENT_INIT), kind & ~JS_EVENT_INIT
                        if kind not in (JS_EVENT_AXIS, JS_EVENT_BUTTON):
                            self._fail("INVALID_EVENT_TYPE")
                            return ()
                        values = self._axes if kind == JS_EVENT_AXIS else self._buttons
                        if index >= len(values) or kind == JS_EVENT_BUTTON and value not in (0, 1):
                            self._fail("INVALID_EVENT_VALUE")
                            return ()
                        resync = False
                        if init and self._initialized:
                            self._reset()
                            # reset also clears baseline and both sequence counters
                            values = self._axes if kind == JS_EVENT_AXIS else self._buttons
                            resync = True
                        if self._anchor_ms is None:
                            if not init:
                                self._fail("MISSING_INIT_BASELINE")
                                return ()
                            self._anchor_ms = stamp
                            self._anchor_ns = max(
                                0, data_start - int(self.profile.timestamp_uncertainty_s * 1e9)
                            )
                        delta = ((stamp - self._anchor_ms + 2**31) % 2**32) - 2**31
                        translated = self._anchor_ns + delta * 1_000_000
                        if (
                            abs(delta) > self.profile.max_epoch_s * 1000
                            or translated > now
                            or translated < 0
                            or self._last_ms is not None
                            and ((stamp - self._last_ms + 2**31) % 2**32) - 2**31 < 0
                        ):
                            self._fail("EVENT_TIME_DISCONTINUITY")
                            return ()
                        self._last_ms = stamp
                        changed = values[index] != value
                        values[index] = value
                        if not init:
                            self._event_seq += 1
                            self._raw_ns = translated
                        self._initialized = all(
                            v is not None for v in (*self._axes, *self._buttons)
                        )
                        self._reason = "" if self._initialized else "BASELINE_PENDING"
                        frames.append(
                            self._frame(
                                now,
                                axes=(index,)
                                if changed and not init and kind == JS_EVENT_AXIS
                                else (),
                                buttons=(index,)
                                if changed and not init and kind == JS_EVENT_BUTTON
                                else (),
                                resync=resync,
                                kernel_ms=stamp,
                            )
                        )
            # Even without events this is a real reader observation, never a new
            # physical event. FSM consumes it only in qualified-state mode.
            return tuple(frames) if frames else (self._frame(now),)
        except OSError as exc:
            self._heartbeat = self.clock()
            self._fail(
                "DISCONNECTED"
                if exc.errno in (errno.ENODEV, errno.EIO, errno.EBADF)
                else "READ_ERROR"
            )
            return ()

    def health(self):
        return InputHealth(
            self._connected,
            self._initialized,
            self._heartbeat,
            self._source,
            self._epoch,
            self._reason,
            identity=self.identity,
        )

    def close(self):
        if self.fd is not None:
            try:
                if self._poller is not None:
                    self._poller.unregister(self.fd)
            finally:
                self.closer(self.fd)
                self.fd = None
        self._fail("CLOSED")
