"""Explicitly started, deterministic software-only input source.

A script is data, never executable Python. Ordinary construction/open produces
neutral, paused input. End, freeze and disconnect are health transitions that
must stop motion; restart always changes input epoch.
"""

from __future__ import annotations

import json
from time import monotonic_ns
from uuid import uuid4

from gca.domain.manual import host_clock_id

from ..config import bounded_text, finite
from ..model import InputError, InputFrame, InputHealth, InputIdentity


def load_script(path):
    try:
        data = json.loads(bounded_text(path, 1_000_000))
    except RecursionError:
        raise ValueError("Script nesting exceeds limit") from None
    if not isinstance(data, dict) or set(data) - {"schema_version", "events", "duration_s"}:
        raise ValueError("invalid script schema")
    if data.get("schema_version") != 1:
        raise ValueError("unsupported script version")
    finite(data.get("duration_s"), "script duration", 0.001, 3600)
    events = data.get("events")
    if not isinstance(events, list) or len(events) > 20_000:
        raise ValueError("invalid script events")
    previous = -1
    for event in events:
        if not isinstance(event, dict) or set(event) - {"at_s", "axes", "buttons", "action"}:
            raise ValueError("invalid script event")
        finite(event.get("at_s"), "script event time", 0, data["duration_s"])
        if event["at_s"] < previous:
            raise ValueError("script events must be ordered")
        previous = event["at_s"]
        if event.get("action", "") not in ("", "freeze", "disconnect"):
            raise ValueError("invalid script action")
        for field in ("axes", "buttons"):
            values = event.get(field, {})
            if not isinstance(values, dict) or len(values) > 256:
                raise ValueError("invalid scripted control values")
            for key, value in values.items():
                if not isinstance(key, str) or not key.isdigit() or int(key) > 255:
                    raise ValueError("invalid scripted control index")
                if type(value) is not int or not (
                    -32768 <= value <= 32767 if field == "axes" else value in (0, 1)
                ):
                    raise ValueError("invalid scripted value")
    return data


class ScriptedInput:
    def __init__(self, profile, *, script_file=None, clock=monotonic_ns, backend="simulation"):
        if profile.source != "scripted" or backend != "simulation":
            raise InputError("SCRIPT_REQUIRES_SIMULATION")
        self.profile, self.clock = profile, clock
        self.script = (
            load_script(script_file or profile.script_file)
            if (script_file or profile.script_file)
            else {"schema_version": 1, "duration_s": 1.0, "events": []}
        )
        self.identity = InputIdentity(
            "scripted",
            "SYNTHETIC INPUT",
            0,
            0,
            0,
            1,
            tuple(range(8)),
            tuple(range(16)),
            device_mode="synthetic",
        )
        self._open = False
        self._connected = False
        self._running = False
        self._frozen = False
        self._reason = "NOT_OPEN"
        self._heartbeat = 0
        self._reset()
        from dataclasses import replace

        self.identity = replace(
            self.identity,
            axis_codes=tuple(range(len(self._axes))),
            button_codes=tuple(range(len(self._buttons))),
        )

    def _reset(self):
        self._epoch = uuid4().hex
        self._source = "scripted:" + self.profile.profile_id
        self._event_seq = 0
        self._observation_seq = 0
        self._raw_ns = 0
        self._index = 0
        m = self.profile.mapping
        axis_count = (
            max([7, *(x.index for x in (m.lx, m.ly, m.lt) if x is not None), *m.reserved_axes]) + 1
        )
        button_count = (
            max([15, m.b_button if m.b_button is not None else 0, *m.reserved_buttons]) + 1
        )
        self._axes = [0] * axis_count
        self._buttons = [0] * button_count
        if m.complete:
            self._axes[m.lx.index], self._axes[m.ly.index] = int(m.lx.center), int(m.ly.center)
            self._axes[m.lt.index] = int(m.lt.released)

    def open(self):
        self._reset()
        self._open, self._connected, self._running = True, True, False
        self._reason, self._heartbeat = "SCRIPT_PAUSED", self.clock()
        return self.identity

    def start(self):
        if not self._open:
            raise InputError("NOT_OPEN")
        self._reset()
        self._running, self._connected, self._frozen = True, True, False
        self._started_ns = self.clock()
        self._heartbeat, self._reason = self._started_ns, ""

    def stop(self):
        self._running, self._reason = False, "SCRIPT_PAUSED"

    def freeze(self, frozen=True):
        self._frozen = bool(frozen)
        if not frozen:
            # No resumption of an old active episode after a frozen reader.
            self.stop()

    def disconnect(self):
        self._connected, self._running, self._reason = False, False, "DISCONNECTED"

    def reconnect(self):
        return self.open()

    def _frame(self, now, axes=(), buttons=()):
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
            True,
            True,
            identity=self.identity,
            source_kind="SCRIPT_SAMPLE",
        )

    def poll(self, timeout_s=0.0):
        if not self._open or not self._connected or self._frozen:
            return ()
        now = self.clock()
        self._heartbeat = now
        if not self._running:
            # Paused/finished states are unhealthy for motion, even though the
            # reader remains alive. No stale active axes are emitted.
            return ()
        if now - self._started_ns >= int(self.script["duration_s"] * 1e9):
            self._running, self._reason = False, "SCRIPT_FINISHED"
            return ()
        frames = []
        budget = self.profile.max_poll_events
        while self._index < len(self.script["events"]):
            event = self.script["events"][self._index]
            if int(event["at_s"] * 1e9) > now - self._started_ns:
                break
            if len(frames) >= budget:
                self._running, self._reason = False, "SCRIPT_EVENT_BUDGET_EXCEEDED"
                return ()
            self._index += 1
            if event.get("action") == "freeze":
                self.freeze()
                break
            if event.get("action") == "disconnect":
                self.disconnect()
                break
            changed = []
            for field, values in (("axes", self._axes), ("buttons", self._buttons)):
                indices = []
                for key, value in event.get(field, {}).items():
                    index = int(key)
                    if index >= len(values):
                        self._running, self._reason = False, "SCRIPT_CAPABILITY_MISMATCH"
                        return ()
                    if values[index] != value:
                        values[index] = value
                        indices.append(index)
                changed.append(tuple(indices))
            self._event_seq += 1
            self._raw_ns = self._started_ns + int(event["at_s"] * 1e9)
            frames.append(self._frame(now, *changed))
        if self._frozen or not self._connected:
            return tuple(frames)
        # A script is an actual clocked synthetic sample. This deliberately
        # differs from physical events, and is rejected by hardware receivers.
        if not frames:
            self._event_seq += 1
            self._raw_ns = now
            frames.append(self._frame(now))
        return tuple(frames)

    def health(self):
        return InputHealth(
            self._connected,
            self._open,
            self._heartbeat,
            self._source,
            self._epoch,
            self._reason,
            "SCRIPTED_ONLY",
            identity=self.identity,
        )

    def close(self):
        self._open, self._connected, self._running = False, False, False
        self._reason = "CLOSED"
