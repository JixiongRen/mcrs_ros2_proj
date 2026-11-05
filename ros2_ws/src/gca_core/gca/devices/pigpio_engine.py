"""Finite single-wave PUL/DIR adapter using the official pigpio client.

Completion counting is conditional: an exclusive, HIL-validated wave service
must report our wave running and subsequently idle, without abort or connection
failure. It is not shaft feedback. The adapter never estimates a partially
executed wave from elapsed time. Without those validated capabilities hardware
open is refused; the dependency's presence is not capability evidence.
"""

from __future__ import annotations

import math
import time
import uuid
from collections import OrderedDict
from collections.abc import Callable

from gca.domain.models import DeviceIdentity, PulseAbort, PulseCompletion, PulseSegment

from .common import DeviceError, DeviceLock, PigpioConfig
from .sdk_support import bounded_pigpio_client


class PigpioPulseEngine:
    """One finite segment in flight; all operations belong to the pulse owner."""

    def __init__(
        self,
        config: PigpioConfig,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        client_factory=None,
        pulse_factory=None,
    ):
        self.config, self.clock = config, clock
        self.client_factory = client_factory or bounded_pigpio_client
        self.pulse_factory = pulse_factory
        self.client = None
        self.identity = None
        self._active = None
        self._wave_id = None
        self._saw_running = False
        self._seen = OrderedDict()
        self._generation = -1
        self._lock = DeviceLock(f"pigpio-wave-port:{config.port}")

    def _call(self, method: str, *args):
        if self.client is None or not self.client.connected:
            raise DeviceError("DEVICE_CLOSED", "pigpio connection unavailable")
        try:
            result = getattr(self.client, method)(*args)
            if isinstance(result, int) and result < 0:
                raise DeviceError("PULSE_BACKEND_ERROR", f"{method} returned {result}")
            return result
        except DeviceError:
            raise
        except Exception as exc:
            raise DeviceError("PULSE_BACKEND_ERROR", f"{method}: {exc}") from exc

    def open(self) -> DeviceIdentity:
        """Explicitly connect/configure safe pins; constructor itself has no I/O."""
        c = self.config
        if not (c.verified and c.exclusive_wave_owner and c.waveform_hil_verified):
            raise DeviceError(
                "HARDWARE_UNVERIFIED", "verified exclusive finite-wave backend required"
            )
        if self.client is not None:
            raise DeviceError("ALREADY_OPEN", "close pulse connection before reopen")
        if self.pulse_factory is None:
            try:
                import pigpio
            except ImportError as exc:
                raise DeviceError("DEPENDENCY_MISSING", "install pigpio==1.78") from exc
            self.pulse_factory = pigpio.pulse
        self._lock.acquire()
        try:
            self.client = self.client_factory(c.host, c.port, c.socket_timeout_s)
            if not self.client.connected:
                raise DeviceError("CONNECT_FAILED", "pigpio daemon unavailable")
            # Process restarts do not inherit daemon-owned waves. Ownership must
            # already be exclusive; otherwise stopping them could affect others.
            self._call("wave_tx_stop")
            if self._call("wave_tx_busy"):
                raise DeviceError("STOP_UNKNOWN", "pigpio still transmitting after stop")
            self._call("set_mode", c.pulse_gpio, 1)
            self._call("set_mode", c.direction_gpio, 1)
            self._call("write", c.pulse_gpio, 0)
            self.identity = DeviceIdentity(
                "pigpio-finite-wave",
                str(uuid.uuid4()),
                device_id=f"{c.host}:{c.port}",
                capabilities=(
                    "finite_pulses",
                    "abort_count_unknown",
                    "conditional_full_wave_count",
                ),
            )
            self._seen.clear()
            self._generation = -1
            return self.identity
        except Exception as exc:
            self._disconnect()
            if isinstance(exc, DeviceError):
                raise
            raise DeviceError("CONNECT_FAILED", str(exc)) from exc

    def _disconnect(self) -> None:
        try:
            if self.client is not None:
                self.client.stop()
        finally:
            self.client = None
            self.identity = None
            self._lock.release()

    def _wave(self, segment: PulseSegment):
        c = self.config
        count = abs(segment.signed_pulses)
        total_us = round(segment.duration_s * 1e6)
        available = total_us - c.direction_setup_us - c.direction_hold_us
        if available < count * (c.high_us + c.low_us):
            raise DeviceError(
                "PULSE_TIMING_INVALID", "segment cannot satisfy high/low and DIR timing"
            )
        level = (
            c.positive_direction_level
            if segment.signed_pulses > 0
            else 1 - c.positive_direction_level
        )
        direction_mask, pulse_mask = 1 << c.direction_gpio, 1 << c.pulse_gpio
        initial_on = direction_mask if level else 0
        initial_off = pulse_mask | (direction_mask if not level else 0)
        waveform = [self.pulse_factory(initial_on, initial_off, c.direction_setup_us)]
        base, remainder = divmod(available, count)
        for index in range(count):
            period = base + (1 if index < remainder else 0)
            waveform.append(self.pulse_factory(pulse_mask, 0, c.high_us))
            waveform.append(self.pulse_factory(0, pulse_mask, period - c.high_us))
        waveform.append(self.pulse_factory(0, pulse_mask, c.direction_hold_us))
        return waveform

    def submit(self, segment: PulseSegment) -> None:
        """Accept a bounded wave; acceptance is never an executed pulse receipt."""
        if self.identity is None:
            raise DeviceError("DEVICE_CLOSED", "pulse engine is closed")
        if segment.device_epoch != self.identity.device_epoch:
            raise DeviceError("STALE_EPOCH", "pulse epoch mismatch")
        if segment.generation < self._generation:
            raise DeviceError("STALE_GENERATION", "old pulse generation")
        if segment.segment_id in self._seen:
            if self._seen[segment.segment_id] == segment:
                return
            raise DeviceError("ID_CONFLICT", "segment ID reused with different content")
        c = self.config
        if (
            not segment.segment_id
            or type(segment.expires_ns) is not int
            or segment.expires_ns < 0
            or not isinstance(segment.signed_pulses, int)
            or not 0 < abs(segment.signed_pulses) <= c.max_segment_pulses
            or not math.isfinite(segment.duration_s)
            or not 0 < segment.duration_s <= c.max_segment_s
        ):
            raise DeviceError("INVALID_ARGUMENT", "invalid finite pulse segment")
        if self._active is not None or self._call("wave_tx_busy"):
            raise DeviceError("PULSE_BUSY", "only one unaccounted wave may be active")
        waveform = self._wave(segment)
        wave_id = None
        send_attempted = False
        try:
            self._call("wave_add_new")
            self._call("wave_add_generic", waveform)
            wave_id = self._call("wave_create")
            # Preparation RPCs consume the original permit window. Recheck at
            # the final motion-trigger boundary; never restart TTL after RPCs.
            if segment.expires_ns and (
                self.clock() + round(segment.duration_s * 1e9) > segment.expires_ns
            ):
                raise DeviceError("PULSE_EXPIRED", "pulse preparation consumed execution permit")
            send_attempted = True
            self._call("wave_send_once", wave_id)
            self._active, self._wave_id = segment, wave_id
            self._saw_running = False
            self._generation = segment.generation
            self._seen[segment.segment_id] = segment
            while len(self._seen) > 256:
                self._seen.popitem(last=False)
            if self._call("wave_tx_busy"):
                if self._call("wave_tx_at") != wave_id:
                    raise DeviceError("PULSE_OWNERSHIP_LOST", "different wave is transmitting")
                self._saw_running = True
        except Exception:
            if wave_id is not None:
                if send_attempted:
                    # Sending may already have taken effect. Keep uncertain
                    # history until abort, never infer successful completion.
                    self._active, self._wave_id = segment, wave_id
                else:
                    # An expired preparation has never triggered motion.
                    # Free its daemon resource without pretending it ran.
                    self._call("wave_delete", wave_id)
            raise

    def poll(self) -> list[PulseCompletion]:
        """Settle a fully observed, uninterrupted finite wave exactly once."""
        if self._active is None:
            return []
        busy = self._call("wave_tx_busy")
        if busy:
            if self._call("wave_tx_at") != self._wave_id:
                raise DeviceError(
                    "PULSE_OWNERSHIP_LOST", "another wave replaced the active segment"
                )
            self._saw_running = True
            return []
        if not self._saw_running:
            raise DeviceError("ROTATION_ESTIMATE_INVALID", "wave completion was not observed")
        # Evidence includes known finite-wave contents, validated exclusive
        # service, matching epoch/wave identity and observed normal completion.
        # Idle by itself, send return code or elapsed duration is insufficient.
        segment = self._active
        self._call("wave_delete", self._wave_id)
        self._active = self._wave_id = None
        self._saw_running = False
        return [
            PulseCompletion(
                segment.segment_id,
                segment.signed_pulses,
                segment.generation,
                segment.device_epoch,
                segment.command_id,
            )
        ]

    def abort(self) -> PulseAbort:
        """Stop a wave. Partial count is unknown, even if abort ACK succeeds."""
        if self.client is None:
            return PulseAbort(0, True)
        active = self._active is not None
        self._call("wave_tx_stop")
        idle = not bool(self._call("wave_tx_busy"))
        if not idle:
            return PulseAbort(None if active else 0, False)
        if self._wave_id is not None:
            self._call("wave_delete", self._wave_id)
        self._active = self._wave_id = None
        self._saw_running = False
        return PulseAbort(None if active else 0, True)

    def is_idle(self) -> bool:
        if self.client is None:
            return self._active is None
        return self._active is None and not bool(self._call("wave_tx_busy"))

    def close(self) -> None:
        """Cancel before disconnecting; surface an uncertain stop to the owner."""
        if self.client is None:
            return
        try:
            stopped = self.abort()
            if not stopped.idle:
                raise DeviceError("STOP_UNKNOWN", "cannot confirm pulse engine idle")
            self._call("write", self.config.pulse_gpio, 0)
        finally:
            self._disconnect()
