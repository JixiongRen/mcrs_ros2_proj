"""Configured encoder serial adapter: single outstanding request and no writes.

A function-03 response carries no transaction identifier. After an incomplete
or invalid transaction, this adapter requires an explicit reconnect; open()
then drains for the configured quiet interval before issuing any request. The
quiet interval must exceed the device's verified maximum delayed response.
"""

from __future__ import annotations

import os
import time
import uuid
from collections.abc import Callable

from gca.domain.models import DeviceIdentity, EncoderSample
from gca.protocols.encoder import CountFormat, ProtocolError, ResponseParser, read_request

from .common import DeviceError, DeviceLock, EncoderConfig


class SerialEncoder:
    """Raw count acquisition, exclusively called by its serial I/O owner."""

    def __init__(
        self,
        config: EncoderConfig,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        serial_factory=None,
        sleeper: Callable[[float], None] = time.sleep,
    ):
        self.config, self.clock = config, clock
        self.serial_factory, self.sleeper = serial_factory, sleeper
        self.serial = None
        self.identity = None
        self.seq = 0
        self._lock = DeviceLock(os.path.realpath(config.port))
        self._synchronized = False
        self._format = CountFormat(config.bits, config.signed, config.byteorder, config.modulus)

    def open(self) -> DeviceIdentity:
        """Connect without setting zero, baud registers, direction or movement."""
        if not self.config.verified:
            raise DeviceError("HARDWARE_UNVERIFIED", "encoder protocol/electrical record required")
        if self.serial is not None:
            raise DeviceError("ALREADY_OPEN", "close encoder before reconnect")
        if self.serial_factory is None:
            try:
                from serial import Serial
            except ImportError as exc:
                raise DeviceError("DEPENDENCY_MISSING", "install pyserial==3.5") from exc
            self.serial_factory = Serial
        c = self.config
        self._lock.acquire()
        try:
            self.serial = self.serial_factory(
                port=c.port, baudrate=c.baudrate, timeout=0, write_timeout=c.transaction_timeout_s
            )
            self.serial.reset_input_buffer()
            # A bounded quiet window, restarted by input, isolates late old replies.
            end = self.clock() + int((c.quiet_s + c.transaction_timeout_s) * 1e9)
            quiet_until = self.clock() + int(c.quiet_s * 1e9)
            while self.clock() < quiet_until:
                if self.clock() >= end:
                    raise DeviceError("RESYNC_FAILED", "encoder never reached quiet interval")
                waiting = self.serial.in_waiting
                if waiting:
                    if waiting > 260:
                        raise DeviceError("BAD_FRAME", "encoder input exceeds bound")
                    self.serial.read(waiting)
                    quiet_until = self.clock() + int(c.quiet_s * 1e9)
                self.sleeper(min(0.001, c.quiet_s))
            self.seq = 0
            self._synchronized = True
            self.identity = DeviceIdentity(
                "configured-modbus-encoder",
                str(uuid.uuid4()),
                device_id=f"{c.port}:{c.address}",
                capabilities=("raw_count", "configured_identity_only"),
            )
            return self.identity
        except Exception as exc:
            if self.serial is not None:
                self.serial.close()
            self.serial = None
            self._lock.release()
            if isinstance(exc, DeviceError):
                raise
            raise DeviceError("CONNECT_FAILED", str(exc)) from exc

    def read(self, deadline_ns: int) -> EncoderSample:
        """Return one new validated count or an explicit failure, never cached data."""
        if self.serial is None:
            raise DeviceError("DEVICE_CLOSED", "encoder is closed")
        if not self._synchronized:
            raise DeviceError("RECONNECT_REQUIRED", "failed transaction requires bounded resync")
        c = self.config
        started = self.clock()
        deadline_ns = min(deadline_ns, started + int(c.transaction_timeout_s * 1e9))
        if deadline_ns <= started:
            raise DeviceError("DEVICE_TIMEOUT", "encoder request already expired")
        parser = ResponseParser(c.address, c.bits // 8)
        try:
            if self.serial.in_waiting:
                raise DeviceError("STALE_RESPONSE", "unsolicited bytes before encoder request")
            request = read_request(c.address, c.register, c.bits // 16)
            self.serial.write_timeout = (deadline_ns - self.clock()) / 1e9
            if self.serial.write(request) != len(request):
                raise DeviceError("DEVICE_IO_ERROR", "incomplete encoder request write")
            while self.clock() < deadline_ns:
                remaining = (deadline_ns - self.clock()) / 1e9
                self.serial.timeout = min(0.005, max(0, remaining))
                available = self.serial.in_waiting
                if available > 260:
                    raise DeviceError("BAD_FRAME", "encoder response exceeds bound")
                chunk = self.serial.read(max(1, available))
                payloads = parser.feed(chunk)
                if payloads:
                    if len(payloads) != 1 or parser.buffer or self.serial.in_waiting:
                        raise DeviceError("BAD_FRAME", "multiple/trailing encoder responses")
                    if self.clock() >= deadline_ns:
                        raise DeviceError("DEVICE_TIMEOUT", "late encoder measurement")
                    count = self._format.decode(payloads[0])
                    self.seq += 1
                    return EncoderSample(
                        count,
                        self.seq,
                        self.clock(),
                        self.identity.device_epoch,
                        request_started_ns=started,
                    )
                # Serial.read may be nonblocking in injected clients; avoid a busy loop.
                if not chunk:
                    self.sleeper(min(0.001, max(0, (deadline_ns - self.clock()) / 1e9)))
            raise DeviceError("DEVICE_TIMEOUT", "encoder response deadline expired")
        except Exception as exc:
            self._synchronized = False
            if isinstance(exc, DeviceError):
                raise
            if isinstance(exc, ProtocolError):
                raise DeviceError("BAD_FRAME", str(exc)) from exc
            raise DeviceError("DEVICE_IO_ERROR", str(exc)) from exc

    def close(self) -> None:
        """Release the serial port without changing hardware reference/configuration."""
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None
                self._lock.release()
        self._synchronized = False
