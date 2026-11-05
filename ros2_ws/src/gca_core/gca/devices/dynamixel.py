"""Single-owner DYNAMIXEL X-series velocity adapter using official SDK 4.1.0.

The selected model's control table must match this adapter. A model number in
legacy filenames is not verification. ``stop`` acknowledges a zero setpoint;
only a later measured velocity/verified disable can prove physical stopping.
"""

from __future__ import annotations

import math
import os
import time
import uuid
from collections.abc import Callable

from gca.domain.models import DeviceIdentity, ServoSample

from .common import DeviceError, DeviceLock, DynamixelConfig
from .sdk_support import dynamixel_factories


class DynamixelServo:
    """Hardware lifecycle; every method belongs to the servo bus I/O owner."""

    # ROBOTIS e-Manual xm430-w350 control table: model 1020.
    # Broader X-series address similarity is not a supported-model guarantee.
    SUPPORTED_MODELS = {1020: "XM430-W350"}
    MIN_WATCHDOG_FIRMWARE = 38
    MODEL = 0
    FIRMWARE = 6
    MODE = 11
    VELOCITY_LIMIT = 44
    TORQUE = 64
    WATCHDOG = 98
    GOAL_VELOCITY = 104
    PRESENT_VELOCITY = 128

    def __init__(
        self,
        config: DynamixelConfig,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        port_factory=None,
        packet_factory=None,
    ):
        self.config, self.clock = config, clock
        self._port_factory, self._packet_factory = port_factory, packet_factory
        self.port = self.packet = None
        self.identity = None
        self.configured = False
        self.enabled = False
        self._lock = DeviceLock(os.path.realpath(config.port))

    def _deadline(self, requested: int | None = None) -> int:
        limit = self.clock() + int(self.config.transaction_timeout_s * 1e9)
        return min(limit, requested) if requested is not None else limit

    def _check(self, deadline_ns: int) -> None:
        if self.port is None or self.packet is None:
            raise DeviceError("DEVICE_CLOSED", "DYNAMIXEL is not open")
        if self.clock() >= deadline_ns:
            raise DeviceError("DEVICE_TIMEOUT", "DYNAMIXEL deadline expired")
        self.port.deadline_ns = deadline_ns
        if getattr(self.port, "ser", None) is not None:
            self.port.ser.write_timeout = max(0.001, (deadline_ns - self.clock()) / 1e9)

    def _result(self, result: int, error: int) -> None:
        if result != 0:
            raise DeviceError("SDK_ERROR", f"DYNAMIXEL communication result={result}")
        if error:
            raise DeviceError("DEVICE_ERROR", f"DYNAMIXEL status error={error}")

    def _read(self, address: int, width: int, deadline_ns: int, *, signed=False) -> int:
        self._check(deadline_ns)
        try:
            data, result, error = self.packet.readTxRx(
                self.port, self.config.device_id, address, width
            )
            self._result(result, error)
            if len(data) != width or any(not isinstance(v, int) or not 0 <= v <= 255 for v in data):
                raise DeviceError("BAD_FRAME", "DYNAMIXEL parameter length mismatch")
            if self.clock() >= deadline_ns:
                raise DeviceError("DEVICE_TIMEOUT", "late DYNAMIXEL read")
            return int.from_bytes(bytes(data), "little", signed=signed)
        except DeviceError:
            raise
        except (OSError, ValueError, IndexError, TypeError) as exc:
            raise DeviceError("SDK_ERROR", str(exc)) from exc

    def _write(self, address: int, width: int, value: int, deadline_ns: int, *, signed=False):
        self._check(deadline_ns)
        try:
            encoded = list(value.to_bytes(width, "little", signed=signed))
            result, error = self.packet.writeTxRx(
                self.port, self.config.device_id, address, width, encoded
            )
            self._result(result, error)
            if self.clock() >= deadline_ns:
                raise DeviceError("DEVICE_TIMEOUT", "late DYNAMIXEL write")
        except DeviceError:
            raise
        except (OSError, ValueError, OverflowError, IndexError, TypeError) as exc:
            raise DeviceError("SDK_ERROR", str(exc)) from exc

    def _write_verified(self, address: int, width: int, value: int, *, signed=False):
        deadline = self._deadline()
        self._write(address, width, value, deadline, signed=signed)
        if self._read(address, width, deadline, signed=signed) != value:
            raise DeviceError("CONFIG_READBACK_MISMATCH", f"register {address} did not apply")

    def open(self) -> DeviceIdentity:
        """Connect and identify; never configure mode, set zero or enable torque."""
        if not self.config.verified:
            raise DeviceError("HARDWARE_UNVERIFIED", "confirmed DYNAMIXEL configuration required")
        if self.config.model_number not in self.SUPPORTED_MODELS:
            raise DeviceError(
                "MODEL_UNSUPPORTED", "only verified XM430-W350 model 1020 is implemented"
            )
        if self.port is not None:
            raise DeviceError("ALREADY_OPEN", "close before opening again")
        if self._port_factory is None or self._packet_factory is None:
            self._port_factory, self._packet_factory = dynamixel_factories(self.clock)
        self._lock.acquire()
        try:
            self.port = self._port_factory(self.config.port)
            self.packet = self._packet_factory()
            # setBaudRate opens the pinned SDK port; opening twice loses bounds.
            if not self.port.setBaudRate(self.config.baudrate):
                raise DeviceError("CONNECT_FAILED", "unsupported baud rate/open failed")
            if getattr(self.port, "ser", None) is not None:
                self.port.ser.timeout = 0
                self.port.ser.write_timeout = self.config.transaction_timeout_s
            model = self._read(self.MODEL, 2, self._deadline())
            if model != self.config.model_number:
                raise DeviceError(
                    "IDENTITY_MISMATCH", f"expected {self.config.model_number}, got {model}"
                )
            firmware = self._read(self.FIRMWARE, 1, self._deadline())
            if firmware < self.MIN_WATCHDOG_FIRMWARE:
                raise DeviceError("CAPABILITY_UNAVAILABLE", "firmware v38+ Bus Watchdog required")
            self.identity = DeviceIdentity(
                f"DYNAMIXEL:{model}",
                str(uuid.uuid4()),
                str(firmware),
                str(self.config.device_id),
                ("velocity", "measured_velocity", "torque_disable"),
            )
            return self.identity
        except Exception as exc:
            if self.port is not None and getattr(self.port, "is_open", True):
                try:
                    self.port.closePort()
                except (OSError, AttributeError):
                    self.enabled = False
            self.port = self.packet = None
            self._lock.release()
            if isinstance(exc, DeviceError):
                raise
            raise DeviceError("CONNECT_FAILED", str(exc)) from exc

    def configure(self) -> None:
        """Disabled configuration transaction; enable only after every readback."""
        self.configured = False
        self.enabled = False
        self._write_verified(self.TORQUE, 1, 0)
        self._write_verified(self.WATCHDOG, 1, 0)  # Explicitly clear a latched watchdog error.
        self._write_verified(self.MODE, 1, 1)
        self._write_verified(self.VELOCITY_LIMIT, 4, self.config.max_velocity_raw)
        self._write_verified(self.WATCHDOG, 1, self.config.watchdog_raw)
        self._write_verified(self.GOAL_VELOCITY, 4, 0, signed=True)
        self.configured = True

    def enable(self, enabled: bool) -> None:
        """Explicit ARM/disarm operation. Zero readback precedes torque enable."""
        if enabled:
            if not self.configured:
                raise DeviceError("NOT_CONFIGURED", "cannot enable before verified configuration")
            self._write_verified(self.GOAL_VELOCITY, 4, 0, signed=True)
        self._write_verified(self.TORQUE, 1, int(enabled))
        self.enabled = enabled

    def write_velocity(self, velocity_mm_s: float, deadline_ns: int) -> float:
        """Write bounded signed velocity and return the quantized setpoint mm/s."""
        if not self.configured or not self.enabled:
            raise DeviceError("NOT_ARMED", "velocity requires configured enabled servo")
        if not math.isfinite(velocity_mm_s):
            raise DeviceError("INVALID_ARGUMENT", "non-finite velocity")
        c = self.config
        raw = round(
            c.direction_sign * 60 * velocity_mm_s / (c.mm_per_revolution * c.velocity_unit_rpm)
        )
        if abs(raw) > c.max_velocity_raw:
            raise DeviceError("LIMIT_EXCEEDED", "velocity exceeds confirmed local limit")
        self._write(self.GOAL_VELOCITY, 4, raw, self._deadline(deadline_ns), signed=True)
        return c.direction_sign * raw * c.velocity_unit_rpm * c.mm_per_revolution / 60

    def read_state(self, deadline_ns: int) -> ServoSample:
        """Read measured signed velocity; no cached value can acquire a new age."""
        deadline = self._deadline(deadline_ns)
        raw = self._read(self.PRESENT_VELOCITY, 4, deadline, signed=True)
        enabled = bool(self._read(self.TORQUE, 1, deadline))
        c = self.config
        velocity = c.direction_sign * raw * c.velocity_unit_rpm * c.mm_per_revolution / 60
        return ServoSample(velocity, enabled, self.clock(), self.identity.device_epoch)

    def stop(self, deadline_ns: int) -> None:
        """Always permit the restricted zero command while connected.

        This does not assert that the mechanism has stopped. The owner must
        obtain fresh measured stopping evidence or report UNKNOWN.
        """
        self._write(self.GOAL_VELOCITY, 4, 0, self._deadline(deadline_ns), signed=True)

    def close(self) -> None:
        """Request zero/disable then release. Raise on uncertain exit, after close."""
        if self.port is None:
            return
        failure = None
        try:
            try:
                self.stop(self._deadline())
            except DeviceError as exc:
                failure = exc
            # A failed zero command must not skip the independently verified
            # torque-disable path. Each attempt has its own bounded budget.
            try:
                self.enable(False)
            except DeviceError as exc:
                if failure is None:
                    failure = exc
        finally:
            try:
                self.port.closePort()
            finally:
                self.port = self.packet = None
                self.enabled = self.configured = False
                self._lock.release()
        if failure is not None:
            raise failure
