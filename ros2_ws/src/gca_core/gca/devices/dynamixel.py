"""Single-owner native RPM drive and legacy linear adapter, official SDK 4.1.0.

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
from gca.servo_config import NativeDynamixelConfig

from .common import DeviceError, DeviceLock, DynamixelConfig
from .sdk_support import dynamixel_factories
from .servo_types import ServoShaftSample, VelocityWriteGuard


class DynamixelVelocityDrive:
    """Hardware lifecycle; every method belongs to the servo bus I/O owner."""

    # ROBOTIS e-Manual xm430-w350 control table: model 1020.
    # Broader X-series address similarity is not a supported-model guarantee.
    SUPPORTED_MODELS = {1020: "XM430-W350"}
    MIN_WATCHDOG_FIRMWARE = 38
    MODEL = 0
    FIRMWARE = 6
    DRIVE_MODE = 10
    MODE = 11
    PROTOCOL = 13
    VELOCITY_LIMIT = 44
    TORQUE = 64
    HARDWARE_ERROR = 70
    WATCHDOG = 98
    GOAL_VELOCITY = 104
    PROFILE_ACCELERATION = 108
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132

    def __init__(
        self,
        config: NativeDynamixelConfig,
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
        self._sample_seq = 0
        self._write_attempted = False
        self.configuration_readback = {}
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

    def _result(self, result: int, error: int, *, allow_alert=False) -> None:
        if result != 0:
            raise DeviceError("SDK_ERROR", f"DYNAMIXEL communication result={result}")
        if error and not (allow_alert and error == 0x80):
            raise DeviceError("DEVICE_ERROR", f"DYNAMIXEL status error={error}")

    def _read_bytes(self, address: int, width: int, deadline_ns: int, *, allow_alert=False):
        deadline_ns = self._deadline(deadline_ns)
        self._check(deadline_ns)
        try:
            data, result, error = self.packet.readTxRx(
                self.port, self.config.device_id, address, width
            )
            self._result(result, error, allow_alert=allow_alert)
            if len(data) != width or any(type(v) is not int or not 0 <= v <= 255 for v in data):
                raise DeviceError("BAD_FRAME", "DYNAMIXEL parameter length mismatch")
            if self.clock() >= deadline_ns:
                raise DeviceError("DEVICE_TIMEOUT", "late DYNAMIXEL read")
            return bytes(data)
        except DeviceError:
            raise
        except (OSError, ValueError, IndexError, TypeError) as exc:
            raise DeviceError("SDK_ERROR", str(exc)) from exc

    def _read(self, address: int, width: int, deadline_ns: int, *, signed=False) -> int:
        return int.from_bytes(
            self._read_bytes(address, width, deadline_ns), "little", signed=signed
        )

    def _write(
        self,
        address: int,
        width: int,
        value: int,
        deadline_ns: int,
        *,
        signed=False,
        write_guard: VelocityWriteGuard | None = None,
    ):
        deadline_ns = self._deadline(deadline_ns)
        self._check(deadline_ns)
        self._write_attempted = True
        self.port.write_guard = write_guard
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
        finally:
            self.port.write_guard = None

    def _write_verified(
        self, address: int, width: int, value: int, *, signed=False, deadline_ns=None
    ):
        deadline = self._deadline(deadline_ns)
        self._write(address, width, value, deadline, signed=signed)
        if self._read(address, width, deadline, signed=signed) != value:
            raise DeviceError("CONFIG_READBACK_MISMATCH", f"register {address} did not apply")

    def open_identified(self) -> DeviceIdentity:
        """Connect and identify; never configure mode, set zero or enable torque."""
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
            if self._read(self.PROTOCOL, 1, self._deadline()) != 2:
                raise DeviceError("IDENTITY_MISMATCH", "Protocol 2.0 control table required")
            self.identity = DeviceIdentity(
                f"DYNAMIXEL:{model}",
                str(uuid.uuid4()),
                str(firmware),
                str(self.config.device_id),
                ("servo_velocity", "servo_shaft_feedback", "torque_disable"),
            )
            self._sample_seq = 0
            self._write_attempted = False
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

    def configure_velocity_mode(self) -> None:
        """Disabled configuration transaction; enable only after every readback."""
        self.configured = False
        self.enabled = False
        self.configuration_readback = {}
        self._write_verified(self.TORQUE, 1, 0)
        if self._read(self.WATCHDOG, 1, self._deadline(), signed=True) == -1:
            raise DeviceError("WATCHDOG_LATCHED", "explicit disabled recovery required")
        if self._read(self.HARDWARE_ERROR, 1, self._deadline()):
            raise DeviceError("HARDWARE_FAULT", "hardware error requires explicit recovery")
        if self._read(self.DRIVE_MODE, 1, self._deadline()) != 0:
            raise DeviceError("CONFIG_READBACK_MISMATCH", "Drive Mode must be 0")
        previous_mode = self._read(self.MODE, 1, self._deadline())
        if previous_mode != 1:
            self._write_verified(self.MODE, 1, 1)
        if self._read(self.VELOCITY_LIMIT, 4, self._deadline()) != self.config.max_velocity_raw:
            self._write_verified(self.VELOCITY_LIMIT, 4, self.config.max_velocity_raw)
        if self.config.profile_acceleration_raw is not None:
            self._write_verified(self.PROFILE_ACCELERATION, 4, self.config.profile_acceleration_raw)
        self._write_verified(self.WATCHDOG, 1, self.config.watchdog_raw)
        self._write_verified(self.GOAL_VELOCITY, 4, 0, signed=True)
        self.configuration_readback = {
            "previous_mode": previous_mode,
            "mode": 1,
            "drive_mode": 0,
            "velocity_limit_raw": self.config.max_velocity_raw,
            "profile_acceleration_raw": self.config.profile_acceleration_raw,
            "watchdog_raw": self.config.watchdog_raw,
            "goal_velocity_raw": 0,
            "torque_enabled": False,
        }
        self.configured = True

    def read_configuration_registers(self):
        """Read current registers without applying configuration or stopping."""
        block = self._read_bytes(self.DRIVE_MODE, 102, self._deadline(), allow_alert=True)
        return {
            "drive_mode": block[0],
            "mode": block[1],
            "protocol": block[3],
            "velocity_limit_raw": int.from_bytes(block[34:38], "little"),
            "torque_enabled": bool(block[54]),
            "hardware_error": block[60],
            "watchdog_raw": int.from_bytes(block[88:89], "little", signed=True),
            "goal_velocity_raw": int.from_bytes(block[94:98], "little", signed=True),
            "profile_acceleration_raw": int.from_bytes(block[98:102], "little"),
        }

    def disconnect_readonly(self):
        """Release a genuinely read-only connection; never send zero or torque writes."""
        if self._write_attempted:
            raise DeviceError("NOT_READONLY", "use stop/close after any attempted register write")
        try:
            if self.port is not None:
                self.port.closePort()
        finally:
            self.port = self.packet = None
            self.configured = self.enabled = False
            self._lock.release()

    def set_torque(
        self,
        enabled: bool,
        deadline_ns: int | None = None,
        *,
        enable_guard: Callable[[], bool] | None = None,
    ) -> None:
        """Explicit ARM/disarm operation. Zero readback precedes torque enable."""
        if type(enabled) is not bool:
            raise DeviceError("INVALID_ARGUMENT", "torque state must be boolean")
        if enable_guard is not None and not callable(enable_guard):
            raise DeviceError("INVALID_ARGUMENT", "enable_guard must be callable")
        # ARM is a bounded management transaction, not a renewed motion target.
        deadline = (
            self.clock() + int(4 * self.config.transaction_timeout_s * 1e9)
            if deadline_ns is None
            else deadline_ns
        )
        if enabled:
            if not self.configured:
                raise DeviceError("NOT_CONFIGURED", "cannot enable before verified configuration")
            self._write(self.GOAL_VELOCITY, 4, 0, deadline, signed=True)
            block = self._read_bytes(self.DRIVE_MODE, 102, deadline)
            checks = (
                (0, 1, 0),
                (1, 1, 1),
                (34, 4, self.config.max_velocity_raw),
                (60, 1, 0),
                (88, 1, self.config.watchdog_raw),
                (94, 4, 0),
            )
            if self.config.profile_acceleration_raw is not None:
                checks += ((98, 4, self.config.profile_acceleration_raw),)
            if any(
                int.from_bytes(block[offset : offset + width], "little") != expected
                for offset, width, expected in checks
            ):
                self.configured = False
                raise DeviceError(
                    "CONFIG_READBACK_MISMATCH", "configuration/zero changed before ARM"
                )
            if enable_guard is not None and not enable_guard():
                raise DeviceError("STOP_SUPERSEDED", "ARM authorization revoked before enable")
        self._write_verified(self.TORQUE, 1, int(enabled), deadline_ns=deadline)
        self.enabled = enabled
        if enabled and enable_guard is not None and not enable_guard():
            # A stop arriving during the in-flight enable transaction is
            # handled independently, without extending the motion deadline.
            self._write_verified(self.TORQUE, 1, 0)
            self.enabled = False
            raise DeviceError("STOP_SUPERSEDED", "ARM authorization revoked during enable")

    def write_velocity_rpm(
        self, rpm: float, deadline_ns: int, *, write_guard: VelocityWriteGuard | None = None
    ) -> float:
        """Write native signed RPM and return the quantized accepted setpoint."""
        if not self.configured or not self.enabled:
            raise DeviceError("NOT_ARMED", "velocity requires configured enabled servo")
        if type(rpm) not in (int, float) or not math.isfinite(rpm):
            raise DeviceError("INVALID_ARGUMENT", "non-finite velocity")
        c = self.config
        raw = round(rpm / c.velocity_unit_rpm)
        if abs(rpm) > c.max_rpm or abs(raw) > c.max_velocity_raw:
            raise DeviceError("LIMIT_EXCEEDED", "velocity exceeds confirmed local limit")
        self._write(
            self.GOAL_VELOCITY,
            4,
            raw,
            self._deadline(deadline_ns),
            signed=True,
            write_guard=write_guard,
        )
        return raw * c.velocity_unit_rpm

    def read_shaft_state(self, deadline_ns: int) -> ServoShaftSample:
        """One contiguous RAM read; never renew a cached sample's original age."""
        started = self.clock()
        deadline = self._deadline(deadline_ns)
        block = self._read_bytes(self.TORQUE, 72, deadline, allow_alert=True)
        enabled = block[0]
        if enabled not in (0, 1):
            raise DeviceError("BAD_FRAME", "invalid torque feedback")
        hardware_error = block[6]
        watchdog = int.from_bytes(block[34:35], "little", signed=True)
        velocity = int.from_bytes(block[64:68], "little", signed=True)
        position = int.from_bytes(block[68:72], "little", signed=True)
        if watchdog < -1:
            raise DeviceError("BAD_FRAME", "invalid watchdog feedback")
        error = "HARDWARE_FAULT" if hardware_error else "WATCHDOG_LATCHED" if watchdog == -1 else ""
        self._sample_seq += 1
        return ServoShaftSample(
            position,
            velocity * self.config.velocity_unit_rpm,
            bool(enabled),
            self._sample_seq,
            self.clock(),
            self.identity.device_epoch,
            True,
            error,
            started,
            hardware_error,
            watchdog,
        )

    def recover_watchdog(self) -> None:
        """Explicit disabled recovery; no auto-enable or historical target restore."""
        if self._read(self.TORQUE, 1, self._deadline()) != 0:
            raise DeviceError("NOT_DISARMED", "disable torque before watchdog recovery")
        if self._read(self.HARDWARE_ERROR, 1, self._deadline()):
            raise DeviceError("HARDWARE_FAULT", "watchdog recovery cannot clear hardware errors")
        self.configured = False
        self._write_verified(self.WATCHDOG, 1, 0)
        self._write_verified(self.GOAL_VELOCITY, 4, 0, signed=True)
        self._write_verified(self.WATCHDOG, 1, self.config.watchdog_raw)

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
                self.set_torque(False)
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


class DynamixelServo:
    """Legacy mm/s adapter; all bus operations use the single native RPM drive."""

    def __init__(
        self,
        config: DynamixelConfig,
        *,
        clock=time.monotonic_ns,
        port_factory=None,
        packet_factory=None,
    ):
        self.config = config
        # The old profile has no acceleration setting: explicitly preserve its
        # existing register.  New ServoSettings requires a finite explicit one.
        native = NativeDynamixelConfig(
            config.port,
            config.baudrate,
            config.device_id,
            config.model_number,
            config.velocity_unit_rpm,
            config.max_velocity_raw,
            config.watchdog_raw,
            None,
            config.transaction_timeout_s,
        )
        self.drive = DynamixelVelocityDrive(
            native, clock=clock, port_factory=port_factory, packet_factory=packet_factory
        )

    @property
    def port(self):
        return self.drive.port

    @property
    def packet(self):
        return self.drive.packet

    @property
    def configured(self):
        return self.drive.configured

    @property
    def enabled(self):
        return self.drive.enabled

    @property
    def identity(self):
        return self.drive.identity

    def open(self):
        if not self.config.verified:
            raise DeviceError("HARDWARE_UNVERIFIED", "confirmed DYNAMIXEL configuration required")
        return self.drive.open_identified()

    def configure(self):
        self.drive.configure_velocity_mode()

    def enable(self, enabled):
        self.drive.set_torque(enabled)

    def write_velocity(self, velocity_mm_s, deadline_ns):
        c = self.config
        rpm = c.direction_sign * 60 * velocity_mm_s / c.mm_per_revolution
        accepted = self.drive.write_velocity_rpm(rpm, deadline_ns)
        return c.direction_sign * accepted * c.mm_per_revolution / 60

    def read_state(self, deadline_ns):
        sample = self.drive.read_shaft_state(deadline_ns)
        c = self.config
        return ServoSample(
            c.direction_sign * sample.velocity_rpm * c.mm_per_revolution / 60,
            sample.enabled,
            sample.received_ns,
            sample.device_epoch,
            sample.valid,
            sample.error_code,
        )

    def stop(self, deadline_ns):
        self.drive.stop(deadline_ns)

    def close(self):
        self.drive.close()
