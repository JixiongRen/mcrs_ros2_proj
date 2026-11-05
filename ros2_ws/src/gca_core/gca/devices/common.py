"""Device-local failures and validated immutable hardware configuration."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from gca.domain.models import DeviceError


def positive(value: float, label: str) -> None:
    if not isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")


def validate_types(instance) -> None:
    """Reject YAML booleans-as-integers, strings-as-numbers and mutable tricks."""
    from dataclasses import fields
    from typing import get_type_hints

    hints = get_type_hints(type(instance))
    for item in fields(instance):
        value, expected = getattr(instance, item.name), hints[item.name]
        if expected is bool and type(value) is not bool:
            raise ValueError(f"{item.name} must be boolean")
        if expected is int and type(value) is not int:
            raise ValueError(f"{item.name} must be integer")
        if expected is float and (type(value) not in (float, int) or not isfinite(value)):
            raise ValueError(f"{item.name} must be finite numeric")
        if expected is str and (not isinstance(value, str) or len(value) > 512):
            raise ValueError(f"{item.name} must be bounded string")
        if item.name == "modulus" and value is not None and type(value) is not int:
            raise ValueError("modulus must be integer or null")


@dataclass(frozen=True)
class DynamixelConfig:
    """Confirmed X-series Protocol-2 velocity control table and calibration.

    ``verified`` means the operator has verified model/interface/configuration;
    it is never set by merely installing the SDK. Numbers have no hardware
    defaults: they must come from a device record.
    """

    port: str
    baudrate: int
    device_id: int
    model_number: int
    mm_per_revolution: float
    velocity_unit_rpm: float
    max_velocity_raw: int
    watchdog_raw: int
    direction_sign: int = 1
    transaction_timeout_s: float = 0.08
    verified: bool = False

    def __post_init__(self) -> None:
        validate_types(self)
        if not self.port or not 0 <= self.device_id <= 252 or self.baudrate <= 0:
            raise ValueError("invalid DYNAMIXEL connection")
        if not 1 <= self.model_number <= 65535 or self.direction_sign not in (-1, 1):
            raise ValueError("invalid DYNAMIXEL identity/direction")
        if not 1 <= self.max_velocity_raw <= 1023 or not 1 <= self.watchdog_raw <= 127:
            raise ValueError("invalid DYNAMIXEL limits/watchdog")
        positive(self.mm_per_revolution, "mm_per_revolution")
        positive(self.velocity_unit_rpm, "velocity_unit_rpm")
        if abs(self.velocity_unit_rpm - 0.229) > 1e-12:
            raise ValueError("XM430-W350 velocity unit must be 0.229 rpm")
        positive(self.transaction_timeout_s, "transaction_timeout_s")


@dataclass(frozen=True)
class EncoderConfig:
    port: str
    baudrate: int
    address: int
    register: int
    bits: int
    signed: bool
    quiet_s: float
    byteorder: str = "big"
    modulus: int | None = None
    transaction_timeout_s: float = 0.08
    verified: bool = False

    def __post_init__(self) -> None:
        validate_types(self)
        if not self.port or self.baudrate <= 0 or not 1 <= self.address <= 247:
            raise ValueError("invalid encoder connection")
        if self.bits not in (16, 32, 64) or self.byteorder not in ("big", "little"):
            raise ValueError("invalid encoder count format")
        if not 0 <= self.register <= 65535 - self.bits // 16 + 1:
            raise ValueError("invalid encoder register range")
        positive(self.quiet_s, "quiet_s")
        positive(self.transaction_timeout_s, "transaction_timeout_s")


@dataclass(frozen=True)
class PigpioConfig:
    pulse_gpio: int
    direction_gpio: int
    high_us: int
    low_us: int
    direction_setup_us: int
    direction_hold_us: int
    max_segment_s: float
    max_segment_pulses: int
    positive_direction_level: int
    host: str = "localhost"
    port: int = 8888
    socket_timeout_s: float = 0.1
    verified: bool = False
    exclusive_wave_owner: bool = False
    waveform_hil_verified: bool = False

    def __post_init__(self) -> None:
        validate_types(self)
        if self.pulse_gpio == self.direction_gpio or not all(
            0 <= p <= 31 for p in (self.pulse_gpio, self.direction_gpio)
        ):
            raise ValueError("distinct BCM GPIO 0..31 required")
        if (
            min(
                self.high_us,
                self.low_us,
                self.direction_setup_us,
                self.direction_hold_us,
                self.max_segment_pulses,
            )
            <= 0
        ):
            raise ValueError("pulse timing/count limits must be positive")
        if self.positive_direction_level not in (0, 1) or not 1024 <= self.port <= 30000:
            raise ValueError("invalid direction or pigpio port")
        if self.host not in ("localhost", "127.0.0.1"):
            raise ValueError("pigpio backend supports only local numeric IPv4/localhost")
        positive(self.max_segment_s, "max_segment_s")
        positive(self.socket_timeout_s, "socket_timeout_s")


class DeviceLock:
    """Advisory OS lock shared by compliant local processes.

    This prevents our own services from competing. It is not authentication and
    cannot exclude arbitrary vendor software or an unrestricted pigpio socket;
    deployment must additionally restrict device/daemon access.
    """

    def __init__(self, resource: str):
        self.resource = resource
        self.fd = None

    def acquire(self) -> None:
        import fcntl
        import hashlib
        import os
        import tempfile

        key = hashlib.sha256(self.resource.encode()).hexdigest()[:24]
        path = os.path.join(tempfile.gettempdir(), f"gca-device-{os.getuid()}-{key}.lock")
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            if os.fstat(fd).st_uid != os.getuid():
                raise DeviceError("DEVICE_BUSY", "device lock belongs to another user")
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, DeviceError) as exc:
            os.close(fd)
            if isinstance(exc, DeviceError):
                raise
            raise DeviceError("DEVICE_BUSY", f"device owner already holds {self.resource}") from exc
        self.fd = fd

    def release(self) -> None:
        import os

        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
