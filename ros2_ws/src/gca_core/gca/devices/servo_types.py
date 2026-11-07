"""Native shaft feedback: motor angle is not measured catheter displacement."""

from collections.abc import Callable
from contextlib import AbstractContextManager
from dataclasses import dataclass
from math import pi
from typing import Protocol

from gca.domain.models import DeviceIdentity

VelocityWriteGuard = Callable[[], AbstractContextManager[None]]


@dataclass(frozen=True)
class ServoShaftSample:
    raw_position: int
    velocity_rpm: float
    enabled: bool
    seq: int
    received_ns: int
    device_epoch: str
    valid: bool = True
    error_code: str = ""
    request_started_ns: int = 0
    hardware_error: int = 0
    watchdog_raw: int = 0

    @property
    def angle_rad(self):
        """Device register angle, scoped to device_epoch; no absolute linear reference."""
        return self.raw_position * (2 * pi / 4096)


class ServoVelocityPort(Protocol):
    def open_identified(self) -> DeviceIdentity: ...
    def configure_velocity_mode(self) -> None: ...
    def set_torque(
        self,
        enabled: bool,
        deadline_ns: int | None = None,
        *,
        enable_guard: Callable[[], bool] | None = None,
    ) -> None: ...
    def write_velocity_rpm(
        self, rpm: float, deadline_ns: int, *, write_guard: VelocityWriteGuard | None = None
    ) -> float: ...
    def read_shaft_state(self, deadline_ns: int) -> ServoShaftSample: ...
    def stop(self, deadline_ns: int) -> None: ...
    def close(self) -> None: ...
