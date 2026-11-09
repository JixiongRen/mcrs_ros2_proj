"""Encoder observes the accepted servo plant, including a fixed transmission slip."""

from math import isfinite
from threading import RLock
from time import monotonic_ns
from uuid import uuid4

from gca.domain.models import DeviceError, EncoderSample
from gca.simulation.servo import SimulatedVelocityDrive


class PositionSimulationDrive:
    """Serialize pure-model advances shared by two simulated device owners.

    Encoder reads advance the mechanism only; they do not issue servo bus
    instructions, refresh its watchdog, or create fake shaft telemetry.
    """

    def __init__(self, settings, *, clock=monotonic_ns, slip_ratio=1.0, drive=None):
        if (
            type(slip_ratio) not in (int, float)
            or not isfinite(slip_ratio)
            or not 0 < slip_ratio <= 1
        ):
            raise ValueError("simulation slip_ratio must be in (0, 1]")
        self._drive = drive or SimulatedVelocityDrive(settings, clock=clock)
        if not isinstance(self._drive, SimulatedVelocityDrive):
            raise ValueError("position simulation requires a SimulatedVelocityDrive")
        self._lock = RLock()
        self.settings, self.slip_ratio = settings, slip_ratio

    def __getattr__(self, name):
        return getattr(self._drive, name)

    def _call(self, name, *args, **kwargs):
        with self._lock:
            return getattr(self._drive, name)(*args, **kwargs)

    def open_identified(self):
        return self._call("open_identified")

    def configure_velocity_mode(self):
        return self._call("configure_velocity_mode")

    def set_torque(self, enabled, deadline_ns=None, *, enable_guard=None):
        return self._call("set_torque", enabled, deadline_ns, enable_guard=enable_guard)

    def write_velocity_rpm(self, rpm, deadline_ns, *, write_guard=None):
        return self._call("write_velocity_rpm", rpm, deadline_ns, write_guard=write_guard)

    def read_shaft_state(self, deadline_ns):
        return self._call("read_shaft_state", deadline_ns)

    def stop(self, deadline_ns):
        return self._call("stop", deadline_ns)

    def close(self):
        return self._call("close")

    def encoder_position_mm(self):
        with self._lock:
            if self._drive.opened:
                self._drive._advance()
            mechanics = self.settings.position.mechanics
            return (
                self._drive._position_count
                / 4096
                * mechanics.mm_per_shaft_revolution
                * mechanics.direction_sign
                * self.slip_ratio
            )


class SimulatedPositionEncoder:
    def __init__(self, drive, config, *, clock=monotonic_ns):
        self.drive, self.config, self.clock = drive, config, clock
        self.fail_read = False
        self._opened = False
        self._seq = 0
        self._epoch = ""

    def open(self):
        if self._opened:
            raise DeviceError("ALREADY_OPEN")
        self._opened, self._seq, self._epoch = True, 0, str(uuid4())

    def read(self, deadline_ns):
        started = self.clock()
        if not self._opened or started >= deadline_ns:
            raise DeviceError("DEVICE_CLOSED" if not self._opened else "DEVICE_TIMEOUT")
        if self.fail_read:
            raise DeviceError("ENCODER_IO_ERROR", "injected simulated encoder failure")
        position = self.drive.encoder_position_mm()
        raw = (
            round(position / (self.config.direction_sign * self.config.mm_per_count))
            % self.config.count_modulus
        )
        self._seq += 1
        return EncoderSample(raw, self._seq, self.clock(), self._epoch, request_started_ns=started)

    def close(self):
        self._opened = False
