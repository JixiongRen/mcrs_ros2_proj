"""Independent actuator-driven plant; target and trajectory are never inputs."""

from dataclasses import dataclass
from math import exp, isfinite
from threading import RLock


@dataclass(frozen=True)
class PlantState:
    position_mm: float
    velocity_mm_s: float
    mechanical_angle_deg: float
    accepted_velocity_mm_s: float


class MechanicalPlant:
    """First-order velocity actuator with deadband, saturation and slip.

    Call advance using an independent monotonic/simulated time source. The
    plant only consumes accepted motor velocity and emitted pulse edges; it
    has no knowledge of a motion goal, controller reference, or ROS messages.
    The lock protects physical state shared by simulated device I/O owners.
    """

    def __init__(
        self,
        now_ns=0,
        tau_s=0.04,
        max_velocity_mm_s=30.0,
        deadband_mm_s=0.01,
        slip=0.0,
        degrees_per_pulse=0.031875,
    ):
        if tau_s <= 0 or max_velocity_mm_s <= 0 or not 0 <= slip < 1:
            raise ValueError("invalid simulation plant parameters")
        self.tau_s, self.max_velocity = tau_s, max_velocity_mm_s
        self.deadband, self.slip = deadband_mm_s, slip
        self.degrees_per_pulse = degrees_per_pulse
        self.lock = RLock()
        self._now = now_ns
        self._position = self._velocity = self._angle = self._demand = 0.0
        self.stuck = False
        self.ignore_zero = False
        self.missed_step_fraction = 0.0

    def advance(self, now_ns):
        with self.lock:
            if now_ns < self._now:
                raise ValueError("plant time cannot run backwards")
            dt = (now_ns - self._now) / 1e9
            self._now = now_ns
            target = self._demand * (1 - self.slip)
            if abs(target) <= self.deadband or self.stuck:
                target = 0.0
            # Exact integration of dv/dt=(target-v)/tau for fixed demand.
            decay = exp(-dt / self.tau_s)
            self._position += target * dt + (self._velocity - target) * self.tau_s * (1 - decay)
            self._velocity = target + (self._velocity - target) * decay
            return self.snapshot()

    def accept_velocity(self, value, now_ns):
        if not isfinite(value):
            raise ValueError("finite plant demand required")
        with self.lock:
            self.advance(now_ns)
            if not (value == 0 and self.ignore_zero):
                self._demand = min(self.max_velocity, max(-self.max_velocity, value))
            return self._demand

    def emit_pulses(self, signed_pulses):
        with self.lock:
            self._angle += signed_pulses * self.degrees_per_pulse * (1 - self.missed_step_fraction)

    def disable(self, now_ns):
        """Remove drive demand; unlike a failed zero command this is an output gate."""
        with self.lock:
            self.advance(now_ns)
            self._demand = 0.0

    def snapshot(self):
        with self.lock:
            return PlantState(self._position, self._velocity, self._angle, self._demand)
