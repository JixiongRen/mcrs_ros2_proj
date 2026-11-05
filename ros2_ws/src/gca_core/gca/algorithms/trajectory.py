"""Analytical rest-to-rest trapezoidal trajectories and pulse quantization."""

from dataclasses import dataclass
from math import isfinite, sqrt


@dataclass(frozen=True)
class Reference:
    position: float
    velocity: float
    complete: bool


class Trapezoid:
    """Finite velocity/acceleration constrained trajectory in mm or degrees.

    This initial implementation admits a new point-to-point command only from
    rest; replace-on-the-fly motion is intentionally rejected by the runtime.
    """

    def __init__(self, start, target, velocity, acceleration, started_ns):
        if not all(isfinite(v) for v in (start, target, velocity, acceleration)):
            raise ValueError("finite trajectory inputs required")
        if velocity <= 0 or acceleration <= 0:
            raise ValueError("positive trajectory limits required")
        self.start, self.target = start, target
        self.acceleration, self.started_ns = acceleration, started_ns
        self.sign = 1.0 if target >= start else -1.0
        self.distance = abs(target - start)
        self.ramp = min(velocity / acceleration, sqrt(self.distance / acceleration))
        self.peak = self.ramp * acceleration
        self.cruise = max(0.0, self.distance / self.peak - self.ramp) if self.peak else 0.0
        self.duration = 2 * self.ramp + self.cruise

    def at(self, now_ns):
        t = max(0.0, (now_ns - self.started_ns) / 1e9)
        if t >= self.duration:
            return Reference(self.target, 0.0, True)
        if t < self.ramp:
            d, v = 0.5 * self.acceleration * t * t, self.acceleration * t
        elif t < self.ramp + self.cruise:
            d = 0.5 * self.acceleration * self.ramp**2 + self.peak * (t - self.ramp)
            v = self.peak
        else:
            left = self.duration - t
            d, v = self.distance - 0.5 * self.acceleration * left * left, self.acceleration * left
        return Reference(self.start + self.sign * d, self.sign * v, False)


class PulseQuantizer:
    """Absolute target rounding prevents loss of repeated sub-pulse increments."""

    def __init__(self, pulses_per_rev, catheter_per_motor_ratio):
        if pulses_per_rev <= 0 or catheter_per_motor_ratio <= 0:
            raise ValueError("positive pulse calibration required")
        self.pulses_per_deg = pulses_per_rev / (360 * catheter_per_motor_ratio)

    def pulses(self, angle_deg):
        if not isfinite(angle_deg):
            raise ValueError("finite angle required")
        return round(angle_deg * self.pulses_per_deg)

    def angle(self, pulses):
        return pulses / self.pulses_per_deg
