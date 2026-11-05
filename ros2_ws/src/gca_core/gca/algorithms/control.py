"""Position-to-velocity control with explicit sample and output time bases."""

from dataclasses import dataclass
from math import isfinite, sqrt


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a finite scalar to an ordered interval."""
    if not all(isfinite(x) for x in (value, low, high)) or low > high:
        raise ValueError("invalid clamp interval/value")
    return min(high, max(low, value))


def slew(value: float, previous: float, acceleration: float, dt: float) -> float:
    """Limit velocity change using the actual output-update interval, in seconds."""
    if acceleration <= 0 or dt <= 0 or not isfinite(dt):
        raise ValueError("positive acceleration and dt required")
    return clamp(value, previous - acceleration * dt, previous + acceleration * dt)


def boundary_velocity(
    value: float,
    position: float,
    low: float,
    high: float,
    deceleration: float,
    delay: float = 0.0,
    margin: float = 0.0,
) -> float:
    """Cap approach speed using v*T + v²/(2*a) <= remaining travel.

    Units are mm, mm/s, mm/s² and seconds. Values are model/configuration
    assumptions; a software calculation is not physical stopping evidence.
    """
    if deceleration <= 0 or delay < 0 or margin < 0 or low >= high:
        raise ValueError("invalid braking configuration")
    distance = (high - position if value >= 0 else position - low) - margin
    distance = max(0.0, distance)
    limit = sqrt((deceleration * delay) ** 2 + 2 * deceleration * distance)
    limit -= deceleration * delay
    return min(value, limit) if value >= 0 else max(value, -limit)


@dataclass(frozen=True)
class ControllerOutput:
    """One sample's requested velocity and diagnostics, all in mm/s."""

    requested: float
    proportional: float
    integral: float
    derivative: float


class PositionController:
    """Stateful PI/PID whose integral stores a velocity contribution.

    A single control owner calls request/commit once per new feedback sample.
    Device quantization receipts may add only the residual against the already
    committed limited output; duplicate and old receipts are ignored.
    """

    def __init__(self, kp=3.0, ki=0.0, kd=0.0, kaw=2.0, integral_limit=10.0, dt_max=0.5):
        if (
            not all(isfinite(v) and v >= 0 for v in (kp, ki, kd, kaw, integral_limit))
            or dt_max <= 0
        ):
            raise ValueError("invalid PID configuration")
        self.kp, self.ki, self.kd, self.kaw = kp, ki, kd, kaw
        self.integral_limit, self.dt_max = integral_limit, dt_max
        self.integral = 0.0
        self._tracking = {}

    def reset(self):
        self.integral = 0.0
        self._tracking.clear()

    def request(self, reference_position, reference_velocity, position, velocity):
        values = (reference_position, reference_velocity, position, velocity)
        if not all(isfinite(v) for v in values):
            raise ValueError("non-finite controller input")
        p = self.kp * (reference_position - position)
        d = -self.kd * velocity
        return ControllerOutput(reference_velocity + p + self.integral + d, p, self.integral, d)

    def commit(self, output_seq, generation, error, raw, limited, dt):
        if not isfinite(dt) or not 0 < dt <= self.dt_max:
            raise ValueError("sample dt outside controller budget")
        key = (generation, output_seq)
        if key in self._tracking:
            return
        if self.ki == 0:
            self.integral = 0.0
            return
        self.integral = clamp(
            self.integral + self.ki * error * dt + self.kaw * (limited - raw) * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        self._tracking[key] = (dt, limited)
        # Receipts have a bounded useful lifetime; evicted output ids must not
        # be recommitted by the owner, which monotonically increments its seq.
        if len(self._tracking) > 128:
            self._tracking.pop(next(iter(self._tracking)))

    def applied(self, output_seq, generation, accepted_velocity):
        if self.ki == 0:
            return False
        tracking = self._tracking.pop((generation, output_seq), None)
        if tracking is None:
            return False
        dt, limited = tracking
        self.integral = clamp(
            self.integral + self.kaw * (accepted_velocity - limited) * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        return True
