"""Measured-velocity PI for Jog; it never integrates a position target."""

from collections import OrderedDict
from math import isfinite

from .control import clamp, slew


class JogVelocityController:
    """Single-control-owner mm/s PI with real sample dt and once-only receipts.

    kp is dimensionless; ki/kaw are 1/s; integral stores mm/s. Neutral removes
    historical correction instead of turning it into a position HOLD. The
    outer runtime still applies final velocity/acceleration/braking constraints.
    """

    def __init__(
        self,
        kp: float = 0.25,
        ki: float = 4.0,
        kaw: float = 2.0,
        integral_limit: float = 10.0,
        filter_s: float = 0.04,
        dt_max: float = 0.2,
    ) -> None:
        if (
            not all(isfinite(v) and v >= 0 for v in (kp, ki, kaw, integral_limit, filter_s))
            or dt_max <= 0
        ):
            raise ValueError("invalid Jog velocity controller parameters")
        self.kp, self.ki, self.kaw = kp, ki, kaw
        self.limit, self.filter_s, self.dt_max = integral_limit, filter_s, dt_max
        self.reset()

    def reset(self, measured_velocity: float = 0.0) -> None:
        """Control owner resets mm/s state after a confirmed stop or first entry."""
        self.integral = 0.0
        self.filtered = measured_velocity
        self.target = 0.0
        self.error = 0.0
        self._tracking = OrderedDict()
        self._last_seq = -1

    def shape(self, requested: float, acceleration: float, dt: float) -> float:
        """Return mm/s target limited by mm/s² and actual seconds; invalid dt raises."""
        if not 0 < dt <= self.dt_max:
            raise ValueError("Jog update dt outside budget")
        self.target = slew(requested, self.target, acceleration, dt)
        return self.target

    def request(self, measured_velocity: float, dt: float, *, neutral: bool = False) -> float:
        """Once per fresh encoder sample return PI request in mm/s, without I/O."""
        if not 0 < dt <= self.dt_max or not isfinite(measured_velocity):
            raise ValueError("Jog feedback dt/value outside budget")
        alpha = dt / (self.filter_s + dt) if self.filter_s else 1.0
        self.filtered += alpha * (measured_velocity - self.filtered)
        self.error = self.target - self.filtered
        if neutral:
            self.integral = 0.0
            return self.target  # Deceleration only; never catch up past travel.
        return self.target + self.kp * self.error + self.integral

    def commit(
        self,
        seq: int,
        generation: int,
        requested: float,
        limited: float,
        dt: float,
        *,
        neutral: bool = False,
    ) -> None:
        """Integrate error/final-limit residual once using sample seconds and mm/s."""
        if seq <= self._last_seq:
            return
        self._last_seq = seq
        if neutral:
            self.integral = 0.0
            self._tracking.clear()
            return
        self.integral = clamp(
            self.integral + self.ki * self.error * dt + self.kaw * (limited - requested) * dt,
            -self.limit,
            self.limit,
        )
        self._tracking[(generation, seq)] = (dt, limited)
        while len(self._tracking) > 128:
            self._tracking.popitem(last=False)

    def applied(self, seq: int, generation: int, accepted: float) -> bool:
        """Settle accepted mm/s quantization residual once; unknown/duplicate is false."""
        item = self._tracking.pop((generation, seq), None)
        if item is None:
            return False
        dt, limited = item
        self.integral = clamp(
            self.integral + self.kaw * (accepted - limited) * dt, -self.limit, self.limit
        )
        return True
