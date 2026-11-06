"""Continuous Jog phase with bounded finite, non-uniform rising-edge schedules."""

from dataclasses import dataclass
from math import ceil, copysign, floor, sqrt

from gca.config import RuntimeConfig

from .control import boundary_velocity, clamp, slew
from .trajectory import PulseQuantizer


@dataclass(frozen=True)
class RotationProfile:
    signed_pulses: int
    edge_offsets_us: tuple[int, ...]
    duration_s: float
    start_rate_deg_s: float
    end_rate_deg_s: float
    phase_start_pulses: float
    phase_end_pulses: float


class JogRotationPlanner:
    """Pure planner; only complete receipts can advance runtime confirmed count.

    A bounded fractional phase survives sub-pulse commands. Reference phase is
    driven by real elapsed input time; bounded phase feedback compensates
    observed single-wave gaps without exceeding rate/acceleration limits.
    Neutral discards unsubmitted whole-edge debt, never schedules catch-up.
    """

    def __init__(self, config: RuntimeConfig, quantizer: PulseQuantizer) -> None:
        self.config, self.quantizer = config, quantizer
        self.reset(0, 0)

    def reset(self, now_ns: int, confirmed_pulses: int) -> None:
        """Control-owner entry reset, using absolute local ns and confirmed edges."""
        self.updated_ns = now_ns
        self.cursor_ns = now_ns
        self.reference_phase = float(confirmed_pulses)
        self.fraction = 0.0
        self.commanded_rate = self.wave_rate = 0.0
        self.requested_rate = 0.0
        self._reversing = False

    def update(self, requested_rate: float, now_ns: int, angle_deg: float) -> None:
        """Integrate limited deg/s with actual dt; time-budget errors raise ValueError."""
        dt = (now_ns - self.updated_ns) / 1e9
        if dt < 0 or dt > self.config.jog_dt_max_s:
            raise ValueError("Jog rotation dt outside budget")
        if dt == 0:
            self.requested_rate = requested_rate
            return
        if requested_rate and (
            self.commanded_rate * requested_rate < 0 or self.wave_rate * requested_rate < 0
        ):
            self._reversing = True
        old = self.commanded_rate
        requested_rate = clamp(
            requested_rate, -self.config.max_rotation_deg_s, self.config.max_rotation_deg_s
        )
        requested_rate = boundary_velocity(
            requested_rate,
            angle_deg,
            self.config.rotation_min_deg,
            self.config.rotation_max_deg,
            self.config.max_rotation_deg_s2,
            self.config.pulse_segment_s + self.config.io_period_s,
        )
        self.commanded_rate = slew(requested_rate, old, self.config.max_rotation_deg_s2, dt)
        self.reference_phase += (
            (old + self.commanded_rate) * 0.5 * dt * self.quantizer.pulses_per_deg
        )
        self.requested_rate, self.updated_ns = requested_rate, now_ns

    @staticmethod
    def _crossing(target, phase0, rate0, acceleration, duration):
        c = target - phase0
        # A half-edge remainder can put the first boundary exactly at the
        # segment start, including rest/reversal. Its earliest crossing is
        # zero; it is not the far endpoint of a degenerate quadratic. This
        # equality is exact so real sub-microsecond movement is not discarded.
        if c == 0:
            return 0.0
        if acceleration == 0:
            if rate0 == 0:
                raise ValueError("stationary phase cannot reach a different edge")
            t = c / rate0
        else:
            discriminant = rate0 * rate0 + 2 * acceleration * c
            if discriminant < 0:
                raise ValueError("phase profile cannot reach the pulse edge")
            # Rationalized root avoids cancellation for near-constant rates.
            # A zero denominator with c!=0 has no representable valid crossing;
            # fail closed instead of inventing a far-end edge.
            denominator = rate0 + copysign(sqrt(discriminant), c)
            if denominator == 0:
                raise ValueError("pulse edge crossing is numerically degenerate")
            t = 2 * c / denominator
        if not -1e-9 <= t <= duration + 1e-9:
            raise ValueError("pulse edge is outside phase profile")
        return min(duration, max(0.0, t))

    def plan(
        self, now_ns: int, confirmed_pulses: int, *, angle_offset: float = 0.0
    ) -> RotationProfile | None:
        """Return a finite µs edge plan, or None; never claim accepted/executed edges.

        Caller permits one in-flight plan and validates deadlines before output.
        Profile rates satisfy configured deg/s²; GPIO/real mechanics need HIL.
        """
        if now_ns < self.cursor_ns:
            return None
        c = self.config
        duration = c.pulse_segment_s
        motion_s = (
            duration
            - (c.direction_setup_us + max(c.direction_hold_us, c.pulse_low_us) + c.pulse_high_us)
            / 1e6
        )
        phase_error = (
            self.reference_phase - confirmed_pulses - self.fraction
        ) / self.quantizer.pulses_per_deg
        desired = self.commanded_rate
        if self.requested_rate and not self._reversing:
            desired += c.jog_phase_gain_s * phase_error
        else:
            # Neutral/reversal discards unsubmitted whole-edge debt; only
            # the bounded physical deceleration and fractional phase survive.
            self.reference_phase = confirmed_pulses + self.fraction
        desired = clamp(desired, -c.max_rotation_deg_s, c.max_rotation_deg_s)
        start = self.wave_rate
        end = slew(desired, start, c.max_rotation_deg_s2, motion_s)
        if start * end < 0:
            end = 0.0  # A segment has one direction; reversal passes through rest.
        if not self.requested_rate and abs(start) < 1e-9 and abs(end) < 1e-9:
            self.cursor_ns = now_ns + round(duration * 1e9)
            return None
        scale = self.quantizer.pulses_per_deg
        displacement = 0.5 * (start + end) * motion_s * scale
        phase0, phase1 = self.fraction, self.fraction + displacement
        count = floor(phase1 + 0.5) if displacement >= 0 else ceil(phase1 - 0.5)
        low = ceil((c.rotation_min_deg - angle_offset) * scale)
        high = floor((c.rotation_max_deg - angle_offset) * scale)
        if not low <= confirmed_pulses + count <= high:
            raise ValueError("Jog rotation braking profile would cross a soft limit")
        edges = []
        sign = 1 if count > 0 else -1
        for index in range(abs(count)):
            target = sign * (index + 0.5)
            t = self._crossing(
                target, phase0, start * scale, (end - start) * scale / motion_s, motion_s
            )
            edges.append(c.direction_setup_us + max(0, round(t * 1e6)))
        self.fraction = phase1 - count
        self.wave_rate = end
        if (
            self._reversing
            and self.commanded_rate * self.requested_rate >= 0
            and end * self.requested_rate >= 0
        ):
            self._reversing = False
        self.cursor_ns = now_ns + round(duration * 1e9)
        if not edges:
            return None
        return RotationProfile(count, tuple(edges), duration, start, end, phase0, phase1)
