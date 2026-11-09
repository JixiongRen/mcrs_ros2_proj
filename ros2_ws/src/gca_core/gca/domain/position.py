"""Pure contracts for external-position feedback and its injected controller.

A new sample is consumed once with its original time; limits are committed once
and the device quantization residual once. These contracts create no device and
carry no default mechanical mapping or PID gains.
"""

from collections.abc import Callable
from dataclasses import dataclass
from math import isfinite
from typing import Protocol

from gca.algorithms.control import boundary_velocity
from gca.domain.models import EncoderSample, OperationResult


@dataclass(frozen=True)
class ResolvedBraking:
    """Effective braking assumptions and their source, not measurement evidence."""

    deceleration_mm_s2: float
    delay_s: float
    deceleration_source: str
    delay_source: str

    def limit_velocity(self, value, position, low, high, margin=0.0):
        return boundary_velocity(
            value, position, low, high, self.deceleration_mm_s2, self.delay_s, margin
        )


def resolve_braking_parameters(motion, defaults) -> ResolvedBraking:
    """Use explicitly configured defaults only for the two unknown (None) values.

    Never changes the measured-value placeholders. Invalid supplied values are
    errors, not a reason to silently select defaults. The caller uses this same
    result for boundary calculation and recording its parameter provenance.
    """
    values, sources = {}, {}
    for name in ("braking_deceleration_mm_s2", "braking_delay_s"):
        value = motion[name]
        use_default = value is None
        if use_default:
            value = defaults[name]
        if type(value) not in (int, float) or not isfinite(value):
            raise ValueError(f"{name} must resolve to a finite number")
        if value < 0 or (name == "braking_deceleration_mm_s2" and value == 0):
            raise ValueError(f"{name} outside its positive/nonnegative range")
        values[name] = float(value)
        sources[name] = "default" if use_default else "configured"
    return ResolvedBraking(
        values["braking_deceleration_mm_s2"],
        values["braking_delay_s"],
        sources["braking_deceleration_mm_s2"],
        sources["braking_delay_s"],
    )


@dataclass(frozen=True)
class ExternalPositionFeedback:
    position_mm: float
    velocity_mm_s: float
    position_valid: bool
    velocity_valid: bool
    seq: int
    sampled_ns: int
    device_epoch: str
    reference_id: str
    request_started_ns: int = 0
    raw_count: int = 0
    error_code: str = ""

    @property
    def evidence_ns(self):
        return self.request_started_ns or self.sampled_ns


@dataclass(frozen=True)
class MechanicalMapping:
    mm_per_shaft_revolution: float
    direction_sign: int
    calibration_id: str

    def __post_init__(self):
        if (
            type(self.mm_per_shaft_revolution) not in (float, int)
            or not isfinite(self.mm_per_shaft_revolution)
            or self.mm_per_shaft_revolution <= 0
            or type(self.direction_sign) is not int
            or self.direction_sign not in (-1, 1)
            or not isinstance(self.calibration_id, str)
            or not self.calibration_id
        ):
            raise ValueError("explicit mechanical mapping and provenance required")

    def velocity_to_rpm(self, velocity_mm_s: float) -> float:
        if type(velocity_mm_s) not in (int, float) or not isfinite(velocity_mm_s):
            raise ValueError("finite velocity required")
        return self.direction_sign * 60.0 * velocity_mm_s / self.mm_per_shaft_revolution

    def rpm_to_velocity(self, rpm: float) -> float:
        if type(rpm) not in (int, float) or not isfinite(rpm):
            raise ValueError("finite RPM required")
        return self.direction_sign * rpm * self.mm_per_shaft_revolution / 60.0


@dataclass(frozen=True)
class EncoderAcquisitionSnapshot:
    sample: EncoderSample | None = None
    healthy: bool = False
    error_code: str = ""


class ExternalPositionPort(Protocol):
    """Position-only raw acquisition; the control owner does estimation."""

    def start(self, on_fault: Callable[[str], None] | None = None) -> OperationResult: ...
    def snapshot(self) -> EncoderAcquisitionSnapshot: ...
    def close(self) -> None: ...


class PositionFeedbackPort(Protocol):
    def update(self, sample: EncoderSample, now_ns: int) -> ExternalPositionFeedback: ...
    def snapshot(self, now_ns: int) -> ExternalPositionFeedback: ...
    def set_reference(self, position_mm: float, now_ns: int) -> OperationResult: ...
    def invalidate(self, reason: str) -> None: ...


@dataclass(frozen=True)
class PositionOutput:
    requested_mm_s: float
    limited_mm_s: float
    rpm: float
    reference_position_mm: float
    reference_velocity_mm_s: float
    complete: bool
    feedback: ExternalPositionFeedback
    proportional: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0


class PositionStrategy(Protocol):
    """Injected external-position PID and trajectory lifecycle.

    The owner resolves relative targets from fresh accepted feedback, verifies
    absolute reference_id, and enforces trajectory/position/velocity dwell and
    separate stop evidence. reset/stop clear targets, integral and receipts.
    step rejects duplicate/stale/invalid samples rather than integrating again.
    """

    def start(
        self,
        target_mm: float,
        max_velocity_mm_s: float,
        max_acceleration_mm_s2: float,
        feedback: ExternalPositionFeedback,
        now_ns: int,
    ) -> None: ...
    def step(
        self, feedback: ExternalPositionFeedback, now_ns: int, output_seq: int, generation: int
    ) -> PositionOutput | None: ...
    def accepted(self, output_seq: int, generation: int, rpm: float) -> bool: ...
    def at_target(self, feedback: ExternalPositionFeedback, target_mm: float) -> bool: ...
    def stop(self) -> None: ...


@dataclass(frozen=True)
class PositionComponents:
    acquisition: ExternalPositionPort
    feedback: PositionFeedbackPort
    strategy: PositionStrategy
