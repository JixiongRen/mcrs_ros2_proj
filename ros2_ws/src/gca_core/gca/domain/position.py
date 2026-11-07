"""Future external-position strategy contracts; no encoder is instantiated here.

Position strategy will reuse LinearEstimator and PositionController. A new
sample is consumed once with its original time; limits are committed once and
the device quantization residual once. Manual native-RPM control never imports
or creates this strategy. No default mechanical calibration or PID gain exists.
"""

from dataclasses import dataclass
from math import isfinite
from typing import Protocol

from gca.algorithms.control import ControllerOutput


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
            raise ValueError("explicit measured mechanical calibration required")

    def velocity_to_rpm(self, velocity_mm_s: float) -> float:
        if type(velocity_mm_s) not in (int, float) or not isfinite(velocity_mm_s):
            raise ValueError("finite velocity required")
        return self.direction_sign * 60.0 * velocity_mm_s / self.mm_per_shaft_revolution

    def rpm_to_velocity(self, rpm: float) -> float:
        if type(rpm) not in (int, float) or not isfinite(rpm):
            raise ValueError("finite RPM required")
        return self.direction_sign * rpm * self.mm_per_shaft_revolution / 60.0


class ExternalPositionPort(Protocol):
    """Only POSITION_EXTERNAL_ENCODER may own/start/read this future port."""

    def open(self) -> None: ...
    def read(self, deadline_ns: int) -> ExternalPositionFeedback: ...
    def close(self) -> None: ...


class PositionStrategy(Protocol):
    """Contract for future real-position PID; capability remains unavailable.

    The owner resolves relative targets from fresh accepted feedback, verifies
    absolute reference_id, and enforces trajectory/position/velocity dwell and
    separate stop evidence. reset/stop clear targets, integral and receipts.
    step rejects duplicate/stale/invalid samples rather than integrating again.
    """

    def reset(self) -> None: ...
    def update_target(self, position_mm: float, reference_id: str) -> None: ...
    def step(self, feedback: ExternalPositionFeedback, dt: float) -> ControllerOutput: ...
    def commit_limited_output(
        self,
        output_seq: int,
        generation: int,
        error_mm: float,
        requested_mm_s: float,
        limited_mm_s: float,
        dt: float,
    ) -> None: ...
    def observe_accepted_output(
        self,
        output_seq: int,
        generation: int,
        accepted_mm_s: float,
    ) -> None: ...
    def stop(self) -> None: ...
