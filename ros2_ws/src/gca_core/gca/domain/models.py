"""Shared domain contract. Internal distances/angles use mm/degrees; time is ns."""

from dataclasses import dataclass, field
from typing import Optional

from .manual import ManualInputQuality


@dataclass(frozen=True)
class OperationResult:
    accepted: bool
    code: str = "OK"
    message: str = ""
    token: str = ""
    reference_id: str = ""

    def __bool__(self):
        return self.accepted


@dataclass(frozen=True)
class CommandContext:
    session_id: str
    task_id: str
    command_id: str
    generation: int
    sequence: int


@dataclass(frozen=True)
class Lease:
    session_id: str
    generation: int
    sequence: int
    ttl_s: float = 1.0
    token: str = ""


@dataclass(frozen=True)
class MotionLimits:
    max_linear_mm_s: float = 10.0
    max_linear_mm_s2: float = 50.0
    max_rotation_deg_s: float = 90.0
    max_rotation_deg_s2: float = 360.0


@dataclass(frozen=True)
class MotionCommand:
    context: CommandContext
    axis_mask: int = 3
    mode: int = 1
    linear_mm: float = 0.0
    rotation_deg: float = 0.0
    limits: MotionLimits = field(default_factory=MotionLimits)
    end_policy: int = 0
    freshness_token: str = ""
    admission_timeout_s: float = 0.5
    execution_timeout_s: float = 30.0
    reference_id: str = ""
    # Receiver-issued fixed deadline, retained across ROS/local queueing.
    # Zero is reserved for direct core callers with no remote freshness nonce.
    admission_deadline_ns: int = 0


@dataclass(frozen=True)
class JogCommand:
    context: CommandContext
    linear_velocity_mm_s: float = 0.0
    rotation_velocity_deg_s: float = 0.0
    axis_mask: int = 3
    freshness_token: str = ""
    ttl_s: float = 0.15
    deadman: bool = True
    admission_deadline_ns: int = 0
    quality: ManualInputQuality | None = None


@dataclass(frozen=True)
class RuntimeState:
    state: str = "INITIALIZING"
    lifecycle_active: bool = False
    armed: bool = False
    configured: bool = False
    position_mm: float = 0.0
    velocity_mm_s: float = 0.0
    linear_valid: bool = False
    velocity_valid: bool = False
    rotation_deg: float = 0.0
    rotation_estimate_valid: bool = False
    rotation_reference_valid: bool = False
    linear_referenced: bool = False
    reference_id: str = ""
    calibration_id: str = "simulation-only"
    sample_seq: int = 0
    sample_received_ns: int = 0
    sample_age_s: float = 0.0
    device_epoch: str = ""
    boot_id: str = ""
    session_id: str = ""
    task_id: str = ""
    command_id: str = ""
    generation: int = 0
    output_generation: int = 0
    output_seq: int = 0
    lease_remaining_s: float = 0.0
    confirmed_pulses: int = 0
    reserved_pulses: int = 0
    engine_idle: bool = True
    stop_progress: str = "CONFIRMED"
    stop_evidence: str = "INITIAL_DISABLED"
    error_code: str = ""
    message: str = ""
    state_seq: int = 0
    backend: str = "simulation"
    config_revision: str = ""
    requested_config_revision: str = ""
    applied_config_revision: str = ""
    target_linear_mm: float = 0.0
    target_rotation_deg: float = 0.0
    reference_linear_mm: float = 0.0
    reference_rotation_deg: float = 0.0
    requested_velocity_mm_s: float = 0.0
    applied_velocity_mm_s: float = 0.0
    dropped_records: int = 0
    stop_serial: int = 0
    input_valid_until_ns: int = 0
    jog_linear_velocity_mm_s: float = 0.0
    jog_rotation_velocity_deg_s: float = 0.0
    rotation_planned_phase_deg: float = 0.0
    rotation_rate_deg_s: float = 0.0
    jog_velocity_error_mm_s: float = 0.0
    jog_velocity_integral_mm_s: float = 0.0


@dataclass(frozen=True)
class MotionEvent:
    command_id: str
    outcome: str
    code: str = "OK"
    message: str = ""
    outputs_held: bool = False
    stop_confirmed: bool = False


@dataclass(frozen=True)
class DeviceIdentity:
    model: str
    device_epoch: str
    firmware: str = ""
    device_id: str = ""
    capabilities: tuple[str, ...] = ()


@dataclass(frozen=True)
class EncoderSample:
    raw_count: int
    seq: int
    received_ns: int
    device_epoch: str
    valid: bool = True
    error_code: str = ""
    request_started_ns: int = 0


@dataclass(frozen=True)
class ServoSample:
    velocity_mm_s: float
    enabled: bool
    received_ns: int
    device_epoch: str
    valid: bool = True
    error_code: str = ""


@dataclass(frozen=True)
class PulseSegment:
    segment_id: str
    signed_pulses: int
    duration_s: float
    generation: int
    device_epoch: str
    command_id: str = ""
    expires_ns: int = 0
    edge_offsets_us: tuple[int, ...] | None = None
    stop_serial: int = 0


@dataclass(frozen=True)
class PulseCompletion:
    segment_id: str
    signed_pulses: int
    generation: int
    device_epoch: str
    command_id: str = ""


@dataclass(frozen=True)
class PulseAbort:
    """Confirmed incremental pulses from the aborted active segment, or unknown.

    This is not the engine's lifetime total. Fully completed segments are
    reported exactly once through poll() and must not be counted again here.
    """

    confirmed_pulses: Optional[int]
    idle: bool


class DeviceError(RuntimeError):
    """Stable device error code plus diagnostic text, never a success value."""

    def __init__(self, code, message=""):
        self.code = code
        super().__init__(message or code)
