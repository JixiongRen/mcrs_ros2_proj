"""Native feed-servo contracts. No external linear or catheter-angle feedback."""

from dataclasses import dataclass, field

from .manual import ManualInputQuality
from .models import CommandContext, RuntimeState

MANUAL_SERVO_VELOCITY = "MANUAL_SERVO_VELOCITY"
POSITION_EXTERNAL_ENCODER = "POSITION_EXTERNAL_ENCODER"


@dataclass(frozen=True)
class ServoJogCommand:
    context: CommandContext
    velocity_rpm: float
    freshness_token: str = ""
    ttl_s: float = 0.15
    deadman: bool = True
    admission_deadline_ns: int = 0
    quality: ManualInputQuality | None = None


@dataclass(frozen=True)
class ServoState:
    raw_position_count: int = 0
    shaft_angle_rad: float = 0.0
    measured_velocity_rpm: float = 0.0
    requested_velocity_rpm: float = 0.0
    accepted_velocity_rpm: float = 0.0
    torque_enabled: bool = False
    hardware_error: int = 0
    sample_seq: int = 0
    sampled_ns: int = 0
    device_epoch: str = ""
    valid: bool = False
    configured: bool = False
    sample_age_s: float = 1e9
    error_code: str = ""
    watchdog_raw: int = 0


@dataclass(frozen=True)
class FeedRuntimeState(RuntimeState):
    # No physical connection or measurement exists before runtime.start().
    stop_progress: str = "UNKNOWN"
    stop_evidence: str = "UNKNOWN"
    servo: ServoState = field(default_factory=ServoState)
    control_policy: str = MANUAL_SERVO_VELOCITY
