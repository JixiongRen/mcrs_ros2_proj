"""Validated, immutable runtime configuration; simulation values are not calibration."""

from dataclasses import asdict, dataclass, field, fields
from math import isfinite


@dataclass(frozen=True)
class RuntimeConfig:
    backend: str = "simulation"
    revision: str = "simulation-v1"
    control_period_s: float = 0.01
    encoder_period_s: float = 0.02
    io_period_s: float = 0.005
    feedback_max_age_s: float = 0.20
    output_ttl_s: float = 0.10
    io_timeout_s: float = 0.05
    stop_timeout_s: float = 1.0
    lease_max_s: float = 5.0
    queue_capacity: int = 64
    history_capacity: int = 256
    linear_min_mm: float = -100.0
    linear_max_mm: float = 100.0
    rotation_min_deg: float = -720.0
    rotation_max_deg: float = 720.0
    max_linear_mm_s: float = 20.0
    max_linear_mm_s2: float = 100.0
    max_rotation_deg_s: float = 180.0
    max_rotation_deg_s2: float = 720.0
    encoder_mm_per_count: float = 0.01
    encoder_count_modulus: int = 0
    max_encoder_speed_mm_s: float = 100.0
    kp: float = 3.0
    ki: float = 0.0
    kd: float = 0.0
    kaw: float = 2.0
    integral_limit_mm_s: float = 5.0
    jog_velocity_kp: float = 0.25
    jog_velocity_ki: float = 4.0
    jog_velocity_kaw: float = 2.0
    jog_integral_limit_mm_s: float = 10.0
    jog_velocity_filter_s: float = 0.04
    jog_dt_max_s: float = 0.2
    manual_input_max_age_s: float = 0.15
    jog_phase_gain_s: float = 8.0
    pulse_high_us: int = 5
    pulse_low_us: int = 5
    direction_setup_us: int = 10
    direction_hold_us: int = 10
    position_tolerance_mm: float = 0.08
    velocity_tolerance_mm_s: float = 0.15
    settle_s: float = 0.15
    pulses_per_motor_rev: int = 6400
    catheter_per_motor_ratio: float = 17 / 30
    pulse_segment_s: float = 0.03
    simulation_seed: int = 7
    simulation_noise_mm: float = 0.0
    simulation_encoder_delay_s: float = 0.0
    simulation_slip: float = 0.0
    hardware_verified: bool = False
    hardware: dict = field(default_factory=dict)

    def __post_init__(self):
        defaults = type(self).__dataclass_fields__
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name == "hardware":
                if not isinstance(value, dict):
                    raise ValueError("hardware must be an explicit mapping")
                continue
            default = defaults[item.name].default
            if isinstance(default, bool):
                if type(value) is not bool:
                    raise ValueError(f"{item.name} must be boolean")
            elif isinstance(default, int):
                if type(value) is not int:
                    raise ValueError(f"{item.name} must be integer")
            elif isinstance(default, float):
                if type(value) not in (int, float):
                    raise ValueError(f"{item.name} must be numeric")
            elif isinstance(default, str) and not isinstance(value, str):
                raise ValueError(f"{item.name} must be string")
        if self.backend not in ("simulation", "hardware"):
            raise ValueError("backend must be simulation or hardware")
        if not self.revision or len(self.revision) > 128:
            raise ValueError("configuration revision required")
        for item in fields(self):
            value = getattr(self, item.name)
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not isfinite(value):
                    raise ValueError(f"{item.name} must be finite")
        positive = (
            "control_period_s",
            "encoder_period_s",
            "io_period_s",
            "feedback_max_age_s",
            "output_ttl_s",
            "io_timeout_s",
            "stop_timeout_s",
            "lease_max_s",
            "queue_capacity",
            "history_capacity",
            "max_linear_mm_s",
            "max_linear_mm_s2",
            "max_rotation_deg_s",
            "max_rotation_deg_s2",
            "encoder_mm_per_count",
            "max_encoder_speed_mm_s",
            "position_tolerance_mm",
            "velocity_tolerance_mm_s",
            "settle_s",
            "pulses_per_motor_rev",
            "catheter_per_motor_ratio",
            "pulse_segment_s",
            "jog_dt_max_s",
            "manual_input_max_age_s",
            "pulse_high_us",
            "pulse_low_us",
            "direction_setup_us",
            "direction_hold_us",
        )
        if any(getattr(self, n) <= 0 for n in positive):
            raise ValueError("periods, limits, scales and capacities must be positive")
        if (
            not self.linear_min_mm < self.linear_max_mm
            or not self.rotation_min_deg < self.rotation_max_deg
        ):
            raise ValueError("invalid travel limits")
        if self.pulse_segment_s >= self.output_ttl_s:
            raise ValueError("finite pulse segment must fit output TTL")
        if self.encoder_period_s >= self.feedback_max_age_s:
            raise ValueError("feedback age budget must exceed acquisition period")
        if (
            not 0 <= self.simulation_slip < 1
            or self.simulation_noise_mm < 0
            or self.simulation_encoder_delay_s < 0
        ):
            raise ValueError("invalid simulation parameters")
        if any(
            getattr(self, n) < 0
            for n in ("kp", "ki", "kd", "kaw", "integral_limit_mm_s", "encoder_count_modulus")
        ):
            raise ValueError("gains/count modulus cannot be negative")
        if self.encoder_count_modulus == 1:
            raise ValueError("encoder_count_modulus must be 0 (disabled) or at least 2")
        if any(
            getattr(self, name) < 0
            for name in (
                "jog_velocity_kp",
                "jog_velocity_ki",
                "jog_velocity_kaw",
                "jog_integral_limit_mm_s",
                "jog_velocity_filter_s",
                "jog_phase_gain_s",
            )
        ):
            raise ValueError("Jog gains/filter cannot be negative")
        if (
            self.direction_setup_us
            + max(self.direction_hold_us, self.pulse_low_us)
            + self.pulse_high_us
            >= self.pulse_segment_s * 1e6
        ):
            raise ValueError("pulse guards must fit the finite segment")
        edge_rate = (
            self.max_rotation_deg_s
            * self.pulses_per_motor_rev
            / (360 * self.catheter_per_motor_ratio)
        )
        if edge_rate > 1e6 / (self.pulse_high_us + self.pulse_low_us):
            raise ValueError("maximum rotation rate violates pulse high/low timing")
        if edge_rate * self.pulse_segment_s > 100_000:
            raise ValueError("finite Jog edge profile exceeds bounded capacity")
        if self.backend == "hardware" and (not self.hardware_verified or not self.hardware):
            raise ValueError("hardware requires verified explicit device configuration")

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict):
            raise ValueError("configuration must be a mapping")
        unknown = set(values) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
        if values.get("backend") == "hardware":
            required = {f.name for f in fields(cls)} - {
                "simulation_seed",
                "simulation_noise_mm",
                "simulation_encoder_delay_s",
                "simulation_slip",
            }
            missing = required - set(values)
            if missing:
                raise ValueError(
                    f"hardware configuration cannot inherit defaults: {sorted(missing)}"
                )
        return cls(**values)

    def as_dict(self):
        return asdict(self)


ControlConfig = RuntimeConfig
