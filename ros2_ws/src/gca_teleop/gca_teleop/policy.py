"""Explicit actuator policy, independent from physical input calibration and ROS."""

from dataclasses import dataclass, replace
from math import isfinite


@dataclass(frozen=True)
class TeleopPolicy:
    control_profile: str = "dual_axis"
    max_servo_rpm: float = 0.0
    manual_axis_to_servo_sign: int = -1
    manual_input_mode: str = "qualified_state"
    manual_input_max_age_s: float = 0.15

    def __post_init__(self):
        if self.control_profile not in ("dual_axis", "servo"):
            raise ValueError("UNKNOWN_CONTROL_PROFILE")
        if type(self.max_servo_rpm) not in (int, float) or not isfinite(self.max_servo_rpm):
            raise ValueError("INVALID_SERVO_SPEED_LIMIT")
        if self.control_profile == "servo" and self.max_servo_rpm <= 0:
            raise ValueError("INVALID_SERVO_SPEED_LIMIT")
        if self.control_profile == "dual_axis" and self.max_servo_rpm != 0:
            raise ValueError("DUAL_AXIS_HAS_NO_SERVO_SPEED_LIMIT")
        if type(self.manual_axis_to_servo_sign) is not int or self.manual_axis_to_servo_sign != -1:
            raise ValueError("MANUAL_SERVO_DIRECTION_MUST_BE_REVERSE")
        if self.manual_input_mode not in ("qualified_state", "joydev_hold"):
            raise ValueError("UNKNOWN_MANUAL_INPUT_MODE")
        if (
            type(self.manual_input_max_age_s) not in (int, float)
            or not isfinite(self.manual_input_max_age_s)
            or self.manual_input_max_age_s <= 0
        ):
            raise ValueError("INVALID_MANUAL_INPUT_AGE")
        if self.control_profile == "dual_axis" and self.manual_input_mode != "qualified_state":
            raise ValueError("JOYDEV_HOLD_REQUIRES_SERVO")

    @property
    def is_servo(self):
        return self.control_profile == "servo"

    @property
    def manual_hold_enabled(self):
        return self.is_servo and self.manual_input_mode == "joydev_hold"

    @property
    def control_policy(self):
        return "MANUAL_SERVO_VELOCITY" if self.is_servo else "LEGACY_DUAL_AXIS"

    def input_max_age_s(self, profile_max_age_s):
        if self.manual_hold_enabled:
            # Reader observations use the servo policy, not event-age calibration.
            return self.manual_input_max_age_s
        if self.is_servo:
            return min(profile_max_age_s, self.manual_input_max_age_s)
        return profile_max_age_s

    def mapped(self, value):
        # Keep physical LX values available to logs while excluding them from
        # the neutral gate. No calibration/profile evidence changes here.
        return replace(value, centered=value.ly == 0) if self.is_servo else value

    def servo_rpm(self, mapped):
        return self.manual_axis_to_servo_sign * mapped.ly * self.max_servo_rpm


DUAL_AXIS_POLICY = TeleopPolicy()
