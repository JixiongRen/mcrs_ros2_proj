"""Finite point-to-point motion; no ROS, serial I/O or authorization decisions."""

from math import floor, isfinite

from gca.algorithms.control import PositionController, clamp, slew
from gca.algorithms.trajectory import Trapezoid
from gca.domain.models import DeviceError
from gca.domain.position import PositionOutput


class EncoderPositionStrategy:
    def __init__(self, config):
        self.settings = config
        self.config = config.position
        self.mapping = self.config.mechanics.mapping
        pid = self.config.pid
        self._pid = PositionController(
            pid.kp, pid.ki, pid.kd, pid.kaw, pid.integral_limit_mm_s, pid.dt_max_s
        )
        self.stop()

    def stop(self):
        self._trajectory = None
        self._reference_id = self._epoch = ""
        self._last_seq = self._last_sample_ns = self._last_output_ns = 0
        self._last_velocity = 0.0
        self._settled_since = None
        self._pid.reset()

    def start(self, target_mm, max_velocity_mm_s, max_acceleration_mm_s2, feedback, now_ns):
        motion = self.config.motion
        if not all(
            type(v) in (int, float) and isfinite(v)
            for v in (target_mm, max_velocity_mm_s, max_acceleration_mm_s2)
        ):
            raise DeviceError("INVALID_ARGUMENT")
        if not motion.target_min_mm <= target_mm <= motion.target_max_mm:
            raise DeviceError("TARGET_OUT_OF_RANGE")
        if not (
            0 < max_velocity_mm_s <= motion.max_linear_mm_s
            and 0 < max_acceleration_mm_s2 <= motion.max_linear_mm_s2
        ):
            raise DeviceError("LIMIT_EXCEEDED")
        if not feedback.position_valid or not feedback.reference_id or not feedback.velocity_valid:
            raise DeviceError("REFERENCE_REQUIRED")
        if abs(feedback.velocity_mm_s) > motion.velocity_tolerance_mm_s:
            raise DeviceError("NOT_STOPPED")
        self.stop()
        self._reference_id, self._epoch = feedback.reference_id, feedback.device_epoch
        self._last_seq, self._last_sample_ns = feedback.seq, feedback.sampled_ns
        self._last_output_ns = now_ns
        self._velocity_limit, self._acceleration_limit = max_velocity_mm_s, max_acceleration_mm_s2
        self._trajectory = Trapezoid(
            feedback.position_mm, target_mm, max_velocity_mm_s, max_acceleration_mm_s2, now_ns
        )

    def at_target(self, feedback, target_mm):
        motion = self.config.motion
        return bool(
            feedback.position_valid
            and feedback.velocity_valid
            and not feedback.error_code
            and abs(target_mm - feedback.position_mm) <= motion.position_tolerance_mm
            and abs(feedback.velocity_mm_s) <= motion.velocity_tolerance_mm_s
        )

    def step(self, feedback, now_ns, output_seq, generation):
        if self._trajectory is None:
            return None
        if (
            not feedback.position_valid
            or not feedback.velocity_valid
            or feedback.error_code
            or feedback.reference_id != self._reference_id
            or feedback.device_epoch != self._epoch
        ):
            raise DeviceError(feedback.error_code or "POSITION_FEEDBACK_INVALID")
        if (
            not 0
            <= now_ns - feedback.evidence_ns
            <= round(self.config.encoder.feedback_max_age_s * 1e9)
        ):
            raise DeviceError("ENCODER_FEEDBACK_STALE")
        if feedback.seq <= self._last_seq:
            return None
        dt = (feedback.sampled_ns - self._last_sample_ns) / 1e9
        output_dt = (now_ns - self._last_output_ns) / 1e9
        if (
            not 0 < dt <= self.config.pid.dt_max_s
            or not 0 < output_dt <= self.settings.output_ttl_s
        ):
            raise DeviceError("POSITION_DT_INVALID")
        reference = self._trajectory.at(now_ns)
        error = reference.position - feedback.position_mm
        pid = self._pid.request(
            reference.position, reference.velocity, feedback.position_mm, feedback.velocity_mm_s
        )
        arrival = (
            reference.complete
            and abs(self._trajectory.target - feedback.position_mm)
            <= self.config.motion.position_tolerance_mm
        )
        requested = 0.0 if arrival else pid.requested
        limited = slew(
            clamp(requested, -self._velocity_limit, self._velocity_limit),
            self._last_velocity,
            self._acceleration_limit,
            output_dt,
        )
        motion = self.config.motion
        limited = self.config.effective_braking.limit_velocity(
            limited,
            feedback.position_mm,
            motion.linear_min_mm,
            motion.linear_max_mm,
            motion.boundary_margin_mm,
        )
        # Let the existing drive perform its register rounding. Cap its result
        # by first clipping the requested RPM to a reachable register limit.
        rpm_limit = (
            floor(
                abs(self.mapping.velocity_to_rpm(self._velocity_limit))
                / self.settings.device.velocity_unit_rpm
                + 1e-12
            )
            * self.settings.device.velocity_unit_rpm
        )
        rpm = clamp(self.mapping.velocity_to_rpm(limited), -rpm_limit, rpm_limit)
        limited = self.mapping.rpm_to_velocity(rpm)
        self._pid.commit(output_seq, generation, error, pid.requested, limited, dt)
        self._last_velocity, self._last_output_ns = limited, now_ns
        self._last_seq, self._last_sample_ns = feedback.seq, feedback.sampled_ns
        if arrival and self.at_target(feedback, self._trajectory.target):
            if self._settled_since is None:
                self._settled_since = feedback.sampled_ns
        else:
            self._settled_since = None
        complete = (
            self._settled_since is not None
            and feedback.sampled_ns - self._settled_since >= round(motion.settle_s * 1e9)
        )
        return PositionOutput(
            requested,
            limited,
            rpm,
            reference.position,
            reference.velocity,
            complete,
            feedback,
            pid.proportional,
            pid.integral,
            pid.derivative,
        )

    def accepted(self, output_seq, generation, rpm):
        return self._pid.applied(output_seq, generation, self.mapping.rpm_to_velocity(rpm))
