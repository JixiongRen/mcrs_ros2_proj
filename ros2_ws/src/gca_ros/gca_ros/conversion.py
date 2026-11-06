"""The single SI-to-core boundary; feedback quality is copied explicitly."""

import math

from gca.domain.manual import ManualInputQuality
from gca.domain.models import CommandContext, JogCommand, MotionCommand, MotionLimits
from gca_system_interfaces.msg import GcaState, StopStatus
from gca_system_interfaces.msg import ManualInputQuality as QualityMessage


def context_from_ros(msg):
    return CommandContext(
        msg.session_id, msg.task_id, msg.command_id, int(msg.generation), int(msg.sequence)
    )


def motion_from_ros(goal):
    return MotionCommand(
        context=context_from_ros(goal.context),
        axis_mask=int(goal.target.axis_mask),
        mode=int(goal.target.mode),
        linear_mm=goal.target.linear_m * 1000.0,
        rotation_deg=math.degrees(goal.target.rotation_rad),
        limits=MotionLimits(
            goal.limits.max_linear_m_s * 1000.0,
            goal.limits.max_linear_m_s2 * 1000.0,
            math.degrees(goal.limits.max_rotation_rad_s),
            math.degrees(goal.limits.max_rotation_rad_s2),
        ),
        end_policy=int(goal.end_policy),
        freshness_token=goal.token,
        admission_timeout_s=goal.admission_timeout_s,
        execution_timeout_s=goal.execution_timeout_s,
        reference_id=goal.target.reference_id,
    )


def quality_from_ros(msg):
    return ManualInputQuality(
        **{name: getattr(msg, name) for name in ManualInputQuality.__dataclass_fields__}
    )


def quality_to_ros(quality):
    return QualityMessage(
        **{name: getattr(quality, name) for name in ManualInputQuality.__dataclass_fields__}
    )


def jog_from_ros(msg):
    return JogCommand(
        context_from_ros(msg.context),
        msg.linear_velocity_m_s * 1000.0,
        math.degrees(msg.rotation_velocity_rad_s),
        int(msg.axis_mask),
        msg.token,
        msg.ttl_s,
        msg.deadman,
        quality=quality_from_ros(msg.quality) if msg.deadman else None,
    )


def stop_from_core(state):
    msg = StopStatus()
    msg.progress = state.stop_progress
    msg.reason = state.error_code or state.message
    msg.linear_confirmed = state.stop_progress == "CONFIRMED"
    msg.rotation_confirmed = state.stop_progress == "CONFIRMED" and state.engine_idle
    msg.evidence_kind = state.stop_evidence
    return msg


def state_from_core(state, stamp=None, requested_revision=""):
    msg = GcaState()
    if stamp is not None:
        msg.stamp = stamp
    for name in (
        "boot_id",
        "device_epoch",
        "state_seq",
        "state",
        "lifecycle_active",
        "armed",
        "sample_seq",
        "engine_idle",
        "confirmed_pulses",
        "reserved_pulses",
        "reference_id",
        "calibration_id",
        "session_id",
        "generation",
        "output_generation",
    ):
        setattr(msg, name, getattr(state, name))
    msg.mode = "GCA_ONLY"
    msg.linear_position_m = state.position_mm / 1000.0
    msg.linear_velocity_m_s = state.velocity_mm_s / 1000.0
    msg.linear_valid = state.linear_valid
    msg.linear_velocity_valid = state.velocity_valid
    msg.feedback_age_s = state.sample_age_s
    msg.rotation_estimated_rad = math.radians(state.rotation_deg)
    msg.rotation_estimate_valid = state.rotation_estimate_valid
    msg.rotation_reference_valid = state.rotation_reference_valid
    msg.linear_reference_valid = state.linear_referenced
    msg.active_command_id = state.command_id
    msg.requested_config_revision = requested_revision or state.config_revision
    msg.applied_config_revision = state.config_revision
    msg.fault_code = state.error_code
    msg.outputs_held = state.state == "HOLDING"
    msg.stop = stop_from_core(state)
    msg.target_linear_m = state.target_linear_mm / 1000.0
    msg.reference_linear_m = state.reference_linear_mm / 1000.0
    msg.target_rotation_rad = math.radians(state.target_rotation_deg)
    msg.reference_rotation_rad = math.radians(state.reference_rotation_deg)
    msg.requested_velocity_m_s = state.requested_velocity_mm_s / 1000.0
    msg.applied_velocity_m_s = state.applied_velocity_mm_s / 1000.0
    msg.output_seq = state.output_seq
    msg.stop_serial = getattr(state, "stop_serial", 0)
    msg.input_valid_until_ns = getattr(state, "input_valid_until_ns", 0)
    msg.jog_linear_velocity_m_s = getattr(state, "jog_linear_velocity_mm_s", 0.0) / 1000.0
    msg.jog_rotation_velocity_rad_s = math.radians(
        getattr(state, "jog_rotation_velocity_deg_s", 0.0)
    )
    msg.estimated_rotation_velocity_rad_s = math.radians(getattr(state, "rotation_rate_deg_s", 0.0))
    msg.rotation_planned_phase_rad = math.radians(getattr(state, "rotation_planned_phase_deg", 0.0))
    msg.jog_velocity_error_m_s = getattr(state, "jog_velocity_error_mm_s", 0.0) / 1000.0
    msg.jog_velocity_integral_m_s = getattr(state, "jog_velocity_integral_mm_s", 0.0) / 1000.0
    return msg
