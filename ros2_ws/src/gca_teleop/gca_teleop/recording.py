"""Read-only input interpretation and feedback displacement for JSONL logs."""

from dataclasses import asdict
from math import degrees
from threading import Lock

from .mapping import map_input
from .model import InputError


def actions(linear, rotation):
    return {
        "linear": "推进" if linear > 0 else "后退" if linear < 0 else "零速",
        "rotation": "正向旋转" if rotation > 0 else "反向旋转" if rotation < 0 else "零速",
    }


def mapped_record(frame, profile):
    """Mapping is interpretation, not permission or proof of actuator motion."""
    if not frame.initialized:
        return {"available": False, "reason": "BASELINE_PENDING"}
    try:
        mapped = map_input(frame, profile)
    except InputError as exc:
        return {"available": False, "reason": exc.code}
    sticks = {}
    for name in ("lx", "ly"):
        binding = getattr(profile.mapping, name)
        raw = frame.axes[binding.index]
        span = (
            binding.maximum - binding.center
            if raw >= binding.center
            else (binding.center - binding.minimum)
        )
        sticks[name] = {
            "index": binding.index,
            "raw": raw,
            "center": binding.center,
            "offset_raw": raw - binding.center,
            "normalized_before_deadzone": binding.sign * (raw - binding.center) / span,
            "mapped_after_curve": getattr(mapped, name),
        }
    linear = mapped.ly * profile.max_linear_mm_s
    rotation = mapped.lx * profile.max_rotation_deg_s
    return {
        "available": True,
        "interpretation_only": True,
        "sticks": sticks,
        "controls": asdict(mapped),
        "actions": actions(linear, rotation),
        "linear_mm_s_before_gate": linear,
        "rotation_deg_s_before_gate": rotation,
    }


class FeedbackRecord:
    """Displacements start at the first valid sample of each reference epoch.

    Each axis loses its baseline when its feedback/reference becomes invalid.
    Logging times are not sensor sampling times; the original stamp/sequence
    and feedback age are retained, including repeated cached states.
    """

    def __init__(self):
        self._lock = Lock()
        self._key = None
        self._linear = self._rotation = None

    def sample(self, state, backend):
        if state is None:
            return {"available": False, "reason": "NO_SYSTEM_STATE"}
        with self._lock:
            key = (state.boot_id, state.device_epoch, state.reference_id)
            if key != self._key:
                self._linear = self._rotation = None
                self._key = key
            linear = state.linear_position_m * 1000.0
            rotation = degrees(state.rotation_estimated_rad)
            linear_valid = state.linear_valid and state.linear_reference_valid
            rotation_valid = state.rotation_estimate_valid and state.rotation_reference_valid
            if not linear_valid:
                self._linear = None
            elif self._linear is None:
                self._linear = (state.state_seq, linear)
            if not rotation_valid:
                self._rotation = None
            elif self._rotation is None:
                self._rotation = (state.state_seq, rotation)
            return {
                "available": True,
                "backend": backend,
                "boot_id": state.boot_id,
                "device_epoch": state.device_epoch,
                "reference_id": state.reference_id,
                "state_seq": state.state_seq,
                "stamp_sec": state.stamp.sec,
                "stamp_nanosec": state.stamp.nanosec,
                "feedback_age_s": state.feedback_age_s,
                "device_state": state.state,
                "armed": state.armed,
                "linear_valid": linear_valid,
                "rotation_valid": rotation_valid,
                "linear_position_mm": linear,
                "rotation_estimated_deg": rotation,
                "linear_baseline": self._linear,
                "rotation_baseline": self._rotation,
                "linear_displacement_mm": linear - self._linear[1] if self._linear else None,
                "rotation_displacement_deg": rotation - self._rotation[1]
                if self._rotation
                else None,
                "linear_velocity_mm_s": state.linear_velocity_m_s * 1000.0,
                "linear_velocity_valid": state.linear_velocity_valid,
                "applied_linear_mm_s": state.applied_velocity_m_s * 1000.0,
                "estimated_rotation_deg_s": degrees(state.estimated_rotation_velocity_rad_s),
                "confirmed_pulses": state.confirmed_pulses,
                "reserved_pulses": state.reserved_pulses,
                "rotation_measured": False,
                "stop_progress": state.stop.progress,
                "stop_reason": state.stop.reason,
                "stop_serial": state.stop_serial,
                "fault_code": state.fault_code,
            }
