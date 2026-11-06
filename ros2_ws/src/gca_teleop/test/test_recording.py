"""Input offsets and feedback displacement must not imply executed motion."""

from math import radians

from gca_system_interfaces.msg import GcaState
from gca_teleop.recording import FeedbackRecord, actions, mapped_record
from test_input_pipeline import Frames, synthetic


def test_mapping_logs_offset_and_direction_without_lt_authorization():
    frames = Frames()
    result = mapped_record(frames.frame(axes={0: 16384, 1: -16384}), synthetic())
    assert result["available"] and result["interpretation_only"]
    assert result["sticks"]["ly"]["offset_raw"] == -16384
    assert result["sticks"]["ly"]["normalized_before_deadzone"] == 0.5
    assert 0 < result["sticks"]["ly"]["mapped_after_curve"] < 0.5
    assert result["controls"]["lt"] == 0
    assert result["actions"] == {"linear": "推进", "rotation": "正向旋转"}
    assert actions(0, 0) == {"linear": "零速", "rotation": "零速"}
    assert actions(-1, -1) == {"linear": "后退", "rotation": "反向旋转"}
    invalid = mapped_record(frames.frame(axes={0: 40000}), synthetic())
    assert invalid == {"available": False, "reason": "INVALID_AXIS_RANGE"}


def test_uninitialized_input_does_not_invent_zero_offsets():
    result = mapped_record(Frames().frame(initialized=False), synthetic())
    assert result == {"available": False, "reason": "BASELINE_PENDING"}


def test_feedback_baselines_reset_on_reference_change_and_invalid_feedback():
    tracker = FeedbackRecord()
    assert tracker.sample(None, "simulation")["available"] is False
    state = GcaState(
        boot_id="boot",
        device_epoch="device",
        reference_id="ref",
        state_seq=1,
        linear_valid=True,
        linear_reference_valid=True,
        rotation_estimate_valid=True,
        rotation_reference_valid=True,
        linear_position_m=0.002,
        rotation_estimated_rad=radians(10),
    )
    initial = tracker.sample(state, "simulation")
    assert initial["linear_displacement_mm"] == 0
    state.state_seq = 2
    state.linear_position_m = 0.005
    state.rotation_estimated_rad = radians(25)
    moved = tracker.sample(state, "simulation")
    assert moved["linear_displacement_mm"] == 3
    assert abs(moved["rotation_displacement_deg"] - 15) < 1e-10
    assert moved["rotation_measured"] is False
    assert moved["linear_baseline"] == (1, 2.0)
    assert tracker.sample(state, "simulation")["state_seq"] == 2
    state.rotation_reference_valid = False
    lost = tracker.sample(state, "simulation")
    assert lost["rotation_displacement_deg"] is None
    assert lost["linear_displacement_mm"] == 3
    state.rotation_reference_valid = True
    assert tracker.sample(state, "simulation")["rotation_displacement_deg"] == 0
    state.reference_id = "new-ref"
    assert tracker.sample(state, "simulation")["linear_displacement_mm"] == 0
