"""Servo-only input contracts; no actuator or ROS side effects."""

import pytest
from gca.domain.manual import QualityTracker
from gca_teleop.policy import TeleopPolicy
from gca_teleop.recording import intent_actions, mapped_record
from gca_teleop.state_machine import TeleopPipeline
from test_input_pipeline import (
    Frames,
    activate,
    physical_evidence,
    process,
    qualified_profile,
    synthetic,
)

POLICY = TeleopPolicy("servo", 4.58)


def test_servo_mapping_direction_magnitude_and_unassigned_lx_neutral():
    frames = Frames()
    pipeline = TeleopPipeline(synthetic(), policy=POLICY)
    # A deflected unused LX must not prevent observing neutral LY/released LT.
    assert process(pipeline, frames, axes={0: 32767}).gate == "READY"
    half = process(pipeline, frames, axes={1: -16384, 2: 32767})
    assert half.action == "JOG" and -4.58 < half.servo_rpm < 0
    full = process(pipeline, frames, axes={1: -32768})
    assert full.servo_rpm == -4.58
    assert full.linear_mm_s == full.rotation_deg_s == 0
    assert full.control_policy == "MANUAL_SERVO_VELOCITY"
    assert intent_actions(full) == {"servo": "反向旋转"}
    down = process(pipeline, frames, axes={1: 32767})
    assert down.servo_rpm == 4.58
    centered = process(pipeline, frames, axes={1: 0})
    assert centered.action == "JOG" and centered.servo_rpm == 0


def test_servo_lx_and_reserved_inputs_cannot_extend_real_event_deadline(tmp_path):
    profile, _, _ = physical_evidence(tmp_path)
    frames = Frames(real=True)
    frames.axes[2] = -32000
    pipeline = TeleopPipeline(profile, policy=POLICY)
    active = activate(pipeline, frames, {1: -20000})
    for axis in (0, 3, 4, 5):
        changed = process(pipeline, frames, axes={axis: 20000})
        assert changed.servo_rpm == active.servo_rpm
        assert changed.quality.valid_until_ns == active.quality.valid_until_ns
    frames.now = active.quality.valid_until_ns
    assert (
        pipeline.tick(
            frames.health(), frames.now, armed=True, reference_id="r1", session_key="s1"
        ).reason
        == "GCA_INPUT_EXPIRED"
    )


@pytest.mark.parametrize("stop", ("LT", "B"))
def test_servo_stop_requires_real_release_neutral_and_new_lt(stop):
    frames = Frames()
    pipeline = TeleopPipeline(synthetic(), policy=POLICY)
    activate(pipeline, frames, {1: -20000})
    if stop == "LT":
        result = process(pipeline, frames, axes={2: -32768})
        assert result.reason == "DEADMAN_RELEASED"
        process(pipeline, frames, axes={2: 32767})
    else:
        result = process(pipeline, frames, buttons={1: 1})
        assert result.reason == "B_STOP"
        process(pipeline, frames, buttons={1: 0})
    assert result.action == "STOP" and result.servo_rpm == 0
    assert process(pipeline, frames, axes={1: 0}).action != "JOG"
    assert process(pipeline, frames, axes={0: 32767, 2: -32768}).gate == "READY"
    assert process(pipeline, frames, axes={1: -20000, 2: 32767}).action == "JOG"


def test_servo_hardware_input_gate_and_logs_do_not_invent_displacement():
    frames = Frames()
    pipeline = TeleopPipeline(synthetic(), policy=POLICY)
    activate(pipeline, frames, {1: -20000})
    assert (
        pipeline.hardware_arm_error(frames.health(), frames.now) == "HARDWARE_INPUT_NOT_QUALIFIED"
    )
    row = mapped_record(frames.frame({1: -32768}), synthetic(), policy=POLICY)
    assert row["servo_rpm_before_gate"] == -4.58
    assert row["linear_mm_s_before_gate"] is None
    assert row["rotation_deg_s_before_gate"] is None
    assert row["sticks"]["ly"]["raw"] == -32768
    assert row["actions"] == {"servo": "反向旋转"}


def test_servo_policy_rejects_ambiguous_profile_or_direction():
    with pytest.raises(ValueError, match="UNKNOWN_CONTROL_PROFILE"):
        TeleopPolicy("unknown")
    with pytest.raises(ValueError, match="INVALID_SERVO_SPEED_LIMIT"):
        TeleopPolicy("servo", float("nan"))
    with pytest.raises(ValueError, match="DIRECTION"):
        TeleopPolicy("servo", 4.58, 1)


def test_hardware_arm_proof_is_neutral_qualified_and_cannot_refresh_cached_input(tmp_path):
    profile, _ = qualified_profile(tmp_path)
    frames = Frames(real=True)
    frames.axes[2] = -32000
    pipeline = TeleopPipeline(
        profile, policy=TeleopPolicy("servo", 4.58, manual_input_max_age_s=0.30)
    )
    process(pipeline, frames, axes={0: 32767})  # Unassigned LX does not block ARM.
    proof = pipeline.hardware_arm_quality(frames.health(), frames.now)
    assert proof.source_kind == "QUALIFIED_STATE_CONTRACT"
    assert proof.qualification_id
    assert proof.valid_until_ns == frames.now + int(profile.input_max_age_s * 1e9)
    assert pipeline.hardware_arm_quality(frames.health(), frames.now + 1) == proof
    assert QualityTracker(backend="hardware").validate(proof, frames.now)
    process(pipeline, frames, axes={1: -20000})
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)
    jog = process(pipeline, frames, axes={1: 0, 2: 32767})
    assert jog.quality.valid_until_ns == jog.quality.observation_ns + int(
        profile.input_max_age_s * 1e9
    )
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)
    process(pipeline, frames, axes={2: -32000}, buttons={1: 1})
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)
    process(pipeline, frames, buttons={1: 0})
    with pytest.raises(RuntimeError, match="READER_STALE"):
        pipeline.hardware_arm_quality(frames.health(), frames.now + 100_000_000)


def test_servo_persistent_stop_is_not_reemitted_by_its_readiness_feedback():
    frames = Frames()
    pipeline = TeleopPipeline(synthetic(), policy=POLICY)
    activate(pipeline, frames, {1: -20000})
    first = process(pipeline, frames, buttons={1: 1})
    assert first.action == "STOP" and first.reason == "B_STOP"
    for ready in (False, True, False, True):
        pipeline.tick(
            frames.health(),
            frames.now,
            armed=True,
            ready=ready,
            reference_id="r1",
            session_key="s1",
        )
        held = pipeline.process(
            frames.frame(),
            frames.health(),
            frames.now,
            armed=True,
            ready=ready,
            reference_id="r1",
            session_key="s1",
        )
        assert held.action == "NONE" and held.reason == "B_STOP"
    process(pipeline, frames, buttons={1: 0})
    pressed_again = process(pipeline, frames, buttons={1: 1})
    assert pressed_again.action == "STOP" and pressed_again.reason == "B_STOP"
    # A fresh ARM cycle must still stop a B that was already held.
    pipeline.tick(frames.health(), frames.now, armed=False, reference_id="r1", session_key="s1")
    assert process(pipeline, frames).action == "STOP"


def test_servo_finished_script_stops_once_across_stopping_and_ready_states():
    frames = Frames()
    pipeline = TeleopPipeline(synthetic(), policy=POLICY)
    activate(pipeline, frames, {1: -20000})
    health = frames.health(reason="SCRIPT_FINISHED")
    first = pipeline.tick(health, frames.now, armed=True, reference_id="r1", session_key="s1")
    assert first.action == "STOP" and first.reason == "SCRIPT_FINISHED"
    for ready in (False, True, False, True):
        pipeline.tick(
            health, frames.now, armed=True, ready=ready, reference_id="r1", session_key="s1"
        )
        repeated = pipeline.tick(
            health, frames.now, armed=True, ready=ready, reference_id="r1", session_key="s1"
        )
        assert repeated.action == "NONE" and repeated.reason == "SCRIPT_FINISHED"
