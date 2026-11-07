"""Explicit joydev hold policy without ROS, device access or motion."""

from dataclasses import replace

import pytest
from gca.domain.manual import QualityTracker
from gca_teleop.policy import TeleopPolicy
from gca_teleop.state_machine import TeleopPipeline
from test_input_pipeline import Frames, physical_evidence, synthetic

HELD = TeleopPolicy("servo", 9.847, manual_input_mode="joydev_hold")


def sample(pipeline, frames, **changes):
    frame = frames.frame(**changes)
    return pipeline.process(
        frame,
        frames.health(),
        frames.now,
        armed=True,
        reference_id="r",
        session_key="s",
        hardware=True,
    )


def setup_input(tmp_path, *, policy=HELD):
    profile, _, _ = physical_evidence(tmp_path)
    frames = Frames(real=True)
    frames.axes[2] = -32000
    return TeleopPipeline(profile, policy=policy), frames


def start(pipeline, frames):
    assert sample(pipeline, frames, real_event=False).gate == "READY"
    sample(pipeline, frames, axes={1: -12000})
    intent = sample(pipeline, frames, axes={2: 32767})
    assert intent.action == "JOG"
    return intent


def test_healthy_static_hold_continues_and_stick_controls_direction_and_magnitude(tmp_path):
    pipeline, frames = setup_input(tmp_path)
    first = start(pipeline, frames)
    for _ in range(10):
        current = sample(pipeline, frames, real_event=False, advance=0.04)
        assert current.action == "JOG"
        quality = current.quality
        assert quality.source_kind == "JOYDEV_HELD_STATE" and quality.link_evidence == "UNKNOWN"
        assert quality.qualification_id == "" and quality.episode_end_ns == 0
        assert (quality.event_ns, quality.event_seq) == (
            first.quality.event_ns,
            first.quality.event_seq,
        )
        assert quality.valid_until_ns == quality.observation_ns + 150_000_000
        repeated = pipeline.tick(
            frames.health(),
            frames.now + 1,
            armed=True,
            reference_id="r",
            session_key="s",
            hardware=True,
        )
        assert repeated.quality == quality
    assert frames.now > first.quality.valid_until_ns
    full_up = sample(pipeline, frames, axes={1: -32768})
    half_up = sample(pipeline, frames, axes={1: -16000})
    full_down = sample(pipeline, frames, axes={1: 32767})
    assert full_up.servo_rpm == -HELD.max_servo_rpm
    assert -HELD.max_servo_rpm < half_up.servo_rpm < 0
    assert full_down.servo_rpm == HELD.max_servo_rpm
    assert full_down.quality.event_seq > first.quality.event_seq


def test_reader_freeze_or_missing_fresh_frame_stops_and_mapping_remains_required(tmp_path):
    pipeline, frames = setup_input(tmp_path)
    first = start(pipeline, frames)
    stale = pipeline.tick(
        frames.health(),
        frames.now + 100_000_000,
        armed=True,
        reference_id="r",
        session_key="s",
        hardware=True,
    )
    assert stale.action == "STOP" and stale.reason == "READER_STALE"
    assert sample(pipeline, frames, real_event=False).action != "JOG"
    pipeline, frames = setup_input(tmp_path)
    first = start(pipeline, frames)
    expired_at = first.quality.valid_until_ns
    health_only = replace(frames.health(), reader_heartbeat_ns=expired_at)
    expired = pipeline.tick(
        health_only, expired_at, armed=True, reference_id="r", session_key="s", hardware=True
    )
    assert expired.action == "STOP" and expired.reason == "GCA_INPUT_EXPIRED"
    mismatched = replace(frames.health(), identity=replace(frames.identity, kernel="changed"))
    assert pipeline.hardware_arm_error(mismatched, frames.now) == "HARDWARE_INPUT_NOT_QUALIFIED"
    invalid = TeleopPipeline(
        replace(pipeline.config, mapping_evidence=str(tmp_path / "missing.json")), policy=HELD
    )
    assert sample(invalid, frames).action == "STOP"
    assert invalid.hardware_arm_error(frames.health(), frames.now) == "HARDWARE_INPUT_NOT_QUALIFIED"


@pytest.mark.parametrize("max_age_s", (0.12, 0.30))
def test_servo_input_age_from_shared_settings_bounds_both_arm_and_jog(tmp_path, max_age_s):
    policy = replace(HELD, manual_input_max_age_s=max_age_s)
    pipeline, frames = setup_input(tmp_path, policy=policy)
    sample(pipeline, frames, real_event=False)
    tracker = QualityTracker(backend="hardware", max_age_s=max_age_s, allow_joydev_hold=True)
    arm = pipeline.hardware_arm_quality(frames.health(), frames.now)
    assert pipeline.config.input_max_age_s == 0.15
    assert arm.valid_until_ns - arm.observation_ns == int(max_age_s * 1e9)
    assert pipeline.hardware_arm_quality(frames.health(), frames.now + 1) == arm
    assert tracker.validate(arm, frames.now) == arm.valid_until_ns
    jog = start(pipeline, frames).quality
    assert jog.valid_until_ns - jog.observation_ns == int(max_age_s * 1e9)
    assert tracker.validate(jog, frames.now) == jog.valid_until_ns
    later = frames.now + int(max_age_s * 0.75 * 1e9)
    repeated = pipeline.tick(
        replace(frames.health(), reader_heartbeat_ns=later),
        later,
        armed=True,
        reference_id="r",
        session_key="s",
        hardware=True,
    )
    assert repeated.action == "JOG" and repeated.quality == jog
    expired = pipeline.tick(
        replace(frames.health(), reader_heartbeat_ns=jog.valid_until_ns),
        jog.valid_until_ns,
        armed=True,
        reference_id="r",
        session_key="s",
        hardware=True,
    )
    assert expired.action == "STOP" and expired.reason == "GCA_INPUT_EXPIRED"


@pytest.mark.parametrize("stop", ("LT", "B", "reconnect"))
def test_stop_requires_release_center_and_a_new_physical_lt_edge(tmp_path, stop):
    pipeline, frames = setup_input(tmp_path)
    first = start(pipeline, frames)
    if stop == "LT":
        stopped = sample(pipeline, frames, axes={2: -32000})
        assert stopped.reason == "DEADMAN_RELEASED"
        assert sample(pipeline, frames, axes={2: 32767}).action != "JOG"
    elif stop == "B":
        stopped = sample(pipeline, frames, buttons={1: 1})
        assert stopped.reason == "B_STOP"
        assert sample(pipeline, frames, buttons={1: 0}).action != "JOG"
    else:
        frames.epoch = "reconnected"
        frames.seq, frames.raw = 0, 0
        stopped = sample(pipeline, frames, real_event=False)
        assert stopped.reason == "INPUT_RESYNC"
        assert sample(pipeline, frames, real_event=False).action != "JOG"
    assert stopped.action == "STOP"
    assert sample(pipeline, frames, axes={1: 0}).action != "JOG"
    assert sample(pipeline, frames, axes={2: -32000}).gate == "READY"
    renewed = sample(pipeline, frames, axes={1: -20000, 2: 32767})
    assert renewed.action == "JOG" and renewed.quality.episode_end_ns == 0
    assert renewed.quality.event_ns > first.quality.event_ns


def test_default_modes_stay_locked_and_explicit_hold_arm_requires_neutral(tmp_path):
    for policy in (TeleopPolicy("servo", 4.58), TeleopPolicy()):
        pipeline, frames = setup_input(tmp_path, policy=policy)
        assert sample(pipeline, frames).reason == "HARDWARE_INPUT_NOT_QUALIFIED"
    with pytest.raises(ValueError, match="JOYDEV_HOLD_REQUIRES_SERVO"):
        TeleopPolicy(manual_input_mode="joydev_hold")
    with pytest.raises(ValueError, match="JOYDEV_HOLD_REQUIRES_JOYDEV_EVENT_AGE"):
        TeleopPipeline(synthetic(), policy=HELD)
    pipeline, frames = setup_input(tmp_path)
    sample(pipeline, frames, real_event=False)
    neutral = pipeline.hardware_arm_quality(frames.health(), frames.now)
    assert neutral.event_seq == 0 and neutral.event_ns == 0
    assert neutral.source_kind == "JOYDEV_HELD_ARM"
    assert sample(pipeline, frames, real_event=False).action != "JOG"
    sample(pipeline, frames, axes={0: 20000})  # Unassigned LX does not inhibit ARM.
    proof = pipeline.hardware_arm_quality(frames.health(), frames.now)
    assert proof.source_kind == "JOYDEV_HELD_ARM" and proof.link_evidence == "UNKNOWN"
    assert proof.qualification_id == "" and proof.episode_end_ns == 0
    assert proof.valid_until_ns == proof.observation_ns + 150_000_000
    assert pipeline.hardware_arm_quality(frames.health(), frames.now + 1) == proof
    sample(pipeline, frames, axes={1: -20000})
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)
    sample(pipeline, frames, axes={1: 0, 2: 32767})
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)
    sample(pipeline, frames, axes={2: -32000}, buttons={1: 1})
    with pytest.raises(RuntimeError, match="ARM_REQUIRES_RELEASE_AND_CENTER"):
        pipeline.hardware_arm_quality(frames.health(), frames.now)


def barrier_sample(pipeline, frames, *, pending=True, epoch=1, session_key="s", **changes):
    frame = frames.frame(**changes)
    return pipeline.process(
        frame,
        frames.health(),
        frames.now,
        armed=True,
        ready=not pending,
        reference_id="r",
        session_key=session_key,
        hardware=True,
        stop_pending=pending,
        stop_epoch=epoch,
    )


def test_fast_lt_repress_counts_neutral_stop_frame_and_uses_current_demand(tmp_path):
    pipeline, frames = setup_input(tmp_path)
    start(pipeline, frames)
    stopped = sample(pipeline, frames, axes={1: 0, 2: -32000})
    assert stopped.action == "STOP" and stopped.reason == "DEADMAN_RELEASED"
    pipeline.bind_stop_epoch(1)
    # No intervening neutral poll: release+center was the STOP-producing frame.
    waiting = barrier_sample(pipeline, frames, axes={1: -20000, 2: 32767})
    assert waiting.action == "NONE" and waiting.gate == "WAIT_STOP"
    assert waiting.quality is None and waiting.servo_rpm == 0
    latest = barrier_sample(pipeline, frames, axes={1: 32767})
    assert latest.action == "NONE" and latest.quality is None
    resumed = barrier_sample(pipeline, frames, pending=False, real_event=False)
    assert resumed.action == "JOG" and resumed.servo_rpm == HELD.max_servo_rpm
    assert resumed.quality.observation_ns == frames.now


def test_b_release_alone_cannot_resume_but_reset_and_new_lt_survive_pending_stop(tmp_path):
    pipeline, frames = setup_input(tmp_path)
    start(pipeline, frames)
    assert sample(pipeline, frames, buttons={1: 1}).reason == "B_STOP"
    pipeline.bind_stop_epoch(1)
    assert barrier_sample(pipeline, frames, buttons={1: 0}).action != "JOG"
    assert barrier_sample(pipeline, frames, pending=False, real_event=False).action != "JOG"
    assert barrier_sample(pipeline, frames, axes={1: 0, 2: -32000}).gate == "READY"
    waiting = barrier_sample(pipeline, frames, axes={1: -32768, 2: 32767})
    assert waiting.gate == "WAIT_STOP" and waiting.action == "NONE"
    resumed = barrier_sample(pipeline, frames, pending=False, real_event=False)
    assert resumed.action == "JOG" and resumed.servo_rpm == -HELD.max_servo_rpm


@pytest.mark.parametrize("failure", ("release", "b", "identity", "epoch", "reader", "input"))
def test_waiting_lt_qualification_cannot_survive_release_stop_or_invalid_input(tmp_path, failure):
    pipeline, frames = setup_input(tmp_path)
    start(pipeline, frames)
    sample(pipeline, frames, axes={1: 0, 2: -32000})
    pipeline.bind_stop_epoch(1)
    assert barrier_sample(pipeline, frames, axes={1: -20000, 2: 32767}).gate == "WAIT_STOP"
    key, epoch = "s", 1
    if failure == "release":
        barrier_sample(pipeline, frames, axes={2: -32000})
    elif failure == "b":
        barrier_sample(pipeline, frames, buttons={1: 1})
        barrier_sample(pipeline, frames, buttons={1: 0})
    elif failure == "identity":
        key = "new-owner"
        barrier_sample(pipeline, frames, session_key=key, real_event=False)
    elif failure == "epoch":
        epoch = 2
        barrier_sample(pipeline, frames, epoch=epoch, real_event=False)
    else:
        when = frames.now + (100_000_000 if failure == "reader" else 150_000_000)
        health = frames.health()
        if failure == "input":
            health = replace(health, reader_heartbeat_ns=when)
        stopped = pipeline.tick(
            health,
            when,
            armed=True,
            ready=False,
            reference_id="r",
            session_key="s",
            hardware=True,
            stop_pending=True,
            stop_epoch=1,
        )
        assert stopped.action == "STOP"
        frames.now = when
    after = barrier_sample(
        pipeline, frames, pending=False, session_key=key, epoch=epoch, real_event=False
    )
    assert after.action != "JOG" and after.quality is None


def test_gate_snapshot_from_before_stop_cannot_admit_waiting_lt(tmp_path):
    pipeline, frames = setup_input(tmp_path)
    start(pipeline, frames)
    sample(pipeline, frames, axes={1: 0, 2: -32000})
    pipeline.bind_stop_epoch(1)
    waiting = barrier_sample(
        pipeline, frames, pending=False, epoch=0, axes={1: -20000, 2: 32767}
    )
    assert waiting.action == "NONE" and waiting.gate == "WAIT_STOP"
    assert barrier_sample(pipeline, frames, real_event=False).action == "NONE"
    assert barrier_sample(pipeline, frames, pending=False, real_event=False).action == "JOG"
