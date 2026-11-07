"""Servo session capability/feedback gates and explicit shutdown sequence."""

import math
import time
from types import SimpleNamespace

import pytest
from gca_system_interfaces.msg import ServoState
from gca_system_interfaces.srv import ControlSession, ManageDevice
from gca_teleop.node import TeleopNode
from gca_teleop.policy import TeleopPolicy
from gca_teleop.recording import ServoFeedbackRecord
from gca_teleop.state_machine import TeleopPipeline
from test_input_pipeline import Frames, activate, process, qualified_profile, synthetic
from test_session_stop import reply_stop, session, state  # noqa: F401

# pytest resolves the imported fixture by its parameter name.
# ruff: noqa: F811


def servo_state(**kwargs):
    result = state(**kwargs)
    result.control_policy = "MANUAL_SERVO_VELOCITY"
    result.linear_valid = result.linear_reference_valid = False
    result.rotation_reference_valid = result.rotation_estimate_valid = False
    result.servo = ServoState(
        configured=True,
        valid=True,
        sample_seq=1,
        sampled_ns=time.monotonic_ns(),
        device_epoch="servo-device",
        raw_position_count=512,
        shaft_angle_rad=math.pi / 4,
        measured_velocity_rad_s=-0.2,
        requested_velocity_rad_s=-0.3,
        accepted_velocity_rad_s=-0.29,
    )
    return result


def use_servo(s):
    s.control_profile = "servo"
    s.servo_settings = SimpleNamespace(
        revision="revision",
        max_rpm=4.58,
        feedback_max_age_s=0.25,
        backend="simulation",
        manual_hold_enabled=False,
    )
    s._servo_capable = True
    s._receive_state(servo_state(sequence=11))


def test_servo_ready_uses_shaft_feedback_and_never_external_references(session):
    use_servo(session)
    assert session.latest_stop_confirmed
    initial = session._state
    session._state = None
    assert not session.latest_stop_confirmed
    session._state = initial
    assert session.gate()["ready"]
    invalid = servo_state(sequence=12)
    invalid.servo.valid = False
    session._receive_state(invalid)
    assert not session.gate()["ready"]
    delayed = servo_state(sequence=13)
    delayed.servo.sampled_ns -= 300_000_000
    session._receive_state(delayed)
    # Device validity retains its local 250 ms check; transport/cache time is
    # bounded independently by state progress at the teleop host.
    assert session.gate()["ready"]
    wrong = servo_state(sequence=14)
    wrong.control_policy = "POSITION_EXTERNAL_ENCODER"
    session._receive_state(wrong)
    assert not session.gate()["ready"]
    changed = servo_state(sequence=15)
    changed.applied_config_revision = "other-config"
    session._receive_state(changed)
    assert not session.gate()["ready"]


def test_servo_same_sequence_does_not_refresh_state_progress_watchdog(session, monkeypatch):
    use_servo(session)
    now = [time.monotonic_ns()]
    monkeypatch.setattr("gca_teleop.session.time.monotonic_ns", lambda: now[0])
    snapshot = servo_state(sequence=12)
    session._receive_state(snapshot)
    progressed_at = session._state_ns
    now[0] += 300_000_000
    session._receive_state(snapshot)
    assert session._state_ns == progressed_at and session.gate()["ready"]
    now[0] += 200_000_000
    session._receive_state(snapshot)
    assert not session.gate()["ready"] and session._state_ns == progressed_at
    session._receive_state(servo_state(sequence=13))
    assert session.gate()["ready"] and session._state_ns == now[0]


@pytest.mark.parametrize("late", (False, True))
def test_servo_stop_confirmation_deadline_includes_time_after_rpc_reply(session, monkeypatch, late):
    use_servo(session)
    now = [time.monotonic_ns()]
    monkeypatch.setattr("gca_teleop.session.time.monotonic_ns", lambda: now[0])
    receipt = session.request_stop("B_STOP")
    assert session.gate()["stop_pending"]
    assert session.gate()["stop_epoch"] == receipt["stop_epoch"]
    now[0] += 10_000_000
    reply_stop(session, servo_state(sequence=12, serial=6, progress="REQUESTED"))
    assert session._stop_future is None
    assert session.gate()["stop_pending"] and not session.gate()["ready"]
    now[0] += 40_000_000 if late else 10_000_000
    session._receive_state(servo_state(sequence=13, serial=6))
    if late:
        assert session.error == "STOP_CONFIRMATION_TIMEOUT"
        assert not session.latest_stop_confirmed
        assert not session.gate()["ready"] and not session.gate()["stop_pending"]
    else:
        assert not session.error and session.latest_stop_confirmed
        assert session.gate()["ready"] and not session.gate()["stop_pending"]


def test_fast_lt_waits_for_latest_stop_serial_without_sending_motion(session, tmp_path):
    from gca_teleop.dispatch import LatestDispatcher
    from test_held_servo_teleop import setup_input

    use_servo(session)
    session.timeout_s = 1.0
    pipeline, frames = setup_input(tmp_path)
    published = []
    dispatcher = LatestDispatcher(
        session, lambda *args: published.append(args), clock=lambda: frames.now
    )

    def input_frame(**changes):
        frame = frames.frame(**changes)
        intent = pipeline.process(frame, frames.health(), frames.now, **session.gate())
        dispatcher.offer(intent)
        dispatcher.pump()
        return intent

    # Start input locally, without dispatching the pre-STOP motion episode.
    frame = frames.frame(real_event=False)
    assert pipeline.process(frame, frames.health(), frames.now, **session.gate()).gate == "READY"
    frame = frames.frame(axes={1: -20000, 2: 32767})
    assert pipeline.process(frame, frames.health(), frames.now, **session.gate()).action == "JOG"
    stopped = input_frame(axes={1: 0, 2: -32000})
    assert stopped.reason == "DEADMAN_RELEASED"
    receipt = session.request_stop(stopped.reason)
    pipeline.bind_stop_epoch(receipt["stop_epoch"])
    assert input_frame(axes={1: 32767, 2: 32767}).gate == "WAIT_STOP"
    reply_stop(session, servo_state(sequence=12, serial=6, progress="REQUESTED"))
    session._receive_state(servo_state(sequence=13, serial=5))
    assert input_frame(real_event=False).action == "NONE"
    session._receive_state(servo_state(sequence=14, serial=6, progress="IN_PROGRESS"))
    assert input_frame(real_event=False).action == "NONE"
    assert session._clients["get_motion_permit"].requests == [] and published == []
    session._receive_state(servo_state(sequence=15, serial=6))
    resumed = input_frame(real_event=False)
    assert resumed.action == "JOG" and resumed.servo_rpm > 0
    assert len(session._clients["get_motion_permit"].requests) == 1
    assert published == []  # Still requires the independently issued motion permit.
    dispatcher.invalidate()


def test_servo_management_arm_does_not_request_dual_axis_reference(session):
    use_servo(session)
    session._receive_state(servo_state(sequence=12, name="DISARMED", armed=False))
    calls = []

    def call(name, request):
        calls.append(name)
        if name == "arm":
            assert request.axis_mask == 0
            return ManageDevice.Response(
                accepted=True, state=servo_state(sequence=13, generation=2)
            )
        assert name == "acknowledge_generation"
        return ControlSession.Response(accepted=True)

    session._call = call
    session.manage("arm")
    assert calls == ["arm", "acknowledge_generation"]
    with pytest.raises(ValueError, match="EXTERNAL_POSITION_CONTROL_UNAVAILABLE"):
        session.manage("reference")


def test_hardware_servo_arm_transmits_original_reader_proof(session, tmp_path):
    use_servo(session)
    session._backend = session.servo_settings.backend = "hardware"
    profile, _ = qualified_profile(tmp_path)
    frames = Frames(real=True)
    frames.axes[2] = -32000
    # A real monotonic timestamp is needed at the session boundary. The pure
    # fixture's usual time advance is unnecessary for this serialization case.
    pipeline = TeleopPipeline(profile, policy=TeleopPolicy("servo", 4.58))
    frames.now = time.monotonic_ns()
    process(pipeline, frames, advance=0)
    proof = pipeline.hardware_arm_quality(frames.health(), time.monotonic_ns())
    calls = []

    def call(name, request):
        calls.append(name)
        if name == "arm":
            assert request.has_input_quality
            assert request.input_quality.valid_until_ns == proof.valid_until_ns
            assert request.input_quality.observation_ns == proof.observation_ns
            assert request.input_quality.qualification_id == proof.qualification_id
            return ManageDevice.Response(
                accepted=True, state=servo_state(sequence=13, generation=2)
            )
        assert name == "acknowledge_generation"
        return ControlSession.Response(accepted=True)

    session._call = call
    session.manage("arm", input_quality=proof)
    assert calls == ["arm", "acknowledge_generation"]


def test_servo_close_confirms_stop_disarms_and_then_releases(session):
    use_servo(session)
    events = []
    original = session.request_stop

    def stop(reason):
        events.append("stop")
        receipt = original(reason)
        reply_stop(
            session, servo_state(sequence=20 + session.stop_epoch, serial=10 + session.stop_epoch)
        )
        return receipt

    def call(name, request):
        events.append(name)
        assert session.latest_stop_confirmed
        # The input producer may still report a held B/finished script while
        # release awaits disarm or its RPC. It must share the final STOP, not
        # create a new receipt that gets abandoned when ownership disappears.
        epoch = session.stop_epoch
        required = session._stop_required_epoch
        coalesced = original("SCRIPT_FINISHED")
        assert coalesced["coalesced_with_release"]
        assert session.stop_epoch == epoch and session._stop_required_epoch == required
        if name == "disarm":
            assert request.axis_mask == 0
            return ManageDevice.Response(
                accepted=True,
                state=servo_state(
                    sequence=30,
                    serial=20,
                    name="DISARMED",
                    armed=False,
                ),
            )
        assert name == "release_control" and not session.state.armed
        return ControlSession.Response(accepted=True)

    session.request_stop, session._call = stop, call
    assert session.close()
    assert events[-2:] == ["disarm", "release_control"]
    assert session.identity() == ("", 0)
    assert session.latest_stop_confirmed


def test_servo_publisher_uses_native_signed_radians_and_feedback_is_not_linear():
    policy = TeleopPolicy("servo", 4.58)
    frames = Frames()
    intent = activate(TeleopPipeline(synthetic(), policy=policy), frames, {1: -32768})
    # Fixture clock is intentionally independent; use a future fixed validity
    # only for this serialization test, never to claim runtime freshness.
    from dataclasses import replace

    intent = replace(
        intent, quality=replace(intent.quality, valid_until_ns=time.monotonic_ns() + 100_000_000)
    )
    messages = []
    node = SimpleNamespace(
        _policy=policy,
        _servo_settings=SimpleNamespace(output_ttl_s=0.30),
        _jog_pub=SimpleNamespace(publish=messages.append),
    )
    from gca_system_interfaces.msg import CommandContext

    TeleopNode._publish_jog(node, intent, CommandContext(), "test-nonce")
    assert len(messages) == 1
    assert math.isclose(messages[0].servo_angular_velocity_rad_s, -4.58 * 2 * math.pi / 60)
    sample = ServoFeedbackRecord().sample(servo_state(), "simulation")
    assert sample["linear_displacement_mm"] is sample["rotation_displacement_deg"] is None
    assert not sample["linear_valid"] and not sample["rotation_valid"]
    assert math.isclose(sample["measured_servo_rpm"], -0.2 * 60 / (2 * math.pi))


@pytest.mark.parametrize("control_profile,expected_ttl", (("servo", 0.30), ("dual_axis", 0.15)))
def test_published_jog_ttl_preserves_configured_limit_and_input_deadline(
    monkeypatch, control_profile, expected_ttl
):
    from dataclasses import replace

    from gca_system_interfaces.msg import CommandContext

    policy = TeleopPolicy("servo", 4.58) if control_profile == "servo" else TeleopPolicy()
    frames = Frames()
    intent = activate(TeleopPipeline(synthetic(), policy=policy), frames, {1: -32768})
    now = 1_000_000_000
    deadline = now + 300_000_000
    intent = replace(intent, quality=replace(intent.quality, valid_until_ns=deadline))
    monkeypatch.setattr("gca_teleop.node.time.monotonic_ns", lambda: now)
    messages = []
    node = SimpleNamespace(
        _policy=policy,
        _servo_settings=SimpleNamespace(output_ttl_s=0.30) if policy.is_servo else None,
        _jog_pub=SimpleNamespace(publish=messages.append),
    )
    TeleopNode._publish_jog(node, intent, CommandContext(), "test-nonce")
    assert messages[-1].ttl_s == pytest.approx(expected_ttl)
    now += 200_000_000
    TeleopNode._publish_jog(node, intent, CommandContext(), "test-nonce")
    assert messages[-1].ttl_s == pytest.approx(0.10)
    assert messages[-1].quality.valid_until_ns == deadline
    now = deadline
    with pytest.raises(ValueError, match="INPUT_EXPIRED_BEFORE_PUBLISH"):
        TeleopNode._publish_jog(node, intent, CommandContext(), "test-nonce")
    assert len(messages) == 2
