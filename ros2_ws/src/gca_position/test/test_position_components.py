"""Small behavior-focused checks; no ROS or hardware connections."""

import json
import math
import runpy
from dataclasses import replace
from pathlib import Path
from threading import Event

import pytest
from gca.domain.models import DeviceError, EncoderSample
from gca.servo_config import (
    make_position_settings,
    make_servo_settings,
    servo_settings_document,
    servo_settings_from_document,
)
from gca_position.acquisition import EncoderAcquisition
from gca_position.assembly import create_position_components
from gca_position.feedback import EncoderFeedback

ROOT = Path(__file__).resolve().parents[4]


def settings():
    servo = runpy.run_path(str(ROOT / "config/servo_settings.py"))
    encoder = runpy.run_path(str(ROOT / "config/encoder_settings.py"))
    return make_position_settings(
        servo["settings_payload"]("simulation"),
        servo["position_settings_payload"](),
        encoder["settings_payload"](),
    )


def test_strict_position_snapshot_and_legacy_manual_roundtrip():
    config = settings()
    document = json.loads(json.dumps(servo_settings_document(config)))
    assert servo_settings_from_document(document) == config
    assert (
        config.control_mode == "position" and config.control_policy == "POSITION_EXTERNAL_ENCODER"
    )
    assert (
        config.position.motion.target_min_mm == -40 and config.position.motion.target_max_mm == 990
    )
    assert config.position.effective_braking.deceleration_source == "default"
    assert config.position.motion.braking_delay_s is None
    assert config.position.encoder.mm_per_count == pytest.approx(2 * math.pi * 10.89 / 4096)
    assert config.position.mechanics.mapping.velocity_to_rpm(2) < 0
    document["position"]["encoder"]["feedback_max_age_s"] = 1.0
    with pytest.raises(ValueError, match="digest"):
        servo_settings_from_document(document)
    source = runpy.run_path(str(ROOT / "config/servo_settings.py"))["settings_payload"](
        "simulation"
    )
    legacy = make_servo_settings(source)
    assert legacy.control_mode == "manual" and legacy.position is None
    assert servo_settings_from_document(servo_settings_document(legacy)) == legacy
    payload = servo_settings_document(config)
    payload.pop("sha256")
    payload["control_mode"] = "manual"
    with pytest.raises(ValueError, match="manual mode"):
        make_servo_settings(payload)
    payload["control_mode"] = "position"
    payload["position"]["encoder"]["signed"] = 0
    with pytest.raises(ValueError, match="unsigned"):
        make_servo_settings(payload)


@pytest.mark.parametrize("direction", [1, -1])
def test_reference_direction_wrap_noise_window_and_invalidation(direction):
    cfg = replace(settings().position.encoder, direction_sign=direction)
    observer = EncoderFeedback(cfg)
    seq, now = 0, 1_000_000_000
    raw = cfg.count_modulus - 2

    def sample(value, *, epoch="one", valid=True):
        nonlocal seq, now
        seq, now = seq + 1, now + 20_000_000
        return observer.update(
            EncoderSample(value, seq, now, epoch, valid=valid, request_started_ns=now - 1_000_000),
            now,
        )

    for _ in range(12):
        result = sample(raw)
    assert not result.position_valid and result.velocity_valid
    reference = observer.set_reference(7.0, now)
    assert reference and not observer.snapshot(now).velocity_valid
    for _ in range(12):
        result = sample(raw)
    assert result.position_mm == pytest.approx(7.0) and result.velocity_valid
    result = sample(1)
    assert result.position_mm == pytest.approx(7.0 + direction * 3 * cfg.mm_per_count)
    result = sample(raw)
    assert result.position_mm == pytest.approx(7.0)
    # One-count static noise must not permanently prevent the 0.1 mm/s criterion.
    for index in range(30):
        result = sample(raw + index % 2)
    assert result.velocity_valid and abs(result.velocity_mm_s) <= 0.1
    duplicate = observer.update(observer._sample, now)
    assert duplicate == result
    assert not observer.snapshot(now + 101_000_000).position_valid
    result = sample(raw, epoch="reconnected")
    assert not result.position_valid and not result.velocity_valid and not result.reference_id
    assert result.error_code == "ENCODER_EPOCH_CHANGED"
    result = sample(raw, epoch="reconnected", valid=False)
    assert not result.position_valid and result.error_code


def test_acquisition_reports_failure_without_refreshing_cached_sample():
    cfg = settings().position.encoder
    failure = Event()

    class Reader:
        closed = False
        reads = 0

        def open(self):
            pass

        def read(self, deadline_ns):
            self.reads += 1
            if self.reads > 1:
                raise DeviceError("ENCODER_TEST_FAILURE")
            return EncoderSample(123, 1, deadline_ns - 50_000_000, "test")

        def close(self):
            self.closed = True

    reader = Reader()
    acquisition = EncoderAcquisition(reader, cfg)
    try:
        assert acquisition.start(lambda reason: failure.set())
        assert failure.wait(0.5)
        snapshot = acquisition.snapshot()
        assert not snapshot.healthy and snapshot.error_code == "ENCODER_TEST_FAILURE"
        assert snapshot.sample.seq == 1
    finally:
        acquisition.close()
    assert reader.closed


def test_strategy_drives_actual_accepted_simulated_output_with_slip_and_bounds():
    config = settings()
    now = [1_000_000_000]

    def clock():
        return now[0]

    drive, components = create_position_components(config, clock=clock, slip_ratio=0.8)
    reader, feedback, strategy = (
        components.acquisition.reader,
        components.feedback,
        components.strategy,
    )
    drive.open_identified()
    drive.configure_velocity_mode()
    reader.open()

    def observe():
        now[0] += 20_000_000
        return feedback.update(reader.read(now[0] + 50_000_000), now[0])

    try:
        for _ in range(12):
            observed = observe()
        assert feedback.set_reference(0.0, now[0])
        for _ in range(12):
            observed = observe()
        with pytest.raises(DeviceError):
            strategy.start(991.0, 2.0, 5.0, observed, now[0])
        strategy.start(5.0, 2.0, 5.0, observed, now[0])
        drive.set_torque(True)
        seq, last = 0, None
        for index in range(500):
            observed = observe()
            if index % 2:
                continue
            seq += 1
            output = strategy.step(observed, now[0], seq, 1)
            assert output is not None
            assert strategy.step(observed, now[0], seq + 1, 1) is None
            accepted = drive.write_velocity_rpm(output.rpm, now[0] + 80_000_000)
            assert abs(config.position.mechanics.mapping.rpm_to_velocity(accepted)) <= 2.0
            strategy.accepted(seq, 1, accepted)
            last = output
            if output.complete:
                break
        assert last is not None and last.complete
        assert strategy.at_target(observed, 5.0)
        assert 4.0 <= drive.encoder_position_mm() <= 6.0
        assert observed.position_mm == pytest.approx(
            drive.encoder_position_mm(), abs=config.position.encoder.mm_per_count
        )
        strategy.stop()
        assert strategy.step(observed, now[0], seq + 1, 1) is None
    finally:
        reader.close()
        drive.stop(now[0] + 80_000_000)
        drive.close()
