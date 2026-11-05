"""T02/T06: plant physics and device accounting are independent of targets."""

from dataclasses import replace
from math import isclose

import pytest
from gca.domain.models import DeviceError, EncoderSample, PulseSegment
from gca.estimation.linear import LinearEstimator
from gca.simulation.devices import SimEncoder, SimPulseEngine, SimServo
from gca.simulation.plant import MechanicalPlant


class Clock:
    def __init__(self):
        self.now = 0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += round(seconds * 1e9)
        return self.now


def test_plant_only_moves_after_accepted_output_and_has_lag():
    clock = Clock()
    plant = MechanicalPlant(0)
    assert plant.advance(clock.advance(1)).position_mm == 0
    servo = SimServo(plant, clock)
    servo.open()
    servo.configure()
    servo.enable(True)
    servo.write_velocity(10, clock() + 10_000_000)
    state = plant.advance(clock.advance(0.04))
    assert 0 < state.velocity_mm_s < 10
    assert 0 < state.position_mm < 0.4
    servo.stop(clock() + 10_000_000)
    assert plant.snapshot().velocity_mm_s > 0  # zero request is not stopped evidence
    assert plant.advance(clock.advance(1)).velocity_mm_s < 0.001


def test_encoder_preserves_frozen_sample_seq_and_age():
    clock = Clock()
    plant = MechanicalPlant(0)
    encoder = SimEncoder(plant, clock=clock)
    first = encoder.read(1)
    encoder.frozen = True
    clock.advance(0.2)
    assert encoder.read(clock() + 1) == first


def test_estimation_repeated_samples_and_first_velocity_invalid():
    estimator = LinearEstimator(0.01, 100, 0.2)
    sample = EncoderSample(100, 1, 1_000_000_000, "epoch")
    first = estimator.update(sample)
    assert not first.velocity_valid
    estimator.set_reference(0)
    same = estimator.update(sample)
    assert same.position_mm == 0 and same.received_ns == first.received_ns
    next_state = estimator.update(replace(sample, raw_count=110, seq=2, received_ns=1_100_000_000))
    assert isclose(next_state.velocity_mm_s, 1.0)
    assert next_state.referenced
    restarted = estimator.update(replace(sample, seq=1, device_epoch="new"))
    assert not restarted.referenced and not restarted.velocity_valid


def test_estimator_bad_jump_does_not_refresh_age_and_wrap_is_explicit():
    estimator = LinearEstimator(1, 10, 1, modulus=256)
    estimator.update(EncoderSample(255, 1, 1_000_000_000, "e"))
    wrapped = estimator.update(EncoderSample(0, 2, 1_200_000_000, "e"))
    assert wrapped.position_mm == 256
    bad = estimator.update(EncoderSample(100, 3, 1_300_000_000, "e"))
    assert not bad.valid and bad.received_ns == wrapped.received_ns


def test_pulse_acceptance_is_not_completion_and_abort_is_unknown():
    clock = Clock()
    plant = MechanicalPlant(0, degrees_per_pulse=1)
    pulse = SimPulseEngine(plant, clock)
    identity = pulse.open()
    segment = PulseSegment("seg", 100, 1, 1, identity.device_epoch)
    pulse.submit(segment)
    assert pulse.poll() == [] and plant.snapshot().mechanical_angle_deg == 0
    clock.advance(0.5)
    result = pulse.abort()
    assert result.confirmed_pulses is None and result.idle
    assert plant.snapshot().mechanical_angle_deg == 50
    assert pulse.poll() == []


def test_open_loop_can_miss_steps_without_fake_feedback():
    clock = Clock()
    plant = MechanicalPlant(0, degrees_per_pulse=1)
    plant.missed_step_fraction = 0.25
    pulse = SimPulseEngine(plant, clock)
    identity = pulse.open()
    pulse.submit(PulseSegment("seg", 100, 1, 1, identity.device_epoch))
    clock.advance(1)
    complete = pulse.poll()
    assert complete[0].signed_pulses == 100
    assert plant.snapshot().mechanical_angle_deg == 75
    assert pulse.poll() == []


def test_pulse_submit_rechecks_remaining_finite_output_window():
    clock = Clock()
    plant = MechanicalPlant(0, degrees_per_pulse=1)
    pulse = SimPulseEngine(plant, clock)
    identity = pulse.open()
    planned = PulseSegment("delayed", 100, 0.03, 1, identity.device_epoch, expires_ns=100_000_000)
    clock.advance(0.08)  # Earlier planning passed; the actual submit is late.
    with pytest.raises(DeviceError) as caught:
        pulse.submit(planned)
    assert caught.value.code == "PULSE_EXPIRED"
    assert pulse.is_idle()
    assert plant.snapshot().mechanical_angle_deg == 0
    pulse.submit(replace(planned, expires_ns=110_000_000))  # Exact fit is valid.
    clock.advance(0.03)
    assert pulse.poll()[0].signed_pulses == 100
