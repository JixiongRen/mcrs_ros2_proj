"""T01/T15: analytical expectations independent from controller implementation."""

from math import isclose

import pytest
from gca.algorithms.control import PositionController, boundary_velocity, slew
from gca.algorithms.trajectory import PulseQuantizer, Trapezoid
from gca.config import RuntimeConfig


def test_trapezoid_respects_limits_and_rest_endpoints():
    profile = Trapezoid(-2.0, 8.0, 4.0, 8.0, 0)
    values = [profile.at(i * 10_000_000) for i in range(401)]
    assert values[0].position == -2 and values[0].velocity == 0
    assert values[-1].complete and values[-1].position == 8
    assert max(v.velocity for v in values) <= 4
    assert all(abs(b.velocity - a.velocity) <= 0.08000001 for a, b in zip(values, values[1:]))


def test_triangular_reverse_and_zero_trajectory():
    reverse = Trapezoid(1.0, -1.0, 20.0, 2.0, 0)
    assert reverse.at(500_000_000).velocity < 0
    assert reverse.at(4_000_000_000).position == -1
    assert Trapezoid(1, 1, 1, 1, 0).at(0).complete


def test_pid_true_dt_and_quantization_residual_settled_once():
    controller = PositionController(kp=0, ki=2, kaw=1)
    controller.commit(1, 3, error=1, raw=0, limited=0, dt=0.1)
    assert isclose(controller.integral, 0.2)
    controller.commit(1, 3, error=100, raw=0, limited=0, dt=0.1)
    assert isclose(controller.integral, 0.2)
    assert controller.applied(1, 3, 0.5)
    assert isclose(controller.integral, 0.25)
    assert not controller.applied(1, 3, 0.5)
    assert not controller.applied(2, 2, 0.5)
    with pytest.raises(ValueError):
        controller.commit(2, 3, 1, 0, 0, 0)


def test_p_only_does_not_create_hidden_integral_from_limiting():
    controller = PositionController(ki=0)
    controller.commit(1, 1, 1, 20, 1, 0.1)
    assert controller.integral == 0


def test_saturation_reversal_and_reset():
    controller = PositionController(kp=1, ki=1, kaw=3, integral_limit=2)
    for index in range(100):
        controller.commit(index, 1, 1, 100, 1, 0.01)
    assert -2 <= controller.integral <= 2
    assert controller.request(0, 0, 5, 0).requested < 0
    controller.reset()
    assert controller.integral == 0


def test_boundary_braking_includes_delay_and_output_dt():
    limited = boundary_velocity(20, 9, 0, 10, 10, delay=0.1)
    assert limited * 0.1 + limited**2 / 20 <= 1.0000001
    assert boundary_velocity(20, 10, 0, 10, 10) == 0
    assert slew(10, 0, 4, 0.25) == 1


def test_absolute_pulse_rounding_preserves_small_moves():
    quantizer = PulseQuantizer(360, 1)
    assert [quantizer.pulses(i * 0.1) for i in range(11)][-1] == 1
    assert quantizer.pulses(-1.0) == -1
    assert quantizer.angle(1) == 1


@pytest.mark.parametrize(
    "change",
    [
        {"queue_capacity": True},
        {"queue_capacity": 1.5},
        {"kp": "3"},
        {"kp": float("nan")},
        {"backend": "typo"},
        {"hardware_verified": 1},
        {"encoder_count_modulus": 1},
        {"backend": "hardware", "hardware_verified": True},
    ],
)
def test_strict_configuration_rejects_ambiguous_values(change):
    with pytest.raises(ValueError):
        RuntimeConfig.from_dict(change)
