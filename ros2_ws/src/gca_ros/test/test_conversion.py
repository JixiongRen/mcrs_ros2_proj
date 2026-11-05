"""Unit conversion uses generated ROS messages, not a mocked ROS API."""

import math

from gca.domain.models import RuntimeState
from gca_ros.conversion import motion_from_ros, state_from_core
from gca_system_interfaces.action import ExecuteMotion


def test_goal_si_boundary_and_invalid_feedback_are_not_confused():
    goal = ExecuteMotion.Goal()
    goal.target.linear_m = 0.023
    goal.target.rotation_rad = math.pi / 2
    goal.limits.max_linear_m_s = 0.004
    goal.limits.max_linear_m_s2 = 0.02
    goal.limits.max_rotation_rad_s = math.pi
    goal.limits.max_rotation_rad_s2 = math.pi * 2
    command = motion_from_ros(goal)
    assert command.linear_mm == 23.0
    assert command.rotation_deg == 90.0
    assert command.limits.max_linear_mm_s == 4.0
    assert command.limits.max_rotation_deg_s == 180.0
    state = RuntimeState(
        position_mm=17.0,
        linear_valid=False,
        velocity_valid=False,
        rotation_deg=45.0,
        rotation_estimate_valid=False,
    )
    message = state_from_core(state)
    assert message.linear_position_m == 0.017
    assert not message.linear_valid and not message.linear_velocity_valid
    assert not message.rotation_estimate_valid
    assert message.rotation_estimated_rad == math.pi / 4
