"""Explicit servo-only simulation with the ordinary joystick owner."""

from system_bringup.servo_launch import description


def generate_launch_description():
    return description("simulation")
