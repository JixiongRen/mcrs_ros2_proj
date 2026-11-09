"""Explicit manual or position assembly; startup never claims or arms."""

from system_bringup.servo_launch import description


def generate_launch_description():
    return description()
