"""Real servo-only launch; input qualification and explicit ARM remain required."""

from system_bringup.servo_launch import description


def generate_launch_description():
    return description("hardware")
