"""One launch assembly for explicit servo backends; never automatic ARM."""

import secrets
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from gca.servo_config import load_servo_settings
from gca_teleop.management import default_socket
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .process_guard import fail_on_unexpected_exit


def description(backend):
    def nodes(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        path = value("servo_config_file")
        config = load_servo_settings(path)
        if config.backend != backend:
            raise ValueError(f"This launch requires a {backend} servo snapshot")
        trust = value("trusted_local_control").lower()
        if trust not in ("true", "false"):
            raise ValueError("trusted_local_control must be true or false")
        authority = secrets.token_urlsafe(32)
        records = value("record_directory")
        device_records = str(Path(records) / "device") if records else ""
        teleop_records = str(Path(records) / "teleop") if records else ""
        return [
            Node(
                package="gca_ros",
                executable="servo_control_node",
                output="screen",
                parameters=[
                    {
                        "servo_config_file": path,
                        "coordinator_key": authority,
                        "record_directory": device_records,
                        "trusted_local_control": trust == "true",
                    }
                ],
            ),
            Node(
                package="system_coordinator",
                executable="system_coordinator",
                output="screen",
                parameters=[
                    {"coordinator_key": authority, "profile": "servo", "servo_config_file": path}
                ],
            ),
            Node(
                package="gca_teleop",
                executable="gca_teleop_node",
                output="screen",
                parameters=[
                    {
                        "control_profile": "servo",
                        "servo_config_file": path,
                        "profile_file": value("profile_file"),
                        "script_file": value("script_file"),
                        "socket_path": value("socket_path"),
                        "record_directory": teleop_records,
                    }
                ],
            ),
        ]

    profile = str(
        Path(get_package_share_directory("gca_teleop")) / "config/beitong_unverified.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo_config_file", description="Required exported servo JSON snapshot"
            ),
            DeclareLaunchArgument("profile_file", default_value=profile),
            DeclareLaunchArgument("script_file", default_value=""),
            DeclareLaunchArgument("socket_path", default_value=str(default_socket())),
            DeclareLaunchArgument("record_directory", default_value=""),
            DeclareLaunchArgument("trusted_local_control", default_value="false"),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            fail_on_unexpected_exit(),
            OpaqueFunction(function=nodes),
        ]
    )
