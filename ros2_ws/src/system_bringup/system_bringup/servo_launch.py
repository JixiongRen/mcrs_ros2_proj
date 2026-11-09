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


def description(backend=None):
    """Legacy backend wrappers select manual; unified entry requires both selectors."""

    def nodes(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        path = value("servo_config_file")
        config = load_servo_settings(path)
        selected_backend = backend if backend is not None else value("backend")
        selected_mode = "manual" if backend is not None else value("control_mode")
        if selected_backend not in ("simulation", "hardware"):
            raise ValueError("backend must be simulation or hardware")
        if selected_mode not in ("manual", "position"):
            raise ValueError("control_mode must be manual or position")
        if config.backend != selected_backend or config.control_mode != selected_mode:
            raise ValueError("Launch mode/backend must match the exported servo snapshot")
        discovery = value("discovery_range")
        if discovery not in ("LOCALHOST", "SUBNET"):
            raise ValueError("discovery_range must be LOCALHOST or SUBNET")
        slip = float(value("simulation_slip_ratio"))
        if not 0 < slip <= 1 or (
            (selected_backend != "simulation" or selected_mode != "position") and slip != 1
        ):
            raise ValueError("simulation_slip_ratio differs from 1 only for position simulation")
        trust = value("trusted_local_control").lower()
        if trust not in ("true", "false"):
            raise ValueError("trusted_local_control must be true or false")
        authority = secrets.token_urlsafe(32)
        records = value("record_directory")
        device_records = str(Path(records) / "device") if records else ""
        teleop_records = str(Path(records) / "teleop") if records else ""
        actions = [
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", discovery),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1" if discovery == "LOCALHOST" else "0"),
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
                        "control_mode": selected_mode,
                        "simulation_slip_ratio": slip,
                    }
                ],
            ),
            Node(
                package="system_coordinator",
                executable="system_coordinator",
                output="screen",
                parameters=[
                    {
                        "coordinator_key": authority,
                        "profile": "servo",
                        "servo_config_file": path,
                        "control_mode": selected_mode,
                    }
                ],
            ),
        ]
        if selected_mode == "position":
            return actions
        actions.append(
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
            )
        )
        return actions

    profile = str(
        Path(get_package_share_directory("gca_teleop")) / "config/beitong_unverified.yaml"
    )
    selectors = (
        []
        if backend is not None
        else [
            DeclareLaunchArgument("control_mode", description="Required: manual or position"),
            DeclareLaunchArgument("backend", description="Required: simulation or hardware"),
        ]
    )
    return LaunchDescription(
        selectors
        + [
            DeclareLaunchArgument(
                "servo_config_file", description="Required exported servo JSON snapshot"
            ),
            DeclareLaunchArgument("profile_file", default_value=profile),
            DeclareLaunchArgument("script_file", default_value=""),
            DeclareLaunchArgument("socket_path", default_value=str(default_socket())),
            DeclareLaunchArgument("record_directory", default_value=""),
            DeclareLaunchArgument("trusted_local_control", default_value="false"),
            DeclareLaunchArgument("discovery_range", default_value="LOCALHOST"),
            DeclareLaunchArgument("simulation_slip_ratio", default_value="1.0"),
            fail_on_unexpected_exit(),
            OpaqueFunction(function=nodes),
        ]
    )
