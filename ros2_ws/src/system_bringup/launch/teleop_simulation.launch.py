"""Joystick owner with simulated GCA; no automatic lifecycle, reference or ARM."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from gca_teleop.management import default_socket
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    config_path = Path(value("config_file"))
    if config_path.stat().st_size > 65536:
        raise ValueError("configuration exceeds size limit")
    configuration = yaml.safe_load(config_path.read_text())
    if (
        not isinstance(configuration, dict)
        or configuration.get("runtime", {}).get("backend") != "simulation"
    ):
        raise ValueError("teleop_simulation requires the simulation actuator backend")
    bringup = Path(get_package_share_directory("system_bringup"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / "launch/simulation.launch.py")),
            launch_arguments={"config_file": str(config_path)}.items(),
        ),
        Node(
            package="gca_teleop",
            executable="gca_teleop_node",
            name="gca_teleop",
            output="screen",
            parameters=[
                {
                    "profile_file": value("profile_file"),
                    "script_file": value("script_file"),
                    "socket_path": value("socket_path"),
                    "record_directory": value("record_directory"),
                }
            ],
        ),
    ]


def generate_launch_description():
    bringup = Path(get_package_share_directory("system_bringup"))
    teleop = Path(get_package_share_directory("gca_teleop"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value=str(bringup / "config/simulation.yaml")
            ),
            DeclareLaunchArgument(
                "profile_file", default_value=str(teleop / "config/beitong_unverified.yaml")
            ),
            DeclareLaunchArgument(
                "script_file", default_value="", description="Only for a scripted input profile"
            ),
            DeclareLaunchArgument("socket_path", default_value=str(default_socket())),
            DeclareLaunchArgument("record_directory", default_value=""),
            OpaqueFunction(function=_nodes),
        ]
    )
