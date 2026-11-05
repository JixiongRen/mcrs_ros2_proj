"""Start the two real ROS processes with simulated devices, initially unarmed."""

import os
import secrets
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from system_bringup.process_guard import fail_on_unexpected_exit


def _nodes(context):
    # A fresh device-management capability is shared only by this launch's nodes.
    # It does not replace host/DDS access control for an untrusted network.
    authority = secrets.token_urlsafe(32)
    config = LaunchConfiguration("config_file").perform(context)
    records = str(
        Path(os.environ.get("ROS_LOG_DIR", str(Path.home() / ".ros/log")))
        / ("gca-records-" + secrets.token_hex(4))
    )
    return [
        Node(
            package="gca_ros",
            executable="gca_control_node",
            name="gca_control_node",
            output="screen",
            parameters=[
                {"config_file": config, "coordinator_key": authority, "record_directory": records}
            ],
        ),
        Node(
            package="system_coordinator",
            executable="system_coordinator",
            name="system_coordinator",
            output="screen",
            parameters=[{"coordinator_key": authority}],
        ),
    ]


def generate_launch_description():
    default = str(
        Path(get_package_share_directory("system_bringup")) / "config" / "simulation.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default,
                description="Complete GCA configuration, simulation by default",
            ),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            fail_on_unexpected_exit(),
            OpaqueFunction(function=_nodes),
        ]
    )
