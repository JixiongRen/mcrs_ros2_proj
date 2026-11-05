"""Require an explicit hardware configuration; startup still does not ARM."""

import os
import secrets
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from system_bringup.process_guard import fail_on_unexpected_exit


def _nodes(context):
    config = LaunchConfiguration("config_file").perform(context)
    if not config:
        raise RuntimeError("hardware launch requires config_file; use a verified device profile")
    with Path(config).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or document.get("runtime", {}).get("backend") != "hardware":
        raise RuntimeError("hardware launch refuses a non-hardware profile")
    authority = secrets.token_urlsafe(32)
    trust = LaunchConfiguration("trusted_local_control").perform(context).lower()
    if trust not in ("true", "false"):
        raise RuntimeError("trusted_local_control must be true or false")
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
                {
                    "config_file": config,
                    "coordinator_key": authority,
                    "record_directory": records,
                    "trusted_local_control": trust == "true",
                }
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Required verified hardware YAML; never a simulation fallback",
            ),
            DeclareLaunchArgument(
                "trusted_local_control",
                default_value="false",
                description="Explicitly trust local ROS clients; otherwise require Enforce DDS security",
            ),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            fail_on_unexpected_exit(),
            OpaqueFunction(function=_nodes),
        ]
    )
