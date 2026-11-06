#!/usr/bin/env python3
"""Check an ordinary install from outside the source tree, without PYTHONPATH edits."""

import importlib
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    for name in (
        "gca",
        "gca_ros",
        "system_coordinator",
        "system_bringup",
        "gca_system_interfaces",
        "gca_teleop",
    ):
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve()
        assert "/src/" not in str(path), f"Source import masks broken installation: {path}"
        print(f"{name}: {path}")
    for package, executable in (
        ("gca_ros", "gca_cli"),
        ("gca_ros", "gca_control_node"),
        ("system_coordinator", "system_coordinator"),
        ("gca_teleop", "gca_teleop_node"),
        ("gca_teleop", "gca_teleop_probe"),
        ("gca_teleop", "gca_teleop_cli"),
        ("gca_teleop", "gca_joystick_monitor"),
    ):
        assert (Path(get_package_prefix(package)) / "lib" / package / executable).is_file()
    share = Path(get_package_share_directory("system_bringup"))
    for relative in (
        "launch/simulation.launch.py",
        "launch/hardware.launch.py",
        "launch/teleop_simulation.launch.py",
        "config/simulation.yaml",
        "config/teleop_acceptance.yaml",
    ):
        assert (share / relative).is_file(), relative
    subprocess.run(["ros2", "run", "gca_ros", "gca_cli", "--help"], check=True, timeout=15)
    for entry in ("gca_teleop_node", "gca_teleop_probe", "gca_teleop_cli"):
        subprocess.run(["ros2", "run", "gca_teleop", entry, "--help"], check=True, timeout=15)
    # Invoke the input-only entry directly, with ROS imports explicitly rejected.
    # Importing its installed module must not require any ROS runtime package.
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys, importlib.abc\n"
            "class NoRos(importlib.abc.MetaPathFinder):\n"
            " def find_spec(self, fullname, path=None, target=None):\n"
            "  if fullname.split('.')[0] in {'rclpy', 'ament_index_python', "
            "'gca_ros', 'gca_system_interfaces', 'system_coordinator'}:\n"
            "   raise RuntimeError('Unexpected ROS dependency: ' + fullname)\n"
            "sys.meta_path.insert(0, NoRos())\n"
            "from gca_teleop.monitor import main\n"
            "main(['--help'])\n",
        ],
        check=True,
        timeout=15,
    )
    subprocess.run(
        [
            str(Path(get_package_prefix("gca_teleop")) / "lib/gca_teleop/gca_joystick_monitor"),
            "--help",
        ],
        check=True,
        timeout=15,
    )
    teleop_share = Path(get_package_share_directory("gca_teleop"))
    for name in ("synthetic.yaml", "beitong_unverified.yaml", "synthetic_idle.json"):
        assert (teleop_share / "config" / name).is_file()
    for launch in ("simulation.launch.py", "teleop_simulation.launch.py"):
        subprocess.run(
            ["ros2", "launch", "system_bringup", launch, "--show-args"],
            check=True,
            timeout=15,
        )
    print("Installed CLI, launch and configuration resources: PASS")


if __name__ == "__main__":
    main()
