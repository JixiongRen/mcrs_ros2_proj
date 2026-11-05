#!/usr/bin/env python3
"""Check an ordinary install from outside the source tree, without PYTHONPATH edits."""

import importlib
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    for name in ("gca", "gca_ros", "system_coordinator", "system_bringup", "gca_system_interfaces"):
        module = importlib.import_module(name)
        path = Path(module.__file__).resolve()
        assert "/src/" not in str(path), f"Source import masks broken installation: {path}"
        print(f"{name}: {path}")
    for package, executable in (
        ("gca_ros", "gca_cli"),
        ("gca_ros", "gca_control_node"),
        ("system_coordinator", "system_coordinator"),
    ):
        assert (Path(get_package_prefix(package)) / "lib" / package / executable).is_file()
    share = Path(get_package_share_directory("system_bringup"))
    for relative in (
        "launch/simulation.launch.py",
        "launch/hardware.launch.py",
        "config/simulation.yaml",
    ):
        assert (share / relative).is_file(), relative
    subprocess.run(["ros2", "run", "gca_ros", "gca_cli", "--help"], check=True, timeout=15)
    subprocess.run(
        ["ros2", "launch", "system_bringup", "simulation.launch.py", "--show-args"],
        check=True,
        timeout=15,
    )
    print("Installed CLI, launch and configuration resources: PASS")


if __name__ == "__main__":
    main()
