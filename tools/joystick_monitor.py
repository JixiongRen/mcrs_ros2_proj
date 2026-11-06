#!/usr/bin/env python3
"""Run the workspace's read-only joystick monitor without sourcing ROS.

Only the pure Python input and domain packages are added to the import path.
This is a source-tree development entry; the installed gca_joystick_monitor
entry uses its installed packages instead.
"""

import sys
from pathlib import Path


def main():
    source = Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
    for package in ("gca_core", "gca_teleop"):
        sys.path.insert(0, str(source / package))
    from gca_teleop.monitor import main as monitor_main

    return monitor_main()


if __name__ == "__main__":
    raise SystemExit(main())
