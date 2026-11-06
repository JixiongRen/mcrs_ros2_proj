"""The source-tree terminal entry must work with ROS and actuator imports blocked."""

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/joystick_monitor.py"
BLOCKED = """
import importlib.abc
import runpy
import sys
class RejectRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {
            'rclpy', 'ament_index_python', 'gca_ros', 'gca_system_interfaces',
            'system_coordinator', 'serial', 'pigpio', 'dynamixel_sdk',
        }:
            raise RuntimeError('Unexpected runtime import: ' + fullname)
sys.meta_path.insert(0, RejectRuntime())
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
"""


def invoke(tmp_path, *args):
    return subprocess.run(
        [sys.executable, "-I", "-c", BLOCKED, str(SCRIPT), *args],
        cwd=tmp_path,
        env={"PATH": os.defpath, "LANG": "C.UTF-8"},
        capture_output=True,
        text=True,
        timeout=10,
    )


def test_source_entry_help_without_ros_environment_or_pythonpath(tmp_path):
    result = invoke(tmp_path, "--help")
    assert result.returncode == 0, result.stderr
    assert "--profile" in result.stdout
    assert "--duration" in result.stdout
    assert "--jsonl" in result.stdout


def test_source_entry_refuses_scripted_profile_before_any_hardware_import(tmp_path):
    profile = ROOT / "ros2_ws/src/gca_teleop/config/synthetic.yaml"
    result = invoke(tmp_path, "--profile", str(profile), "--duration", "0.1")
    assert result.returncode == 2
    assert "physical joydev" in result.stderr
    assert "Unexpected runtime import" not in result.stderr
