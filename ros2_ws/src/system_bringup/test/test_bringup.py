"""Verify installed launch failure propagation and owned child cleanup."""

import os
import signal
import subprocess


def test_invalid_configuration_fails_launch_instead_of_leaving_half_system(tmp_path):
    configuration = tmp_path / "invalid.yaml"
    configuration.write_text("runtime:\n  backend: invalid-backend\n")
    with (tmp_path / "launch.log").open("w") as stream:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "system_bringup",
                "simulation.launch.py",
                f"config_file:={configuration}",
            ],
            cwd=tmp_path,
            env=dict(os.environ, ROS_DOMAIN_ID="207", ROS_LOG_DIR=str(tmp_path / "logs")),
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            assert process.wait(15) != 0
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(2)
    log = (tmp_path / "launch.log").read_text()
    assert "backend must be simulation or hardware" in log
    assert "child exited unexpectedly" in log
