#!/usr/bin/env python3
"""One bounded installed-ROS position run; simulation only, with fixed plant slip."""

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from gca_client.client import GcaClient
from rclpy.signals import SignalHandlerOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    snapshot = directory / "position.json"
    subprocess.run(
        [
            sys.executable,
            str(root / "tools/export_feed_axis_config.py"),
            str(snapshot),
            "--control-mode",
            "position",
            "--backend",
            "simulation",
        ],
        check=True,
        timeout=10,
    )
    os.environ.update(
        ROS_DOMAIN_ID=str(10 + secrets.randbelow(90)),
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
        ROS_LOCALHOST_ONLY="1",
        ROS_LOG_DIR=str(directory / "ros_logs"),
    )
    report = {
        "software_source": "installed ROS packages; physical plant is simulated",
        "simulation_slip_ratio": 0.85,
        "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
        "physical_hil": "NOT_RUN: user confirmed mechanism is not coupled",
        "cross_host": "NOT_RUN: no second ROS host configured",
        "results": [],
        "passed": False,
    }
    process = None
    client = None
    started = time.monotonic()

    def timeout(signum, frame):
        raise TimeoutError("Position integration exceeded its 90 second total budget")

    previous_alarm = signal.signal(signal.SIGALRM, timeout)
    signal.alarm(90)
    try:
        with (directory / "launch.log").open("w") as launch_log:
            process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "system_bringup",
                    "feed_axis.launch.py",
                    "control_mode:=position",
                    "backend:=simulation",
                    f"servo_config_file:={snapshot}",
                    "simulation_slip_ratio:=0.85",
                    f"record_directory:={directory / 'records'}",
                ],
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            client = GcaClient(timeout_s=8.0)
            client.configure()
            client.activate()
            caps = client.capabilities()
            if not caps.simulation or caps.backend != "simulation":
                raise RuntimeError("Verifier refuses a non-simulation device")
            client.wait_state(
                lambda state: (
                    state.state == "DISARMED"
                    and state.sample_seq > 0
                    and state.linear_velocity_valid
                )
            )
            client.claim("AUTO")
            client.set_linear_reference(0.0)
            client.wait_state(
                lambda state: state.linear_reference_valid and state.linear_velocity_valid
            )
            client.manage("arm", axis_mask=1)
            client.wait_state(lambda state: state.armed)
            expected = 0.0
            for target, relative in ((5.0, True), (0.0, False), (-3.0, True), (0.0, False)):
                # REL is resolved once from current measured position, not the preceding target.
                initial = (
                    client.wait_state(lambda state: state.linear_valid).linear_position_m * 1000
                )
                expected = initial + target if relative else target
                result = client.move_linear(target, relative=relative)
                final = result.final_state
                row = {
                    "target_mm": target,
                    "relative": relative,
                    "initial_mm": initial,
                    "expected_mm": expected,
                    "position_mm": final.linear_position_m * 1000,
                    "velocity_mm_s": final.linear_velocity_m_s * 1000,
                    "outcome": result.outcome,
                    "error_code": result.error_code,
                    "stop_confirmed": result.stop_confirmed,
                    "outputs_held": result.outputs_held,
                }
                report["results"].append(row)
                if not (
                    result.outcome == 1
                    and result.stop_confirmed
                    and not result.outputs_held
                    and final.linear_valid
                    and abs(row["position_mm"] - expected) <= 1.0
                    and abs(row["velocity_mm_s"]) <= 0.1
                ):
                    raise AssertionError(row)
                print(json.dumps(row), flush=True)
            result = client.move_linear(20.0, relative=True, cancel_after=0.3)
            row = {
                "operation": "cancel",
                "outcome": result.outcome,
                "stop_confirmed": result.stop_confirmed,
                "error_code": result.error_code,
            }
            report["results"].append(row)
            if result.outcome != 2 or not result.stop_confirmed or result.outputs_held:
                raise AssertionError(row)
            client.manage("request_stop", axis_mask=1)
            client.wait_state(lambda state: state.stop.progress == "CONFIRMED")
            report["renewal_error"] = client.renewal_error
            if client.renewal_error:
                raise AssertionError(client.renewal_error)
            report["passed"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, previous_alarm)
        if client is not None:
            report["cleanup_errors"] = client.close()
            if report["cleanup_errors"]:
                report["passed"] = False
            (directory / "samples.json").write_text(json.dumps(client.samples, indent=2) + "\n")
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    # A hung simulation is a failed verification, not a successful mode switch.
                    report["passed"] = False
                    report["forced_simulation_cleanup"] = True
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
            else:
                report["passed"] = False
                report["unexpected_launch_exit"] = True
            report["launch_exit_code"] = process.returncode
            if process.returncode not in (0, -signal.SIGINT):
                report["passed"] = False
        report["wall_s"] = time.monotonic() - started
        (directory / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
