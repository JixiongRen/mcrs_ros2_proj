#!/usr/bin/env python3
"""Run installed ROS processes, record DDS telemetry, and export offline evidence.

This runner owns only its launch/client process groups. Devices remain simulated;
neither the runner nor the launched simulation configuration opens hardware.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from gca_system_interfaces.msg import GcaState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

SCENARIOS = ("basic", "cancel", "jog", "hold", "faults")


def stop_owned(process: subprocess.Popen, grace_s: float = 8.0) -> int:
    """Bounded shutdown for a process group that this runner created itself."""

    # The launcher can die before its nodes. Keep the original group id and
    # clean that group even when the leader has already been reaped.
    def send(sig):
        try:
            os.killpg(process.pid, sig)
            return True
        except ProcessLookupError:
            return False

    if not send(signal.SIGINT):
        return process.wait(timeout=2.0)
    try:
        code = process.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        send(signal.SIGTERM)
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            send(signal.SIGKILL)
            process.wait(timeout=2.0)
        return -signal.SIGKILL  # Escalated shutdown cannot count as a clean pass.
    deadline = time.monotonic() + 2.0
    while send(0) and time.monotonic() < deadline:
        time.sleep(0.02)
    if send(0):
        send(signal.SIGKILL)
        return -signal.SIGKILL
    return code


def _metrics(rows: list[dict], scenario: str) -> dict:
    configured = [r for r in rows if r["sample_seq"] > 0]
    active = [r for r in configured if r["state"] in ("EXECUTING", "HOLDING")]
    result = {
        "samples": len(rows),
        "active_samples": len(active),
        "clock": "host_monotonic_wall",
        "states": sorted({r["state"] for r in rows}),
        "fault_codes": sorted({r["fault_code"] for r in rows if r["fault_code"]}),
    }
    if active:
        errors = [abs(r["target_linear_m"] - r["linear_position_m"]) * 1000 for r in active]
        result.update(
            max_abs_target_error_mm=max(errors),
            last_active_error_mm=errors[-1],
            motion_observed_duration_s=active[-1]["wall_s"] - active[0]["wall_s"],
        )
    if scenario in ("basic", "hold") and active:
        result["positive_overshoot_mm"] = max(
            0, max(r["linear_position_m"] for r in active) * 1000 - 5
        )
    stopping = next((r for r in rows if r["stop"]["progress"] == "IN_PROGRESS"), None)
    if stopping is not None:
        confirmed = next(
            (
                r
                for r in rows
                if r["wall_s"] >= stopping["wall_s"] and r["stop"]["progress"] == "CONFIRMED"
            ),
            None,
        )
        if confirmed:
            result["dds_observed_stop_response_s"] = confirmed["wall_s"] - stopping["wall_s"]
    return result


def _plots(directory: Path, rows: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not rows:
        return
    x = [r["wall_s"] for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for field, label in (
        ("target_linear_m", "Target"),
        ("reference_linear_m", "Trajectory"),
        ("linear_position_m", "Encoder"),
    ):
        axes[0].plot(x, [r[field] * 1000 for r in rows], label=label)
    axes[0].set_ylabel("Linear position [mm]")
    for field, label in (
        ("requested_velocity_m_s", "Requested"),
        ("applied_velocity_m_s", "Accepted output"),
        ("linear_velocity_m_s", "Estimated feedback"),
    ):
        axes[1].plot(x, [r[field] * 1000 for r in rows], label=label)
    axes[1].set_ylabel("Velocity [mm/s]")
    for field, label in (
        ("target_rotation_rad", "Target"),
        ("rotation_estimated_rad", "Confirmed pulse estimate"),
    ):
        axes[2].plot(x, [math.degrees(r[field]) for r in rows], label=label)
    axes[2].set_ylabel("Rotation [deg]")
    axes[2].set_xlabel("Host monotonic wall time [s] (simulation, no physical performance claim)")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(directory / "curves.png", dpi=150)
    fig.savefig(directory / "curves.svg")
    plt.close(fig)


def run_scenario(scenario: str, directory: Path, timeout_s: float = 70.0) -> dict:
    directory.mkdir(parents=True, exist_ok=True)
    # A fresh graph for each scenario prevents a previous lease, transient state,
    # or failed client from making the next scenario appear successful.
    environment = dict(os.environ)
    environment["ROS_LOG_DIR"] = str(directory / "ros_logs")
    context = Context()
    rclpy.init(context=context, args=[])
    node = Node("gca_evidence_recorder", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    rows = []
    origin = time.monotonic()

    def sample(message):
        row = dict(message_to_ordereddict(message))
        row["wall_s"] = time.monotonic() - origin
        rows.append(row)
        if len(rows) > 100_000:
            raise RuntimeError("Telemetry evidence capacity exceeded")

    node.create_subscription(
        GcaState,
        "/gca/state",
        sample,
        QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    launch_command = ["ros2", "launch", "system_bringup", "simulation.launch.py"]
    client_command = [
        "ros2",
        "run",
        "gca_ros",
        "gca_cli",
        "demo",
        "--scenario",
        scenario,
        "--output",
        str(directory / "client"),
    ]
    result = {
        "scenario": scenario,
        "passed": False,
        "launch_command": launch_command,
        "client_command": client_command,
        "physical_hil": "NOT_RUN",
    }
    launch = client = None
    with (
        tempfile.TemporaryDirectory(prefix="gca-simulation-cwd-") as outside,
        (directory / "launch.log").open("w") as launch_log,
        (directory / "client.log").open("w") as client_log,
    ):
        try:
            launch = subprocess.Popen(
                launch_command,
                cwd=outside,
                env=environment,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            discovery_end = time.monotonic() + 15
            while time.monotonic() < discovery_end:
                if launch.poll() is not None:
                    raise RuntimeError(f"Launch exited before discovery: {launch.returncode}")
                executor.spin_once(timeout_sec=0.05)
                if "/system/claim_control" in {
                    name for name, _ in node.get_service_names_and_types()
                }:
                    break
            else:
                raise TimeoutError("ROS coordinator discovery deadline exceeded")
            client = subprocess.Popen(
                client_command,
                cwd=outside,
                env=environment,
                stdout=client_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + timeout_s
            while client.poll() is None:
                executor.spin_once(timeout_sec=0.03)
                if launch.poll() is not None:
                    raise RuntimeError("ROS launch exited during demonstration")
                if time.monotonic() >= deadline:
                    raise TimeoutError("Scenario wall deadline exceeded")
            result["client_exit_code"] = client.returncode
            for _ in range(5):
                executor.spin_once(timeout_sec=0.03)
            report_path = directory / "client" / "report.json"
            report = json.loads(report_path.read_text()) if report_path.exists() else {}
            result["client_report"] = report
            if client.returncode != 0 or not report.get("passed"):
                raise AssertionError(
                    f"Installed CLI failed: {report.get('error', 'see client.log')}"
                )
            if report.get("renewal_error"):
                raise AssertionError(f"Source renewal failed: {report['renewal_error']}")
            if len(rows) < 3 or not any(r["sample_seq"] > 0 for r in rows):
                raise AssertionError("No independent DDS encoder telemetry")
            if scenario in ("basic", "cancel", "hold", "jog"):
                if not any(abs(r["linear_position_m"]) > 0.00005 for r in rows):
                    raise AssertionError("No observed plant displacement")
            if scenario in ("basic", "hold"):
                outcome = report["results"][0]
                if abs(outcome["linear_position_mm"] - 5.0) > 0.12:
                    raise AssertionError("Simulated final linear error exceeds 0.12 mm")
                if abs(outcome["rotation_estimated_deg"] - 20.0) > 0.04:
                    raise AssertionError("Pulse count angle differs by more than quantization")
            result["passed"] = True
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
            (directory / "exception.txt").write_text(traceback.format_exc())
        finally:
            if client is not None:
                result["client_exit_code"] = stop_owned(client)
            if launch is not None:
                result["launch_exit_code"] = stop_owned(launch)
            if any(result.get(key, 0) != 0 for key in ("client_exit_code", "launch_exit_code")):
                result["passed"] = False
                result.setdefault(
                    "error", "A launched process failed or shutdown required escalation"
                )
            executor.shutdown(timeout_sec=2)
            node.destroy_node()
            rclpy.shutdown(context=context)
    result["wall_duration_s"] = time.monotonic() - origin
    result["metrics"] = _metrics(rows, scenario)
    with (directory / "states.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
    if rows:
        scalar_keys = [k for k, v in rows[0].items() if not isinstance(v, (dict, list))]
        with (directory / "states.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=scalar_keys, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)
        _plots(directory, rows)
    (directory / "report.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/simulation/latest"))
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    configuration = Path(get_package_share_directory("system_bringup")) / "config/simulation.yaml"
    report = {
        "source": "installed ROS 2 nodes, actual DDS; simulated actuators and independent plant",
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ["ROS_DOMAIN_ID"],
        "configuration_sha256": hashlib.sha256(configuration.read_bytes()).hexdigest(),
        "configuration": configuration.read_text(),
        "physical_hil": "NOT_RUN",
        "scenarios": [],
    }
    base_domain = int(os.environ["ROS_DOMAIN_ID"])
    for index, scenario in enumerate(args.scenarios):
        os.environ["ROS_DOMAIN_ID"] = str(100 + (base_domain - 100 + index) % 100)
        print(f"Running ROS simulation: {scenario}", flush=True)
        result = run_scenario(scenario, output / scenario)
        result["ros_domain_id"] = os.environ["ROS_DOMAIN_ID"]
        report["scenarios"].append(result)
        print(
            f"{scenario}: {'PASS' if result['passed'] else 'FAIL'} {result.get('error', '')}",
            flush=True,
        )
    from model_report import run as run_model_report

    print("Running deterministic algorithm/plant reports (model time)", flush=True)
    try:
        report["model_report"] = run_model_report(output / "model")
    except Exception as exc:
        report["model_report"] = {"passed": False, "error": f"{type(exc).__name__}: {exc}"}
        (output / "model_exception.txt").write_text(traceback.format_exc())
    report["passed"] = (
        all(r["passed"] for r in report["scenarios"]) and report["model_report"]["passed"]
    )
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"Evidence: {output / 'report.json'}", flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
