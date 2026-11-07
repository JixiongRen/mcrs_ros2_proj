#!/usr/bin/env python3
"""One bounded installed ROS/joystick/servo simulation run. Never opens hardware."""

import argparse
import json
import math
import os
import subprocess
import tempfile
import time
import traceback
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from gca.servo_config import load_servo_settings
from gca_teleop.cli import request
from lifecycle_msgs.srv import GetState
from simulation_suite import stop_owned
from teleop_suite import Observer, wait_for


def script():
    return {
        "schema_version": 1,
        "duration_s": 10.0,
        "events": [
            {"at_s": 0.0, "axes": {"0": 0, "1": 0, "2": -32768}, "buttons": {"1": 0}},
            {"at_s": 1.0, "axes": {"2": 32767}},
            {"at_s": 1.2, "axes": {"1": -8192}},
            {"at_s": 2.2, "axes": {"1": -32768}},
            {"at_s": 2.5, "axes": {"0": 32767, "3": 32767}, "buttons": {"5": 1}},
            {"at_s": 3.4, "axes": {"2": -32768}},
            {"at_s": 4.0, "axes": {"1": 0}},
            {"at_s": 4.8, "axes": {"2": 32767}},
            {"at_s": 5.0, "axes": {"1": 8192}},
            {"at_s": 6.0, "axes": {"1": 32767}},
            {"at_s": 7.2, "buttons": {"1": 1}},
            {"at_s": 7.5, "buttons": {"1": 0}},
            {"at_s": 8.5, "axes": {"1": 0, "2": -32768}},
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-file", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--domain", type=int, default=213)
    args = parser.parse_args()
    config = load_servo_settings(args.config_file)
    if config.backend != "simulation":
        parser.error("only simulation snapshots are accepted")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    script_path = output / "input.json"
    script_path.write_text(json.dumps(script(), indent=2) + "\n")
    report = {
        "passed": False,
        "backend": "simulation",
        "hardware_tested": False,
        "config_sha256": config.sha256,
        "commands": [],
        "errors": [],
        "predeclared": {
            "startup_s": 20,
            "management_s": 6,
            "stop_s": 3,
            "script_s": 10,
            "max_rpm": config.max_rpm,
        },
    }
    process, observer, log = None, None, None
    socket_dir = tempfile.TemporaryDirectory(prefix="gca-servo-")
    socket = Path(socket_dir.name) / "owner.sock"

    def manage(operation):
        response = request(socket, operation, timeout_s=6)
        # Status excludes capabilities/tokens; retain command result and meaningful evidence.
        report["commands"].append({"operation": operation, "response": response})
        if not response.get("accepted"):
            raise RuntimeError(f"{operation}: {response.get('error')}")
        return response

    try:
        profile = Path(get_package_share_directory("gca_teleop")) / "config/synthetic.yaml"
        command = [
            "ros2",
            "launch",
            "system_bringup",
            "servo_simulation.launch.py",
            f"servo_config_file:={args.config_file.resolve()}",
            f"profile_file:={profile}",
            f"script_file:={script_path}",
            f"socket_path:={socket}",
            f"record_directory:={output / 'records'}",
        ]
        report["launch_command"] = command
        log = (output / "launch.log").open("w")
        process = subprocess.Popen(
            command,
            cwd=output,
            start_new_session=True,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=dict(
                os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(output / "ros_logs")
            ),
        )
        observer = Observer(args.domain)
        wait_for(socket.exists, 20, "owner socket", process)

        def owner_initialized():
            state = request(socket)
            return (
                state["health"]["initialized"]
                and state["health"]["reason"] == "SCRIPT_PAUSED"
                and bool(state["expected"]["input_epoch"])
            )

        # Socket creation precedes the reader's first epoch. A device boot ID
        # exists only after configure; its state must be awaited separately.
        wait_for(owner_initialized, 20, "initialized owner input", process)
        # The owner socket and reader may start before the executor's lifecycle
        # services. Verify an actual read-only reply within the startup budget;
        # do not spend the short management deadline on node startup/discovery.
        lifecycle = observer.node.create_client(GetState, "/gca_control_node/get_state")
        try:
            wait_for(lifecycle.service_is_ready, 20, "executor lifecycle service", process)
            startup_state = lifecycle.call_async(GetState.Request())
            wait_for(startup_state.done, 20, "executor lifecycle response", process)
            state_id = startup_state.result().current_state.id
            report["startup_lifecycle_state"] = state_id
            if state_id != 1:
                raise AssertionError(f"expected unconfigured executor, got {state_id}")
        finally:
            observer.node.destroy_client(lifecycle)
        manage("configure")

        def configured_owner_observed():
            owner = request(socket)
            return (
                owner["device_state"] == "DISARMED"
                and owner["expected"]["config_revision"] == config.revision
                and owner["backend"] == "simulation"
                and owner["stop_confirmed"]
            )

        wait_for(configured_owner_observed, 20, "configured owner state", process)
        manage("activate")

        def active_state_observed():
            states, _, _ = observer.snapshot()
            return bool(
                states
                and states[-1]["mode"] == "SERVO_ONLY"
                and states[-1]["lifecycle_active"]
                and configured_owner_observed()
            )

        wait_for(active_state_observed, 20, "active executor DDS state", process)
        for operation in ("claim", "arm"):
            manage(operation)
        result = manage("script-start")
        start = result["script_started_ns"]
        if start <= 0:
            raise AssertionError("script did not report a start time")
        report["script_started_ns"] = start
        wait_for(
            lambda: time.monotonic_ns() >= start + 10_200_000_000,
            11,
            "bounded input sequence",
            process,
        )
        manage("stop")
        wait_for(lambda: request(socket).get("stop_confirmed"), 3, "STOP confirmation", process)
        manage("release")  # Servo owner waits DISARM before releasing its own credentials.
        final = manage("status")
        if final["session_owned"] or final["armed"] or not final["stop_confirmed"]:
            raise AssertionError("final ownership/disable/stop evidence incomplete")
        states, teleop, _ = observer.snapshot()
        active = [s for s in states if s["received_monotonic_ns"] >= start and s["servo"]["valid"]]
        if not active:
            raise AssertionError("no valid servo samples")

        def rpm(s, name):
            return s["servo"][name] * 60 / (2 * math.pi)

        measured = [rpm(s, "measured_velocity_rad_s") for s in active]
        targets = [rpm(s, "requested_velocity_rad_s") for s in active]
        accepted = [rpm(s, "accepted_velocity_rad_s") for s in active]
        after_b = [s for s in active if 8 <= (s["received_monotonic_ns"] - start) / 1e9 < 8.5]
        checks = {
            "up_negative_request": min(targets) < -0.9 * config.max_rpm,
            "down_positive_request": max(targets) > 0.9 * config.max_rpm,
            "both_measured_directions": min(measured) < -0.229 and max(measured) > 0.229,
            "multiple_requested_amplitudes": any(
                0.02 * config.max_rpm < abs(x) < 0.5 * config.max_rpm for x in targets
            ),
            "bounded_accepted_speed": max(map(abs, accepted)) <= config.max_rpm + 1e-9,
            "no_external_measurements": all(
                not s["linear_valid"] and not s["rotation_estimate_valid"] for s in active
            ),
            "no_late_b_resume": bool(after_b)
            and all(
                abs(rpm(s, "accepted_velocity_rad_s")) < 1e-9
                and abs(rpm(s, "requested_velocity_rad_s")) < 1e-9
                and abs(rpm(s, "measured_velocity_rad_s")) <= config.velocity_tolerance_rpm
                and not s["servo"]["torque_enabled"]
                for s in after_b
            ),
            "no_observer_error": not observer.errors,
        }
        input_records = []
        for path in sorted((output / "records/teleop").glob("control-*.jsonl")):
            input_records.extend(json.loads(line) for line in path.read_text().splitlines())
        reasons = {r["intent"]["reason"] for r in input_records if r.get("kind") == "input"}
        checks["lt_release_recorded"] = "DEADMAN_RELEASED" in reasons
        checks["b_stop_recorded"] = "B_STOP" in reasons
        report["checks"] = checks
        report["range_rpm"] = {
            "requested": [min(targets), max(targets)],
            "accepted": [min(accepted), max(accepted)],
            "measured": [min(measured), max(measured)],
        }
        if not all(checks.values()):
            raise AssertionError(
                "failed checks: " + ", ".join(k for k, v in checks.items() if not v)
            )
        report["passed"] = True
    except Exception as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
        report["traceback"] = traceback.format_exc()
    finally:
        if socket.exists():
            try:
                current = request(socket)
                if current.get("session_owned"):
                    manage("stop")
                    manage("release")
            except Exception as exc:
                report["passed"] = False
                report["errors"].append(f"cleanup: {exc}")
        if process is not None:
            report["launch_exit_code"] = stop_owned(process)
            if report["launch_exit_code"] != 0:
                report["passed"] = False
        if observer is not None:
            states, teleop, phases = observer.snapshot()
            for name, data in (
                ("states", states),
                ("teleop", teleop),
                ("arm_phases", phases),
                ("io_evidence", observer.io_snapshot()),
            ):
                (output / f"{name}.jsonl").write_text(
                    "".join(json.dumps(row) + "\n" for row in data)
                )
            observer.close()
        if log is not None:
            log.close()
        socket_dir.cleanup()
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "report": str(output / "report.json"),
                "errors": report["errors"],
            },
            ensure_ascii=False,
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
