#!/usr/bin/env python3
"""Bounded standalone servo READ probe; never configures, stops or enables torque."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/gca_core"))
from gca.devices.dynamixel import DynamixelVelocityDrive  # noqa: E402
from gca.servo_config import load_servo_settings  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--samples", type=int, default=3)
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 10:
        parser.error("--samples must be between 1 and 10")
    config = load_servo_settings(args.config_file)
    if config.backend != "hardware":
        parser.error("a snapshot explicitly exported with --backend hardware is required")
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "ros_used": False,
        "allowed_instructions": ["READ"],
        "write_instructions_sent": 0,
        "config_revision": config.revision,
        "config_sha256": config.sha256,
        "port": config.device.port,
        "samples": [],
        "operations": [],
        "status": "FAIL",
    }
    drive = DynamixelVelocityDrive(config.device)

    def measured(label, function):
        started = time.monotonic_ns()
        try:
            return function()
        finally:
            report["operations"].append(
                {
                    "name": label,
                    "started_ns": started,
                    "duration_s": (time.monotonic_ns() - started) / 1e9,
                }
            )

    try:
        report["identity"] = asdict(measured("open_identified", drive.open_identified))
        report["registers"] = measured("configuration_read", drive.read_configuration_registers)
        for _ in range(args.samples):
            sample = measured(
                "shaft_feedback",
                lambda: drive.read_shaft_state(
                    time.monotonic_ns() + int(config.io_timeout_s * 1e9)
                ),
            )
            report["samples"].append(asdict(sample))
            time.sleep(config.io_period_s)
        report["status"] = "PASS"
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            drive.disconnect_readonly()
            report["readonly_disconnected"] = True
        except Exception as exc:
            report["readonly_disconnected"] = False
            report["disconnect_error"] = f"{type(exc).__name__}: {exc}"
            report["status"] = "FAIL"
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(
        json.dumps(
            {"status": report["status"], "report": str(output / "report.json")}, ensure_ascii=False
        )
    )
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
