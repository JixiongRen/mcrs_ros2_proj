#!/usr/bin/env python3
"""Single editable source of native servo settings; never opens hardware.

Run without ROS: python3 config/servo_settings.py show
Export: python3 config/servo_settings.py export OUTPUT.json --backend simulation

Identity values were rechecked in servo_ros2/implementation-xce7i_e4/readonly.
Speed raw=349 gives a 79.921 rpm command ceiling. This is not a motor rating:
XM430-W350 no-load speed is 46 rpm at 12 V and 57 rpm at 14.8 V.
Timing/acceleration values below are explicit engineering candidates: the native feedback block took
about 28–32 ms per READ in three read-only samples at 57600 baud.
They must be measured during real-chain acceptance, not labelled HIL PASS.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# The local schema also uses only the standard library.  This makes export
# independent of ROS installation, a virtual environment and package building.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/gca_core"))
from gca.servo_config import make_servo_settings, servo_settings_document  # noqa: E402


def settings_payload(backend: str, *, manual_input_mode: str = "qualified_state"):
    return {
        "schema_version": 2,
        "config_revision": "servo-held-v5"
        if manual_input_mode == "joydev_hold"
        else "servo-manual-v5",
        "backend": backend,
        "device": {
            "port": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5WJAPT-if00-port0",
            "baudrate": 57600,
            "device_id": 1,
            "model_number": 1020,
            "velocity_unit_rpm": 0.229,
            "max_velocity_raw": 349,
            "watchdog_raw": 25,
            "profile_acceleration_raw": 12,
            "transaction_timeout_s": 0.08,
        },
        "manual_axis_to_servo_sign": -1,
        "max_acceleration_rpm_s": 40.0,
        "control_period_s": 0.04,
        "io_period_s": 0.02,
        "feedback_max_age_s": 0.25,
        # A write and feedback read each have an 80 ms transaction budget.
        # Allow control/ROS scheduling as well; never renew an issued deadline.
        "output_ttl_s": 0.30,
        # At about 45 rpm, fresh zero + dwell took 1.28 s after LT release.
        # This bounds confirmation only; STOP still revokes output immediately.
        "stop_timeout_s": 2.0,
        "settle_s": 0.10,
        "velocity_tolerance_rpm": 0.229,
        "lease_max_s": 5.0,
        "manual_input_max_age_s": 0.30,
        "manual_input_mode": manual_input_mode,
        "queue_capacity": 64,
        "history_capacity": 256,
        "mechanical_mapping": None,
        "position_pid": None,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("show", "export"))
    parser.add_argument("output", nargs="?")
    parser.add_argument("--backend", choices=("simulation", "hardware"), default="simulation")
    parser.add_argument(
        "--manual-input-mode",
        choices=("qualified_state", "joydev_hold"),
        default="qualified_state",
    )
    args = parser.parse_args(argv)
    if (args.operation == "export") != (args.output is not None):
        parser.error("export requires OUTPUT.json; show takes no output path")
    settings = make_servo_settings(
        settings_payload(args.backend, manual_input_mode=args.manual_input_mode)
    )
    rendered = (
        json.dumps(servo_settings_document(settings), ensure_ascii=False, indent=2, allow_nan=False)
        + "\n"
    )
    if args.operation == "show":
        print(rendered, end="")
    else:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered)
        print(f"{output.resolve()} sha256={settings.sha256}")


if __name__ == "__main__":
    main()
