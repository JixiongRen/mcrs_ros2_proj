#!/usr/bin/env python3
"""Export one immutable feed-axis snapshot using the two editable Python settings files."""

import argparse
import json
import sys
from pathlib import Path
from runpy import run_path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/gca_core"))

from gca.servo_config import (  # noqa: E402
    make_position_settings,
    make_servo_settings,
    servo_settings_document,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--control-mode", choices=("manual", "position"), required=True)
    parser.add_argument("--backend", choices=("simulation", "hardware"), required=True)
    parser.add_argument(
        "--manual-input-mode",
        choices=("qualified_state", "joydev_hold"),
        default="qualified_state",
    )
    args = parser.parse_args(argv)
    servo = run_path(str(ROOT / "config/servo_settings.py"))
    runtime = servo["settings_payload"](args.backend, manual_input_mode=args.manual_input_mode)
    if args.control_mode == "position":
        encoder = run_path(str(ROOT / "config/encoder_settings.py"))["settings_payload"]()
        settings = make_position_settings(runtime, servo["position_settings_payload"](), encoder)
    else:
        settings = make_servo_settings(
            {**runtime, "schema_version": 3, "control_mode": "manual", "position": None}
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(servo_settings_document(settings), ensure_ascii=False, indent=2, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    print(f"{args.output.resolve()} mode={settings.control_mode} sha256={settings.sha256}")


if __name__ == "__main__":
    main()
