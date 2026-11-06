"""Local management client for an existing teleop owner; never claims separately."""

import argparse
import json
import sys
import time
import uuid
from pathlib import Path

from .management import call, default_socket


def request(
    socket_path, operation="status", args=None, *, timeout_s=6.0, expected=None, request_id=None
):
    priority = operation == "stop"
    if operation != "status" and expected is None:
        state = call(
            socket_path,
            {"request_id": uuid.uuid4().hex, "op": "status"},
            timeout_s,
            priority=priority,
        )
        if not state.get("accepted"):
            raise RuntimeError(state.get("error", "STATUS_FAILED"))
        expected = state["expected"]
    message = {"request_id": request_id or uuid.uuid4().hex, "op": operation, "args": args or {}}
    if operation != "status":
        message["expected"] = expected
    return call(socket_path, message, timeout_s, priority=priority)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, default=default_socket())
    parser.add_argument("--timeout", type=float, default=6.0)
    subs = parser.add_subparsers(dest="operation", required=True)
    for operation in (
        "status",
        "configure",
        "activate",
        "claim",
        "arm",
        "disarm",
        "reset",
        "release",
        "script-start",
        "script-stop",
    ):
        subs.add_parser(operation)
    stop = subs.add_parser("stop")
    stop.add_argument(
        "--wait", action="store_true", help="Wait for observed stop confirmation within --timeout"
    )
    reference = subs.add_parser("reference")
    reference.add_argument("--linear-mm", type=float, required=True)
    reference.add_argument("--rotation-deg", type=float, required=True)
    profile = subs.add_parser("input-config")
    profile.add_argument("path", type=Path)
    fault = subs.add_parser("simulation-fault", help="Simulation-only controlled fault injection")
    fault.add_argument(
        "name", choices=("arm_pause", "control_freeze", "telemetry_pause", "encoder_freeze")
    )
    fault.add_argument("enabled", choices=("true", "false"))
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 30:
        parser.error("timeout must be in (0, 30] seconds")
    payload = {}
    if args.operation == "reference":
        payload = {"linear_mm": args.linear_mm, "rotation_deg": args.rotation_deg}
    elif args.operation == "input-config":
        payload = {"path": str(args.path.resolve())}
    elif args.operation == "simulation-fault":
        payload = {"name": args.name, "enabled": args.enabled == "true"}
    try:
        result = request(args.socket, args.operation, payload, timeout_s=args.timeout)
        if args.operation == "stop" and args.wait and result.get("accepted"):
            deadline = time.monotonic() + args.timeout
            while not (
                result.get("stop_confirmed") is True and result.get("stop_progress") == "CONFIRMED"
            ):
                if time.monotonic() >= deadline:
                    raise TimeoutError("STOP_CONFIRMATION_TIMEOUT")
                time.sleep(0.02)
                result = call(
                    args.socket,
                    {"op": "status", "request_id": uuid.uuid4().hex},
                    min(args.timeout, max(0.01, deadline - time.monotonic())),
                    priority=True,
                )
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0 if result.get("accepted") else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"accepted": False, "error": str(exc)}), file=sys.stderr)
        return 1
