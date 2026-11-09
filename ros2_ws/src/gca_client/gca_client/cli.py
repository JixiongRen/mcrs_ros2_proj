"""Installed client commands; position-shell keeps one renewable AUTO session."""

import argparse
import json
import math
import shlex
import signal
import sys
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

from .client import Credentials, GcaClient, save_credentials


def _result_json(result):
    return {
        "outcome": int(result.outcome),
        "error_code": result.error_code,
        "message": result.message,
        "outputs_held": result.outputs_held,
        "stop_confirmed": result.stop_confirmed,
        "final_state": result.final_state.state,
        "linear_position_mm": result.final_state.linear_position_m * 1000,
        "rotation_estimated_deg": math.degrees(result.final_state.rotation_estimated_rad),
    }


def run_demo(client, scenario="basic", output=None):
    """Exercise real ROS processes; only device and plant implementations are simulated."""
    client.configure()
    client.activate()
    client.wait_state(lambda state: state.state == "DISARMED")
    client.claim(mode="MANUAL" if scenario == "jog" else "AUTO")
    client.manage("set_reference")
    client.wait_state(lambda state: state.linear_reference_valid and state.rotation_reference_valid)
    # Verify ARM admission with a real Action request, not a direct runtime call.
    rejected = client.send_goal(client.make_goal())
    if rejected.accepted:
        raise AssertionError("Motion was accepted before ARM")
    client.manage("arm")
    client.wait_state(lambda state: state.armed)
    results = []
    if scenario == "basic":
        result = client.move()
        assert result.outcome == 1, _result_json(result)
        results.append(_result_json(result))
    elif scenario == "cancel":
        result = client.move(
            linear_mm=50.0,
            rotation_deg=180.0,
            max_linear_mm_s=2.0,
            timeout_s=40.0,
            cancel_after=0.25,
        )
        assert result.outcome == 2 and result.stop_confirmed, _result_json(result)
        results.append(_result_json(result))
    elif scenario == "hold":
        result = client.move(hold=True)
        assert result.outcome == 1 and result.outputs_held, _result_json(result)
        client.wait_state(lambda state: state.state == "HOLDING")
        client.manage("request_stop")
        client.wait_state(
            lambda state: state.stop.progress == "CONFIRMED" and state.state != "HOLDING"
        )
        results.append(_result_json(result))
    elif scenario == "jog":
        client.jog()
        client.wait_state(lambda state: state.stop.progress == "CONFIRMED")
        results.append({"jog_deadman_stop_confirmed": True})
    elif scenario == "faults":
        goal = client.make_goal(
            linear_mm=40.0, rotation_deg=0.0, axis_mask=1, max_linear_mm_s=2.0, timeout_s=30.0
        )
        handle = client.send_goal(goal)
        if not handle.accepted:
            raise AssertionError("Fault scenario motion was rejected")
        client.wait_state(lambda state: state.state == "EXECUTING")
        client.fault("encoder_disconnect", True)
        client.wait_state(lambda state: state.state == "FAULT")
        result = client.wait(handle.get_result_async(), timeout_s=5.0).result
        assert result.outcome == 3, _result_json(result)
        client.fault("encoder_disconnect", False)
        client.wait_state(lambda state: state.linear_valid)
        client.manage("reset_fault")
        client.wait_state(lambda state: state.state == "DISARMED")
        results.append({"encoder_fault_latched_and_reset": True})
    else:
        raise ValueError("unknown scenario")
    client.manage("disarm")
    client.wait_state(lambda state: state.state == "DISARMED")
    report = {
        "scenario": scenario,
        "passed": True,
        "results": results,
        "sample_count": len(client.samples),
        "renewal_error": client.renewal_error,
        "physical_hil": "NOT_RUN",
        "source": "actual ROS processes, simulated device plant",
    }
    if output is not None:
        directory = Path(output)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        )
        (directory / "samples.json").write_text(
            json.dumps(client.samples, ensure_ascii=False, indent=2) + "\n"
        )
    return report


def interactive_shell(client):
    """Keep one renewable session while the operator enters successive commands."""
    print("Commands: configure, activate, claim [MANUAL|AUTO], reference [mm deg], arm,")
    print("move <mm> <deg> [relative] [hold], jog <mm/s> <deg/s> <seconds>,")
    print("stop, disarm, reset, state, fault <name> [off], release, quit")
    try:
        while True:
            try:
                words = shlex.split(input("gca> "))
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not words:
                continue
            command, *values = words
            try:
                if command in ("quit", "exit"):
                    break
                if command in ("configure", "activate"):
                    getattr(client, command)()
                elif command == "claim":
                    client.claim(values[0] if values else "MANUAL")
                elif command == "reference":
                    client.manage(
                        "set_reference",
                        linear_mm=float(values[0]) if values else 0.0,
                        rotation_deg=float(values[1]) if len(values) > 1 else 0.0,
                    )
                elif command == "move":
                    result = client.move(
                        linear_mm=float(values[0]),
                        rotation_deg=float(values[1]),
                        relative="relative" in values[2:],
                        hold="hold" in values[2:],
                    )
                    print(json.dumps(_result_json(result), ensure_ascii=False))
                elif command == "jog":
                    client.jog(float(values[0]), float(values[1]), float(values[2]))
                elif command in ("arm", "stop", "disarm", "reset"):
                    operation = {"stop": "request_stop", "reset": "reset_fault"}.get(
                        command, command
                    )
                    client.manage(operation)
                    if command == "disarm":
                        client.wait_state(
                            lambda state: not state.armed and state.stop.progress == "CONFIRMED"
                        )
                    elif command == "reset":
                        client.wait_state(lambda state: state.state == "DISARMED")
                elif command == "fault":
                    client.fault(values[0], len(values) < 2 or values[1] != "off")
                elif command == "release":
                    client.release()
                elif command in ("state", "status"):
                    state = client.wait_state(lambda _: True)
                    print(
                        json.dumps(
                            {
                                "state": state.state,
                                "armed": state.armed,
                                "linear_mm": state.linear_position_m * 1000,
                                "rotation_estimated_deg": math.degrees(
                                    state.rotation_estimated_rad
                                ),
                                "stop": state.stop.progress,
                                "fault": state.fault_code,
                            }
                        )
                    )
                else:
                    raise ValueError("Unknown command")
                if client.renewal_error:
                    print("Control renewal failed: " + client.renewal_error)
                else:
                    print("OK")
            except (RuntimeError, ValueError, IndexError, TimeoutError) as exc:
                print(f"{type(exc).__name__}: {exc}")
    finally:
        if client.credentials is not None:
            try:
                client.manage("request_stop")
                client.manage("disarm")
                client.wait_state(
                    lambda state: not state.armed and state.stop.progress == "CONFIRMED"
                )
            except Exception as exc:
                client.cleanup_errors.append("Shell stop/disarm: " + str(exc))
            try:
                client.release()
            except Exception as exc:
                client.cleanup_errors.append("Shell release: " + str(exc))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-file", default=str(Path.home() / ".cache/gca/session.json"))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("shell", help="Interactive renewable control session")
    sub.add_parser("position-shell", help="Persistent AUTO client for single-axis position control")
    demo = sub.add_parser("demo")
    demo.add_argument(
        "--scenario", choices=["basic", "cancel", "jog", "hold", "faults"], default="basic"
    )
    demo.add_argument("--output")
    for command in (
        "configure",
        "activate",
        "claim",
        "reference",
        "arm",
        "stop",
        "disarm",
        "reset",
        "release",
        "status",
    ):
        cmd = sub.add_parser(command)
        if command == "claim":
            cmd.add_argument("--mode", choices=["AUTO", "MANUAL", "MAINTENANCE"], default="AUTO")
    move = sub.add_parser("move")
    move.add_argument("--linear-mm", type=float, default=5.0)
    move.add_argument("--rotation-deg", type=float, default=20.0)
    move.add_argument("--axis-mask", type=int, default=3)
    move.add_argument("--relative", action="store_true")
    move.add_argument("--hold", action="store_true")
    move.add_argument("--cancel-after", type=float)
    jog = sub.add_parser("jog")
    jog.add_argument("--linear-mm-s", type=float, default=2.0)
    jog.add_argument("--rotation-deg-s", type=float, default=10.0)
    jog.add_argument("--duration", type=float, default=0.5)
    fault = sub.add_parser("fault")
    fault.add_argument("name")
    fault.add_argument("--clear", action="store_true")
    config = sub.add_parser("apply-config")
    config.add_argument("path")
    parsed = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous_term = signal.signal(signal.SIGTERM, interrupted)
    client = GcaClient()
    session_file = Path(parsed.session_file)
    release_on_close = parsed.command in ("demo", "shell", "position-shell")
    try:
        if parsed.command == "position-shell":
            position_shell(client)
            report = {"accepted": True, "operation": "position_shell_closed"}
        elif parsed.command == "shell":
            interactive_shell(client)
            report = {"accepted": True, "operation": "shell_closed"}
        elif parsed.command == "demo":
            report = run_demo(client, parsed.scenario, parsed.output)
        elif parsed.command in ("configure", "activate"):
            getattr(client, parsed.command)()
            report = {"accepted": True, "operation": parsed.command}
        elif parsed.command == "status":
            state = client.wait_state(lambda _: True)
            report = state_summary(state)
        else:
            if parsed.command == "claim":
                client.claim(parsed.mode, auto_renew=False)
                report = {"accepted": True, "mode": parsed.mode, "lease_s": 4.0}
            else:
                client.credentials = Credentials(**json.loads(session_file.read_text()))
                if parsed.command not in ("release", "stop", "disarm"):
                    client._start_renewal()
                if parsed.command == "move":
                    report = _result_json(
                        client.move(
                            linear_mm=parsed.linear_mm,
                            rotation_deg=parsed.rotation_deg,
                            axis_mask=parsed.axis_mask,
                            relative=parsed.relative,
                            hold=parsed.hold,
                            cancel_after=parsed.cancel_after,
                        )
                    )
                elif parsed.command == "jog":
                    client.jog(parsed.linear_mm_s, parsed.rotation_deg_s, parsed.duration)
                    report = {"accepted": True}
                elif parsed.command == "release":
                    response = client.release()
                    report = {"accepted": response.accepted, "error_code": response.error_code}
                    session_file.unlink(missing_ok=True)
                elif parsed.command == "fault":
                    response = client.fault(parsed.name, not parsed.clear)
                    report = {"accepted": response.accepted, "error_code": response.error_code}
                elif parsed.command == "apply-config":
                    response = client.apply_config(Path(parsed.path).read_text())
                    report = {
                        "accepted": response.accepted,
                        "requested_revision": response.requested_revision,
                        "applied_revision": response.applied_revision,
                    }
                else:
                    operation = {
                        "reference": "set_reference",
                        "stop": "request_stop",
                        "reset": "reset_fault",
                    }.get(parsed.command, parsed.command)
                    response = client.manage(operation)
                    report = {"accepted": response.accepted, "error_code": response.error_code}
            if client.credentials is not None:
                with client._lock:
                    save_credentials(session_file, client.credentials)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except Exception as exc:
        if parsed.command == "demo" and parsed.output:
            directory = Path(parsed.output)
            directory.mkdir(parents=True, exist_ok=True)
            (directory / "report.json").write_text(
                json.dumps(
                    {
                        "scenario": parsed.scenario,
                        "passed": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                    indent=2,
                )
                + "\n"
            )
        raise SystemExit(f"{type(exc).__name__}: {exc}") from exc
    finally:
        previous_error = sys.exc_info()[0] is not None
        cleanup_errors = client.close(release=release_on_close or previous_error)
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
        if cleanup_errors and not previous_error:
            raise SystemExit("Control cleanup incomplete; see warning above")


def state_summary(state):
    return {
        "state": state.state,
        "control_policy": state.control_policy,
        "device_mode": state.mode,
        "session_id": state.session_id,
        "armed": state.armed,
        "reference_id": state.reference_id,
        "linear_reference_valid": state.linear_reference_valid,
        "linear_valid": state.linear_valid,
        "linear_velocity_valid": state.linear_velocity_valid,
        "linear_mm": state.linear_position_m * 1000,
        "linear_velocity_mm_s": state.linear_velocity_m_s * 1000,
        "rotation_estimated_deg": math.degrees(state.rotation_estimated_rad),
        "stop": state.stop.progress,
        "fault": state.fault_code,
    }


def position_shell(client):
    """One explicit AUTO session; nonblocking submission keeps cancel and STOP usable."""
    print("configure, activate, claim, reference <mm>, arm, state")
    print("move <absolute_mm> [speed_mm/s=2] [accel_mm/s²=5] [timeout_s=10]")
    print("relative <delta_mm> [speed] [accel] [timeout], wait, cancel, stop")
    print("disarm, reset, release, quit (stops, disarms and releases)")
    result_future = None
    handle = None
    result_timeout = 15.0
    while True:
        try:
            words = shlex.split(input("position> "))
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not words:
            continue
        command, *values = words
        try:
            if command in ("quit", "exit"):
                return
            if command in ("configure", "activate"):
                getattr(client, command)()
            elif command == "claim":
                client.claim("AUTO")
            elif command == "reference":
                client.set_linear_reference(float(values[0]))
            elif command in ("move", "relative"):
                if result_future is not None and not result_future.done():
                    raise RuntimeError("A motion is active; wait, cancel or stop first")
                target = float(values[0])
                speed = float(values[1]) if len(values) > 1 else 2.0
                acceleration = float(values[2]) if len(values) > 2 else 5.0
                timeout = float(values[3]) if len(values) > 3 else 10.0
                goal = client.make_linear_goal(
                    target,
                    relative=command == "relative",
                    speed_mm_s=speed,
                    acceleration_mm_s2=acceleration,
                    timeout_s=timeout,
                )
                handle = client.send_goal(goal)
                if not handle.accepted:
                    raise RuntimeError("Linear motion rejected")
                result_future = handle.get_result_async()
                result_timeout = goal.execution_timeout_s + 5.0
                print(json.dumps({"accepted": True, "command_id": goal.context.command_id}))
            elif command in ("wait", "cancel"):
                if result_future is None:
                    raise RuntimeError("No submitted motion")
                if command == "cancel" and not result_future.done():
                    client.wait(handle.cancel_goal_async())
                result = client.wait(result_future, result_timeout).result
                print(json.dumps(_result_json(result), ensure_ascii=False))
            elif command in ("arm", "stop", "disarm", "reset"):
                operation = {"stop": "request_stop", "reset": "reset_fault"}.get(command, command)
                client.manage(operation, axis_mask=1)
                if command in ("stop", "disarm"):
                    client.wait_state(lambda state: state.stop.progress == "CONFIRMED")
            elif command == "release":
                client.manage("request_stop", axis_mask=1)
                client.wait_state(lambda state: state.stop.progress == "CONFIRMED")
                client.manage("disarm", axis_mask=1)
                client.wait_state(lambda state: not state.armed)
                client.release()
            elif command in ("state", "status"):
                print(
                    json.dumps(state_summary(client.wait_state(lambda _: True)), ensure_ascii=False)
                )
            else:
                raise ValueError("Unknown position command")
            if client.renewal_error:
                raise RuntimeError("Control renewal failed: " + client.renewal_error)
        except (RuntimeError, ValueError, IndexError, TimeoutError) as exc:
            print(f"{type(exc).__name__}: {exc}")
