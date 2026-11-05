"""Installed ROS client and reproducible simulation demonstrations.

The client never owns device resources. All motion, references and permissions
pass through system_coordinator. The standard lifecycle service is used only
for administrative configure/activate, which never ARM the mechanism.
"""

import argparse
import json
import math
import os
import shlex
import signal
import sys
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import rclpy
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import CommandContext, GcaState, JogCommand
from gca_system_interfaces.srv import (
    ApplyConfig,
    ControlSession,
    GetCommandStatus,
    InjectFault,
    ManageDevice,
)
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


@dataclass
class Credentials:
    session_id: str
    generation: int
    token: str
    sequence: int = 1


def finite_float(value, name):
    """Normalize ROS float64 inputs before serialization; never send integer storage."""
    if isinstance(value, bool):
        raise ValueError(name + " must be numeric")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(name + " must be finite")
    return converted


def save_credentials(path, credentials):
    """Atomically replace a private token file; never create a readable intermediate."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptor, temporary = tempfile.mkstemp(prefix=".gca-session-", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(vars(credentials), stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


class GcaClient:
    """Threaded ROS client; synchronous waits occur outside executor callbacks."""

    def __init__(self, timeout_s=5.0):
        self.node = Node("gca_cli_" + uuid.uuid4().hex[:8])
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        self._group = ReentrantCallbackGroup()
        self.timeout_s = timeout_s
        self.credentials = None
        self._lock = threading.RLock()
        self._latest = None
        self.samples = []
        self._clients = {}
        self._renew_stop = threading.Event()
        self._renew_thread = None
        self.renewal_error = None
        self.cleanup_errors = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            GcaState, "/system/state", self._state, qos, callback_group=self._group
        )
        self._action = ActionClient(
            self.node, ExecuteMotion, "/system/execute_motion", callback_group=self._group
        )
        self._jog_pub = self.node.create_publisher(
            JogCommand,
            "/system/manual_target",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

    def _state(self, msg):
        self._latest = msg
        self.samples.append(
            {
                "received_monotonic_s": time.monotonic(),
                "state": msg.state,
                "position_mm": msg.linear_position_m * 1000,
                "velocity_mm_s": msg.linear_velocity_m_s * 1000,
                "rotation_estimated_deg": math.degrees(msg.rotation_estimated_rad),
                "rotation_estimate_valid": msg.rotation_estimate_valid,
                "sample_seq": msg.sample_seq,
                "fault_code": msg.fault_code,
                "stop_progress": msg.stop.progress,
                "output_generation": msg.output_generation,
                "target_linear_mm": msg.target_linear_m * 1000,
                "reference_linear_mm": msg.reference_linear_m * 1000,
                "target_rotation_deg": math.degrees(msg.target_rotation_rad),
                "reference_rotation_deg": math.degrees(msg.reference_rotation_rad),
                "requested_velocity_mm_s": msg.requested_velocity_m_s * 1000,
                "applied_velocity_mm_s": msg.applied_velocity_m_s * 1000,
                "output_seq": msg.output_seq,
            }
        )
        if len(self.samples) > 20000:
            del self.samples[:1000]

    @property
    def state(self):
        return self._latest

    def wait(self, future, timeout_s=None):
        complete = threading.Event()
        future.add_done_callback(lambda _: complete.set())
        if not complete.wait(self.timeout_s if timeout_s is None else timeout_s):
            raise TimeoutError("ROS request exceeded client deadline")
        return future.result()

    def call(self, service_type, name, request, timeout_s=None):
        client = self._clients.get(name)
        if client is None:
            client = self.node.create_client(service_type, name, callback_group=self._group)
            self._clients[name] = client
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            raise TimeoutError(f"Service unavailable: {name}")
        return self.wait(client.call_async(request), timeout_s)

    def lifecycle(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        response = self.call(ChangeState, "/gca_control_node/change_state", request)
        if not response.success:
            raise RuntimeError(f"Lifecycle transition {transition} rejected")
        return response

    def configure(self):
        state = self.call(
            GetState, "/gca_control_node/get_state", GetState.Request()
        ).current_state.id
        if state == 1:
            self.lifecycle(1)
        elif state not in (2, 3):
            raise RuntimeError(f"Cannot configure lifecycle state {state}")

    def activate(self):
        state = self.call(
            GetState, "/gca_control_node/get_state", GetState.Request()
        ).current_state.id
        if state == 2:
            self.lifecycle(3)
        elif state != 3:
            raise RuntimeError(f"Cannot activate lifecycle state {state}")

    def context(self, command_id=None):
        with self._lock:
            if self.credentials is None:
                raise RuntimeError("claim control before sending commands")
            self.credentials.sequence += 1
            return CommandContext(
                session_id=self.credentials.session_id,
                generation=self.credentials.generation,
                sequence=self.credentials.sequence,
                task_id=str(uuid.uuid4()),
                command_id=command_id or str(uuid.uuid4()),
            )

    def claim(self, mode="AUTO", lease_s=4.0, auto_renew=True):
        lease_s = finite_float(lease_s, "lease_s")
        request = ControlSession.Request(owner="gca_cli", mode=mode, lease_s=lease_s)
        response = self.call(ControlSession, "/system/claim_control", request)
        self._require(response)
        self.credentials = Credentials(
            response.context.session_id,
            response.context.generation,
            response.token,
            response.context.sequence,
        )
        self.renewal_error = None
        if auto_renew:
            self._start_renewal(lease_s)
        return response

    def _start_renewal(self, lease_s=4.0):
        lease_s = finite_float(lease_s, "lease_s")
        self._renew_stop.clear()

        def renew():
            while not self._renew_stop.wait(0.35):
                try:
                    with self._lock:
                        request = ControlSession.Request(
                            context=self.context(), token=self.credentials.token, lease_s=lease_s
                        )
                        response = self._require(
                            self.call(ControlSession, "/system/renew_control", request)
                        )
                        self.credentials.generation = response.context.generation
                        self.credentials.sequence = max(
                            self.credentials.sequence, response.context.sequence
                        )
                except Exception as exc:
                    self.renewal_error = str(exc)
                    self._renew_stop.set()

        self._renew_thread = threading.Thread(target=renew, daemon=True)
        self._renew_thread.start()

    @staticmethod
    def _require(response):
        if not response.accepted:
            raise RuntimeError(response.error_code + ": " + response.message)
        return response

    def manage(self, operation, axis_mask=3, linear_mm=0.0, rotation_deg=0.0):
        # Serialize source renewal with explicit generation rotation on re-ARM.
        with self._lock:
            request = ManageDevice.Request(
                context=self.context(),
                token=self.credentials.token,
                axis_mask=axis_mask,
                linear_m=linear_mm / 1000,
                rotation_rad=math.radians(rotation_deg),
            )
            response = self.call(ManageDevice, "/system/" + operation, request)
            if response.state.generation:
                self.credentials.generation = response.state.generation
            return self._require(response)

    def wait_state(self, predicate, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.state is not None and predicate(self.state):
                return self.state
            time.sleep(0.02)
        raise TimeoutError("State condition not reached before deadline")

    def make_goal(
        self,
        linear_mm=5.0,
        rotation_deg=20.0,
        axis_mask=3,
        relative=False,
        hold=False,
        timeout_s=15.0,
        max_linear_mm_s=8.0,
        max_rotation_deg_s=60.0,
        command_id=None,
    ):
        if self.state is None:
            self.wait_state(lambda _: True)
        goal = ExecuteMotion.Goal()
        goal.context = self.context(command_id)
        goal.target.axis_mask = axis_mask
        goal.target.mode = 2 if relative else 1
        goal.target.linear_m = finite_float(linear_mm, "linear_mm") / 1000.0
        goal.target.rotation_rad = math.radians(finite_float(rotation_deg, "rotation_deg"))
        goal.target.reference_id = self.state.reference_id
        goal.limits.max_linear_m_s = finite_float(max_linear_mm_s, "max_linear_mm_s") / 1000.0
        goal.limits.max_linear_m_s2 = 0.05
        goal.limits.max_rotation_rad_s = math.radians(
            finite_float(max_rotation_deg_s, "max_rotation_deg_s")
        )
        goal.limits.max_rotation_rad_s2 = math.radians(240.0)
        goal.end_policy = 2 if hold else 1
        goal.admission_timeout_s = 0.5
        goal.execution_timeout_s = finite_float(timeout_s, "timeout_s")
        goal.token = self.motion_permit(goal.context, goal.admission_timeout_s).token
        return goal

    def motion_permit(self, context, ttl_s=0.5):
        request = ControlSession.Request(
            context=context,
            token=self.credentials.token,
            lease_s=finite_float(ttl_s, "permit_ttl_s"),
        )
        return self._require(self.call(ControlSession, "/system/get_motion_permit", request))

    def send_goal(self, goal):
        if not self._action.wait_for_server(timeout_sec=self.timeout_s):
            raise TimeoutError("System motion action unavailable")
        return self.wait(self._action.send_goal_async(goal))

    def move(self, cancel_after=None, **kwargs):
        goal = self.make_goal(**kwargs)
        handle = self.send_goal(goal)
        if not handle.accepted:
            request = GetCommandStatus.Request(
                session_id=self.credentials.session_id,
                token=self.credentials.token,
                command_id=goal.context.command_id,
            )
            status = self.call(GetCommandStatus, "/system/get_command_status", request)
            raise RuntimeError("Motion rejected: " + status.error_code)
        if cancel_after is not None:
            time.sleep(cancel_after)
            self.wait(handle.cancel_goal_async())
        wrapped = self.wait(handle.get_result_async(), goal.execution_timeout_s + 5.0)
        return wrapped.result

    def jog(self, linear_mm_s=2.0, rotation_deg_s=10.0, duration_s=0.5, deadman=True):
        linear_mm_s = finite_float(linear_mm_s, "linear_mm_s")
        rotation_deg_s = finite_float(rotation_deg_s, "rotation_deg_s")
        duration_s = finite_float(duration_s, "duration_s")
        until = time.monotonic() + duration_s
        try:
            while time.monotonic() < until:
                context = self.context()
                permit = self.motion_permit(context, 0.15)
                msg = JogCommand(
                    context=context,
                    token=permit.token,
                    axis_mask=3,
                    deadman=bool(deadman),
                    linear_velocity_m_s=linear_mm_s / 1000.0,
                    rotation_velocity_rad_s=math.radians(rotation_deg_s),
                    ttl_s=0.15,
                )
                self._jog_pub.publish(msg)
                time.sleep(0.04)
        finally:
            # Deadman release is a restricted stop, independent from an expired
            # motion nonce. Keep the ROS context alive until it is published.
            already_failing = sys.exc_info()[0] is not None
            try:
                context = self.context()
                msg = JogCommand(
                    context=context,
                    token=self.credentials.token,
                    axis_mask=3,
                    deadman=False,
                    ttl_s=0.15,
                )
                self._jog_pub.publish(msg)
            except Exception as exc:
                self.cleanup_errors.append("Deadman release: " + str(exc))
                if not already_failing:
                    raise

    def fault(self, name, enabled=True):
        request = InjectFault.Request(
            context=self.context(), token=self.credentials.token, name=name, enabled=enabled
        )
        return self._require(self.call(InjectFault, "/system/inject_fault", request))

    def apply_config(self, configuration_yaml):
        request = ApplyConfig.Request(
            context=self.context(),
            token=self.credentials.token,
            configuration_yaml=configuration_yaml,
        )
        return self._require(self.call(ApplyConfig, "/system/apply_config", request))

    def release(self):
        self._renew_stop.set()
        if self._renew_thread is not None:
            self._renew_thread.join(timeout=self.timeout_s + 0.1)
            self._renew_thread = None
        if self.credentials is not None:
            request = ControlSession.Request(context=self.context(), token=self.credentials.token)
            response = self._require(self.call(ControlSession, "/system/release_control", request))
            self.credentials = None
            return response

    def close(self, release=True):
        try:
            if release:
                if self.credentials is not None:
                    try:
                        self.manage("request_stop")
                        self.manage("disarm")
                        self.wait_state(
                            lambda state: not state.armed and state.stop.progress == "CONFIRMED"
                        )
                    except Exception as exc:
                        self.cleanup_errors.append("Stop/disarm: " + str(exc))
                        self.node.get_logger().warning(
                            "Control cleanup incomplete: " + self.cleanup_errors[-1]
                        )
                    self.release()
            else:
                self._renew_stop.set()
                if self._renew_thread is not None:
                    self._renew_thread.join(timeout=self.timeout_s + 0.1)
        except Exception as exc:
            # Preserve an earlier motion/test failure while making cleanup
            # failure inspectable to callers and operators.
            self.cleanup_errors.append(f"{type(exc).__name__}: {exc}")
            self.node.get_logger().warning("Control cleanup incomplete: " + self.cleanup_errors[-1])
        finally:
            self.executor.shutdown(timeout_sec=2.0)
            self._thread.join(timeout=2.0)
            self.node.destroy_node()
        return self.cleanup_errors


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
    release_on_close = parsed.command in ("demo", "shell")
    try:
        if parsed.command == "shell":
            interactive_shell(client)
            report = {"accepted": True, "operation": "shell_closed"}
        elif parsed.command == "demo":
            report = run_demo(client, parsed.scenario, parsed.output)
        elif parsed.command in ("configure", "activate"):
            getattr(client, parsed.command)()
            report = {"accepted": True, "operation": parsed.command}
        elif parsed.command == "status":
            state = client.wait_state(lambda _: True)
            report = {
                "state": state.state,
                "armed": state.armed,
                "linear_mm": state.linear_position_m * 1000,
                "rotation_estimated_deg": math.degrees(state.rotation_estimated_rad),
                "stop": state.stop.progress,
                "fault": state.fault_code,
            }
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
