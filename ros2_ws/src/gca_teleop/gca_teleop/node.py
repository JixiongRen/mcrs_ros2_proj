"""Installed joystick owner. Launch is idle; all lifecycle/ARM actions are explicit."""

from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
import uuid
from dataclasses import asdict, replace
from pathlib import Path
from queue import Empty, Full, Queue

import rclpy
from ament_index_python.packages import get_package_share_directory
from gca.observability.recorder import AsyncRecorder
from gca_ros.conversion import quality_to_ros
from gca_system_interfaces.msg import JogCommand, TeleopState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from .config import finite, load_profile
from .dispatch import LatestDispatcher
from .input.joydev import NativeJoydev
from .input.scripted import ScriptedInput
from .management import ManagementServer, default_socket
from .model import InputHealth, Intent
from .session import ManualSession
from .state_machine import TeleopPipeline


class TeleopNode(Node):
    def __init__(self):
        super().__init__("gca_teleop")
        default = str(
            Path(get_package_share_directory("gca_teleop")) / "config/beitong_unverified.yaml"
        )
        self.declare_parameter("profile_file", default)
        self.declare_parameter("script_file", "")
        self.declare_parameter("socket_path", str(default_socket()))
        self.declare_parameter("record_directory", "")
        self.run_id = uuid.uuid4().hex
        self.profile = load_profile(self.get_parameter("profile_file").value)
        script = self.get_parameter("script_file").value
        if script:
            if self.profile.source != "scripted":
                raise ValueError("SCRIPT_FILE_REQUIRES_SCRIPTED_PROFILE")
            self.profile = replace(self.profile, script_file=str(Path(script).resolve()))
        self._closing = threading.Event()
        self._pipeline_lock = threading.RLock()
        self._management_lock = threading.Lock()
        self._pipeline = TeleopPipeline(self.profile)
        self._intent = Intent()
        self._health = InputHealth(False, False, 0, "", "", "STARTING")
        self._last_frame = None
        self._input_commands = Queue(8)
        self._reader = None
        self._next_reopen_ns = 0
        self._last_error = ""
        self._dispatch_after_ns = 0
        self._script_started_ns = 0
        records = self.get_parameter("record_directory").value
        self._recorder = AsyncRecorder(records) if records else None
        self._session = ManualSession(self, notify=self._record)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
        )
        self._jog_pub = self.create_publisher(JogCommand, "/system/manual_target", qos)
        self._state_pub = self.create_publisher(TeleopState, "/teleop/state", qos)
        self._dispatcher = LatestDispatcher(
            self._session, self._publish_jog, notify=self._record, failure=self._dispatch_failed
        )
        self._dispatch_group = MutuallyExclusiveCallbackGroup()
        self._observation_group = ReentrantCallbackGroup()
        # Constant 250Hz scheduler allows profile changes without replacing a
        # timer during callbacks. Actual dispatch follows the profile's bound.
        self._timer = self.create_timer(0.004, self._tick, callback_group=self._dispatch_group)
        self._state_timer = self.create_timer(
            0.1, self._publish_state, callback_group=self._observation_group
        )
        self._server = ManagementServer(Path(self.get_parameter("socket_path").value), self._manage)
        self._server.start()
        self._input_thread = threading.Thread(
            target=self._input_loop, name="teleop-input", daemon=True
        )
        self._input_thread.start()
        self._record(
            {
                "kind": "startup",
                "profile": asdict(self.profile),
                "profile_sha256": self.profile.fingerprint,
                "physical_mapping": "NOT_RUN"
                if self.profile.source == "scripted"
                else "EVIDENCE_REQUIRED",
                "monotonic_ns": time.monotonic_ns(),
            }
        )

    def _record(self, event):
        if self._recorder is not None:
            self._recorder.submit(dict(event, run_id=self.run_id))

    def _new_reader(self):
        if self.profile.source == "scripted":
            return ScriptedInput(self.profile, backend="simulation")
        return NativeJoydev(self.profile)

    def _input_loop(self):
        try:
            while not self._closing.is_set():
                start = time.monotonic()
                try:
                    self._drain_input_commands()
                    now = time.monotonic_ns()
                    if self._reader is None:
                        if now < self._next_reopen_ns:
                            self._closing.wait(0.02)
                            continue
                        reader = self._new_reader()
                        try:
                            identity = reader.open()
                        except BaseException:
                            reader.close()
                            raise
                        self._reader = reader
                        self._record(
                            {
                                "kind": "input_open",
                                "identity": asdict(identity),
                                "monotonic_ns": now,
                            }
                        )
                    frames = self._reader.poll(self.profile.poll_timeout_s)
                    health = self._reader.health()
                    with self._pipeline_lock:
                        self._health = health
                    for frame in frames:
                        gate = self._session.gate()
                        with self._pipeline_lock:
                            intent = self._pipeline.process(
                                frame, health, time.monotonic_ns(), **gate
                            )
                            self._last_frame = frame
                            self._accept_intent(intent)
                        self._record(
                            {
                                "kind": "input",
                                "frame": asdict(frame),
                                "health": asdict(health),
                                "intent": asdict(intent),
                                "monotonic_ns": time.monotonic_ns(),
                            }
                        )
                    if not frames or not health.connected or health.reason:
                        with self._pipeline_lock:
                            intent = self._pipeline.tick(
                                health, time.monotonic_ns(), **self._session.gate()
                            )
                            self._accept_intent(intent)
                    # Native disconnect must be reopened as the same identified
                    # source. A script disconnect stays failed until explicit start.
                    if not health.connected and self.profile.source == "joydev":
                        self._reader.close()
                        self._reader = None
                        self._next_reopen_ns = now + 1_000_000_000
                except Exception as exc:
                    reason = getattr(exc, "code", type(exc).__name__ + ":" + str(exc)[:160])
                    with self._pipeline_lock:
                        self._health = replace(
                            self._health, connected=False, initialized=False, reason=reason
                        )
                    self._last_error = reason
                    self._session.request_stop(reason)
                    self._dispatcher.invalidate()
                    if self._reader is not None:
                        self._reader.close()
                        self._reader = None
                    self._next_reopen_ns = time.monotonic_ns() + 1_000_000_000
                # Scripted poll is nonblocking; bound it to an actual sampling
                # cadence rather than manufacturing unlimited observations.
                self._closing.wait(
                    max(0.0, self.profile.poll_timeout_s - (time.monotonic() - start))
                )
        finally:
            if self._reader is not None:
                self._reader.close()
            while True:
                try:
                    _, _, _, _, result, complete = self._input_commands.get_nowait()
                except Empty:
                    break
                result["error"] = "INPUT_CLOSED"
                complete.set()

    def _drain_input_commands(self):
        for _ in range(8):
            try:
                operation, value, expected, deadline, result, complete = (
                    self._input_commands.get_nowait()
                )
            except Empty:
                return
            try:
                if time.monotonic_ns() >= deadline:
                    raise TimeoutError("INPUT_MANAGEMENT_EXPIRED")
                self._check_expected(
                    {"op": "input-config", "expected": expected}, self.status()["expected"]
                )
                if operation == "profile":
                    profile, pipeline = value
                    state = self._session.state
                    safe = (
                        state is not None
                        and state.stop.progress == "CONFIRMED"
                        and state.state == "DISARMED"
                    )
                    if not safe or state.armed:
                        raise ValueError("CONFIG_REQUIRES_STOPPED_DISARMED")
                    with self._pipeline_lock:
                        self._pipeline = pipeline
                        self.profile = profile
                        self._intent = Intent(reason="PROFILE_CHANGED")
                    self._dispatcher.invalidate()
                    if self._reader is not None:
                        self._reader.close()
                    self._reader = None
                    self._next_reopen_ns = 0
                elif operation == "start":
                    if (
                        not isinstance(self._reader, ScriptedInput)
                        or self._session.backend != "simulation"
                    ):
                        raise ValueError("SCRIPT_REQUIRES_CONFIRMED_SIMULATION")
                    self._dispatcher.invalidate()
                    with self._pipeline_lock:
                        self._pipeline.reset("SCRIPT_RESTART")
                    self._reader.start()
                    with self._pipeline_lock:
                        self._health = self._reader.health()
                        self._script_started_ns = self._health.reader_heartbeat_ns
                    self._record(
                        {
                            "kind": "script_started",
                            "monotonic_ns": self._script_started_ns,
                            "input_epoch": self._health.input_epoch,
                        }
                    )
                elif operation == "stop":
                    if not isinstance(self._reader, ScriptedInput):
                        raise ValueError("NOT_SCRIPTED_INPUT")
                    self._reader.stop()
                result["accepted"] = True
            except Exception as exc:
                result["error"] = str(exc)
            finally:
                complete.set()

    def _input_request(self, operation, value=None):
        result, complete = {}, threading.Event()
        try:
            self._input_commands.put_nowait(
                (
                    operation,
                    value,
                    self.status()["expected"],
                    time.monotonic_ns() + 900_000_000,
                    result,
                    complete,
                )
            )
        except Full:
            raise RuntimeError("INPUT_MANAGEMENT_BUSY") from None
        if not complete.wait(1.0):
            raise TimeoutError("INPUT_MANAGEMENT_TIMEOUT")
        if "error" in result:
            raise RuntimeError(result["error"])

    def _accept_intent(self, intent):
        with self._pipeline_lock:
            was_active = self._intent.action == "JOG"
            self._intent = intent
        if intent.action == "STOP":
            active = was_active or self._dispatcher.pending_count or self._dispatcher.inflight_count
            benign = intent.reason in (
                "STARTUP",
                "CONTEXT_CHANGED",
                "NOT_ARMED_READY",
                "BASELINE_PENDING",
                "SCRIPT_PAUSED",
            )
            # Initial idle/administrative gates cannot cancel every explicit
            # ARM. Physical B and active-motion failures always request STOP.
            if active or not benign:
                self._session.request_stop(intent.reason)
            self._dispatcher.invalidate()
            self._record(
                {
                    "kind": "gate_stop",
                    "reason": intent.reason,
                    "motion_was_active": bool(active),
                    "monotonic_ns": time.monotonic_ns(),
                }
            )

    def _tick(self):
        if self._closing.is_set():
            return
        guard = self._dispatcher.guard()
        gate = self._session.gate()
        with self._pipeline_lock:
            # Date the health snapshot protected by this lock. The input thread
            # may advance its heartbeat while guard/gate/lock acquisition waits;
            # an earlier timestamp would misclassify that new sample as future.
            now = time.monotonic_ns()
            intent = self._pipeline.tick(self._health, now, **gate)
            self._accept_intent(intent)
            if now < self._dispatch_after_ns:
                return
            self._dispatch_after_ns = now + int(1e9 / self.profile.dispatch_hz)
            if intent.action == "JOG":
                if (
                    intent.quality.source_kind == "SCRIPT_SAMPLE"
                    and self._session.backend != "simulation"
                ):
                    self._dispatch_failed("SCRIPT_REQUIRES_CONFIRMED_SIMULATION")
                    return
                self._dispatcher.offer(intent, guard)
        # DDS/permit work is outside the pipeline mutex. An intervening STOP
        # invalidates the original guard, including work not yet in flight.
        self._dispatcher.pump()

    def _dispatch_failed(self, reason):
        self._session.request_stop(reason)
        self._dispatcher.invalidate()
        with self._pipeline_lock:
            self._pipeline.reset(reason)
            self._intent = Intent(reason=reason)
        self._last_error = reason

    def _publish_jog(self, intent, context, token):
        remaining = (intent.quality.valid_until_ns - time.monotonic_ns()) / 1e9
        if remaining <= 0:
            raise ValueError("INPUT_EXPIRED_BEFORE_PUBLISH")
        self._jog_pub.publish(
            JogCommand(
                context=context,
                token=token,
                axis_mask=3,
                deadman=True,
                linear_velocity_m_s=float(intent.linear_mm_s) / 1000.0,
                rotation_velocity_rad_s=math.radians(intent.rotation_deg_s),
                ttl_s=min(0.15, remaining),
                quality=quality_to_ros(intent.quality),
            )
        )

    def status(self):
        state = self._session.state
        session_id, generation = self._session.identity()
        with self._pipeline_lock:
            intent, health, profile = self._intent, self._health, self.profile
        expected = dict(
            owner_instance=self.run_id,
            input_epoch=health.input_epoch,
            session_id=session_id,
            generation=generation,
            reference_id=state.reference_id if state else "",
            config_revision=state.applied_config_revision if state else "",
        )
        return {
            "accepted": True,
            "expected": expected,
            "source": profile.source,
            "script_started_ns": self._script_started_ns,
            "mode": profile.mode,
            "backend": self._session.backend,
            "profile_id": profile.profile_id,
            "profile_sha256": profile.fingerprint,
            "intent": asdict(intent),
            "health": asdict(health),
            "session_owned": bool(session_id),
            "device_state": state.state if state else "UNKNOWN",
            "armed": bool(state and state.armed),
            "stop_progress": state.stop.progress if state else "UNKNOWN",
            "stop_confirmed": self._session.latest_stop_confirmed,
            "linear_valid": bool(state and state.linear_valid),
            "rotation_reference_valid": bool(state and state.rotation_reference_valid),
            "position_mm": state.linear_position_m * 1000 if state else 0.0,
            "rotation_estimated_deg": math.degrees(state.rotation_estimated_rad) if state else 0.0,
            "pending_count": self._dispatcher.pending_count,
            "inflight_count": self._dispatcher.inflight_count,
            "dropped_count": self._dispatcher.dropped_count,
            "sent_count": self._dispatcher.sent_count,
            "stop_epoch": self._session.stop_epoch,
            "last_error": self._session.error or self._last_error,
            "recorder": asdict(self._recorder.status()) if self._recorder else None,
        }

    def _check_expected(self, request, current):
        expected = request.get("expected", {})
        if set(expected) != set(current):
            raise ValueError("EXPECTED_STATE_FIELDS_REQUIRED")
        # STOP is scoped to current ownership/input; ordinary reference/config
        # revisions cannot make B unusable while ARM changes generation.
        keys = (
            ("owner_instance", "input_epoch", "session_id")
            if request["op"] == "stop"
            else current.keys()
        )
        if any(
            type(expected[k]) is not type(current[k]) or expected[k] != current[k] for k in keys
        ):
            raise ValueError("STALE_MANAGEMENT_CONTEXT")

    def _manage(self, request):
        if request["op"] in ("status", "stop", "simulation-fault", "script-start", "script-stop"):
            return self._manage_impl(request)
        if not self._management_lock.acquire(blocking=False):
            raise RuntimeError("MANAGEMENT_BUSY")
        try:
            return self._manage_impl(request)
        finally:
            self._management_lock.release()

    def _manage_impl(self, request):
        operation, args = request["op"], request.get("args", {})
        if operation == "status":
            if args:
                raise ValueError("UNEXPECTED_ARGUMENTS")
            return self.status()
        self._check_expected(request, self.status()["expected"])
        allowed = {
            "reference": {"linear_mm", "rotation_deg"},
            "input-config": {"path"},
            "simulation-fault": {"name", "enabled"},
        }.get(operation, set())
        if set(args) - allowed:
            raise ValueError("UNEXPECTED_ARGUMENTS")
        if operation == "stop":
            self._session.request_stop("OPERATOR_STOP")
            self._dispatcher.invalidate()
            with self._pipeline_lock:
                self._pipeline.reset("OPERATOR_STOP")
                self._intent = Intent(reason="OPERATOR_STOP")
        elif operation == "input-config":
            profile = load_profile(args["path"])
            # Evidence files load on the ordinary management worker. Only the
            # already prepared pipeline is exchanged under the input lock.
            self._input_request("profile", (profile, TeleopPipeline(profile)))
        elif operation == "script-start":
            self._input_request("start")
        elif operation == "script-stop":
            self._session.request_stop("SCRIPT_STOP")
            self._dispatcher.invalidate()
            self._input_request("stop")
        elif operation == "simulation-fault":
            self._session.simulation_fault(args["name"], args["enabled"])
        else:
            if operation == "arm" and self._session.backend == "hardware":
                with self._pipeline_lock:
                    error = self._pipeline.hardware_arm_error(self._health, time.monotonic_ns())
                if error:
                    raise ValueError(error)
            self._dispatcher.invalidate()
            with self._pipeline_lock:
                self._pipeline.reset("MANAGEMENT_" + operation.upper())
                self._intent = Intent(reason="MANAGEMENT_" + operation.upper())
            linear, rotation = args.get("linear_mm", 0.0), args.get("rotation_deg", 0.0)
            finite(linear, "linear_mm")
            finite(rotation, "rotation_deg")
            self._session.manage(operation, linear_mm=linear, rotation_deg=rotation)
        self._record(
            {"kind": "management", "operation": operation, "monotonic_ns": time.monotonic_ns()}
        )
        return self.status()

    def _publish_state(self):
        if self._closing.is_set():
            return
        state = self._session.state
        with self._pipeline_lock:
            intent, health = self._intent, self._health
        msg = TeleopState(
            run_id=self.run_id,
            input_epoch=health.input_epoch,
            gate_state=intent.gate,
            gate_reason=intent.reason,
            deadman=intent.deadman,
            b_pressed=intent.b_pressed,
            session_owned=bool(self._session.identity()[0]),
            armed=bool(state and state.armed),
            source_mode=self.profile.mode,
            requested_linear_m_s=float(intent.linear_mm_s) / 1000.0,
            requested_rotation_rad_s=math.radians(intent.rotation_deg_s),
            pending_count=self._dispatcher.pending_count,
            inflight_count=self._dispatcher.inflight_count,
            dropped_count=self._dispatcher.dropped_count,
        )
        msg.stamp = self.get_clock().now().to_msg()
        if intent.quality is not None:
            msg.quality = quality_to_ros(intent.quality)
        if state is not None:
            msg.device_state, msg.stop = state.state, state.stop
            msg.linear_valid, msg.rotation_valid = state.linear_valid, state.rotation_estimate_valid
            msg.reference_id, msg.config_revision = (
                state.reference_id,
                state.applied_config_revision,
            )
            msg.applied_linear_m_s = state.applied_velocity_m_s
            msg.estimated_rotation_rad_s = state.estimated_rotation_velocity_rad_s
            msg.estimated_rotation_velocity_valid = state.rotation_estimate_valid
        self._state_pub.publish(msg)

    def close(self):
        self._closing.set()
        self._session.request_stop("NODE_EXIT")
        self._dispatcher.invalidate()
        self._input_thread.join(1.0)
        server_clean = self._server.close()
        self._session.close()
        self._timer.cancel()
        self._state_timer.cancel()
        clean = server_clean and not self._input_thread.is_alive()
        if self._recorder:
            clean = self._recorder.close(1.0) and clean
        return clean


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    _, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown = threading.Event()
    previous = {
        sig: signal.signal(sig, lambda signum, frame: shutdown.set())
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node, executor, thread = None, None, None
    code = 0
    try:
        node = TeleopNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, name="teleop-executor", daemon=True)
        thread.start()
        while not shutdown.wait(0.05) and rclpy.ok():
            if not thread.is_alive():
                raise RuntimeError("TELEOP_EXECUTOR_EXITED")
    except Exception as exc:
        print(json.dumps({"error": type(exc).__name__ + ": " + str(exc)}), flush=True)
        code = 1
    finally:
        if node is not None and not node.close():
            code = 1
        if executor is not None:
            if not executor.shutdown(timeout_sec=2.0):
                code = 1
        if thread is not None:
            thread.join(2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return code
