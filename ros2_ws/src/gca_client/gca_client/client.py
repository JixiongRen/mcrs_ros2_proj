"""Installed ROS client and reproducible simulation demonstrations.

The client never owns device resources. All motion, references and permissions
pass through system_coordinator. The standard lifecycle service is used only
for administrative configure/activate, which never ARM the mechanism.
"""

import json
import math
import os
import sys
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import CommandContext, GcaState, JogCommand, ManualInputQuality
from gca_system_interfaces.srv import (
    ApplyConfig,
    ControlSession,
    GetCapabilities,
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
            self._require(response)
            if operation == "arm":
                ack = ControlSession.Request(context=self.context(), token=self.credentials.token)
                self._require(self.call(ControlSession, "/system/acknowledge_generation", ack))
            return response

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
        max_linear_mm_s2=50.0,
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
        goal.limits.max_linear_m_s2 = finite_float(max_linear_mm_s2, "max_linear_mm_s2") / 1000.0
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

    def capabilities(self):
        return self.call(GetCapabilities, "/gca/get_capabilities", GetCapabilities.Request())

    def set_linear_reference(self, position_mm=0.0):
        previous_reference = self.wait_state(lambda _: True).reference_id
        response = self.manage(
            "set_reference", axis_mask=1, linear_mm=finite_float(position_mm, "position_mm")
        )
        # The management response may precede the next telemetry publication.
        self.wait_state(
            lambda state: (
                bool(state.reference_id)
                and state.reference_id != previous_reference
                and state.linear_reference_valid
                and state.linear_valid
                and state.linear_velocity_valid
            )
        )
        return response

    def make_linear_goal(
        self,
        position_mm,
        *,
        relative=False,
        speed_mm_s=2.0,
        acceleration_mm_s2=5.0,
        timeout_s=10.0,
        command_id=None,
    ):
        """Explicit single-axis command in user units, checked against current capabilities."""
        position_mm = finite_float(position_mm, "position_mm")
        speed_mm_s = finite_float(speed_mm_s, "speed_mm_s")
        acceleration_mm_s2 = finite_float(acceleration_mm_s2, "acceleration_mm_s2")
        timeout_s = finite_float(timeout_s, "timeout_s")
        if min(speed_mm_s, acceleration_mm_s2, timeout_s) <= 0:
            raise ValueError("speed, acceleration and timeout must be positive")
        caps = self.capabilities()
        if not caps.can_linear_motion or not caps.can_measure_linear:
            raise RuntimeError("Position control with external feedback is unavailable")
        if speed_mm_s > caps.limits.max_linear_m_s * 1000:
            raise ValueError("speed exceeds device capability")
        if acceleration_mm_s2 > caps.limits.max_linear_m_s2 * 1000:
            raise ValueError("acceleration exceeds device capability")
        return self.make_goal(
            linear_mm=position_mm,
            rotation_deg=0.0,
            axis_mask=1,
            relative=relative,
            hold=False,
            timeout_s=timeout_s,
            max_linear_mm_s=speed_mm_s,
            max_linear_mm_s2=acceleration_mm_s2,
            command_id=command_id,
        )

    def move_linear(self, position_mm, *, cancel_after=None, **kwargs):
        if cancel_after is not None:
            cancel_after = finite_float(cancel_after, "cancel_after")
            if cancel_after < 0:
                raise ValueError("cancel_after must be nonnegative")
        goal = self.make_linear_goal(position_mm, **kwargs)
        handle = self.send_goal(goal)
        if not handle.accepted:
            raise RuntimeError("Linear motion rejected")
        if cancel_after is not None:
            time.sleep(min(cancel_after, goal.execution_timeout_s))
            self.wait(handle.cancel_goal_async())
        return self.wait(handle.get_result_async(), goal.execution_timeout_s + 5.0).result

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
        if not 0 < duration_s <= 3600.0:
            raise ValueError("duration_s must be within (0, 3600] seconds")
        start_ns = time.monotonic_ns()
        episode_end_ns = start_ns + int(duration_s * 1e9)
        epoch = uuid.uuid4().hex
        sample_seq = 0
        until = episode_end_ns / 1e9
        try:
            while time.monotonic() < until:
                context = self.context()
                sample_seq += 1
                observed = time.monotonic_ns()
                quality = ManualInputQuality(
                    source_kind="PROGRAMMED_JOG",
                    source_id="gca_cli",
                    input_epoch=epoch,
                    event_seq=1,
                    event_ns=start_ns,
                    event_time_source="PROGRAMMED_EPISODE",
                    observation_seq=sample_seq,
                    observation_ns=observed,
                    valid_until_ns=min(observed + 150_000_000, episode_end_ns),
                    clock_domain_id=Path("/proc/sys/kernel/random/boot_id")
                    .read_text(encoding="ascii")
                    .strip(),
                    link_evidence="PROGRAMMED",
                    episode_end_ns=episode_end_ns,
                )
                permit = self.motion_permit(context, 0.15)
                if time.monotonic_ns() >= quality.valid_until_ns:
                    continue
                msg = JogCommand(
                    context=context,
                    token=permit.token,
                    axis_mask=3,
                    deadman=bool(deadman),
                    linear_velocity_m_s=linear_mm_s / 1000.0,
                    rotation_velocity_rad_s=math.radians(rotation_deg_s),
                    ttl_s=0.15,
                    quality=quality,
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
