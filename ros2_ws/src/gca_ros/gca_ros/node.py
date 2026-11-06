"""Lifecycle device host. ROS owns ingress/results; local runtime owns motion.

Action callbacks and the terminal guard share one mutually-exclusive group.
The execute coroutine is an executor task and deliberately does nothing but
await its registered future. This prevents the cancel-to-CANCELING transition
from racing a terminal canceled() call.
"""

import copy
import functools
import json
import math
import os
import signal
import threading
import time
import uuid
from collections import OrderedDict, deque
from dataclasses import dataclass, fields, replace
from queue import SimpleQueue

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gca.config import RuntimeConfig
from gca.domain.manual import QualityTracker
from gca.domain.models import Lease, MotionEvent, OperationResult, RuntimeState
from gca.runtime import ControlRuntime
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import GcaState, JogCommand
from gca_system_interfaces.srv import (
    ApplyConfig,
    ControlSession,
    ExecutionLease,
    GetCapabilities,
    GetCommandStatus,
    InjectFault,
    ManageDevice,
)
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .conversion import (
    context_from_ros,
    jog_from_ros,
    motion_from_ros,
    quality_from_ros,
    state_from_core,
)
from .permits import MotionPermits, capability_part, same_motion_payload
from .security import AccessError, LeaseAuthority


@dataclass
class _Goal:
    command: object
    handle: object = None
    future: object = None
    canceled: bool = False
    committed: bool = False
    accepted_at: float = 0.0
    request: object = None


class GcaControlNode(LifecycleNode):
    """The only process allowed to hold the two physical actuator resources."""

    def __init__(self):
        # Jazzy LifecycleNode constructs its services in the node's default
        # group; its **kwargs are passed to Node, which has no callback_group.
        super().__init__("gca_control_node")
        self.management_group = self.default_callback_group
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.stop_group = MutuallyExclusiveCallbackGroup()
        self.fault_group = MutuallyExclusiveCallbackGroup()
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self._lock = threading.RLock()
        self.runtime = None
        self._active = False
        self._goals = OrderedDict()
        self._denials = OrderedDict()
        self._config_changes = SimpleQueue()
        self._config_pending = False
        self._config_application_error = ""
        self._recorder = None
        self._last_record_ns = 0
        self._terminal_paused = False
        self._pending_terminal_events = deque()
        self._telemetry_release = threading.Event()
        self._telemetry_release.set()
        self._arm_release = threading.Event()
        self._arm_release.set()
        self._arm_waiting = False
        self._config_names = {f.name for f in fields(RuntimeConfig) if f.name != "hardware"}
        defaults = RuntimeConfig().as_dict()
        for name, value in defaults.items():
            if name in self._config_names:
                self.declare_parameter(name, value)
        self.declare_parameter("config_file", "")
        self.declare_parameter("configuration_file", "")
        self.declare_parameter("coordinator_key", "")
        self.declare_parameter("trusted_local_control", False)
        self.declare_parameter("record_directory", "")
        self.declare_parameter("executor_threads", 3)
        self.declare_parameter("telemetry_hz", 10.0)
        self.executor_threads = self.get_parameter("executor_threads").value
        if type(self.executor_threads) is not int or self.executor_threads <= 0:
            raise ValueError("executor_threads must be a positive integer")
        self._config = self._read_config()
        self._requested_revision = self._config.revision
        self._authority = LeaseAuthority(
            self.get_parameter("coordinator_key").value, self._config.lease_max_s
        )
        self._motion_permits = MotionPermits(capacity=self._config.history_capacity)
        self._quality = QualityTracker(
            backend=self._config.backend,
            max_age_s=self._config.manual_input_max_age_s,
            capacity=self._config.history_capacity,
        )
        self._guard = self.create_guard_condition(self._drain, callback_group=self.action_group)
        self._config_guard = self.create_guard_condition(
            self._apply_pending_config, callback_group=self.management_group
        )
        self._action = ActionServer(
            self,
            ExecuteMotion,
            "/gca/execute_motion",
            self._execute,
            callback_group=self.action_group,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            handle_accepted_callback=self._accepted,
            result_timeout=30,
        )
        self.create_service(
            ExecutionLease,
            "/gca/execution_lease",
            self._lease,
            callback_group=self.management_group,
        )
        self.create_service(
            ControlSession,
            "/gca/get_motion_permit",
            self._motion_permit,
            callback_group=self.management_group,
        )
        for operation in ("arm", "disarm", "reset_fault", "request_stop", "set_reference"):
            self.create_service(
                ManageDevice,
                "/gca/" + operation,
                functools.partial(self._manage, operation),
                callback_group=self.stop_group
                if operation in ("request_stop", "disarm")
                else self.management_group,
            )
        for name in ("stop", "disarm"):
            self.create_service(
                ManageDevice,
                "/gca/coordinator_" + name,
                functools.partial(self._coordinator_stop, disarm=name == "disarm"),
                callback_group=self.stop_group,
            )
        self.create_service(
            GetCapabilities,
            "/gca/get_capabilities",
            self._capabilities,
            callback_group=self.management_group,
        )
        self.create_service(
            GetCommandStatus,
            "/gca/get_command_status",
            self._query,
            callback_group=self.management_group,
        )
        self.create_service(
            ApplyConfig,
            "/gca/apply_config",
            self._apply_config,
            callback_group=self.management_group,
        )
        self.create_service(
            InjectFault, "/gca/inject_fault", self._fault, callback_group=self.fault_group
        )
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(
            JogCommand, "/gca/jog", self._jog, qos, callback_group=self.action_group
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Ordinary publishers intentionally remain available while inactive/faulted.
        self._state_pub = self.create_publisher(GcaState, "/gca/state", state_qos)
        self._arm_phase_pub = self.create_publisher(String, "/gca/arm_phase", state_qos)
        self._diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        hz = float(self.get_parameter("telemetry_hz").value)
        if not math.isfinite(hz) or not 0.5 <= hz <= 100:
            raise ValueError("telemetry_hz must be between 0.5 and 100")
        self.create_timer(1.0 / hz, self._telemetry, callback_group=self.telemetry_group)
        self.create_timer(0.2, self._record, callback_group=self.telemetry_group)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._queue_parameters)

    def _read_config(self):
        values = {name: self.get_parameter(name).value for name in self._config_names}
        path = (
            self.get_parameter("config_file").value
            or self.get_parameter("configuration_file").value
        )
        if path:
            with open(path, encoding="utf-8") as stream:
                text = stream.read(65537)
            if len(text.encode("utf-8")) > 65536:
                raise ValueError("CONFIG_TOO_LARGE")
            document = yaml.safe_load(text) or {}
            loaded = document.get("runtime", document)
            if not isinstance(loaded, dict):
                raise ValueError("runtime must be a mapping")
            loaded = dict(loaded)
            if "devices" in document:
                loaded["hardware"] = document["devices"]
            if loaded.get("backend") == "hardware":
                return RuntimeConfig.from_dict(loaded)
            values.update(loaded)
        return RuntimeConfig.from_dict(values)

    def _snapshot(self):
        runtime = self.runtime
        return runtime.snapshot() if runtime is not None else RuntimeState()

    def _state_msg(self):
        return state_from_core(
            self._snapshot(), self.get_clock().now().to_msg(), self._requested_revision
        )

    def on_configure(self, state):
        try:
            if self.runtime is not None:
                return TransitionCallbackReturn.FAILURE
            self._config = self._read_config()
            self._quality = QualityTracker(
                backend=self._config.backend,
                max_age_s=self._config.manual_input_max_age_s,
                capacity=self._config.history_capacity,
            )
            self._requested_revision = self._config.revision
            devices = None
            if self._config.backend == "hardware":
                secured = (
                    os.environ.get("ROS_SECURITY_ENABLE", "").lower() == "true"
                    and os.environ.get("ROS_SECURITY_STRATEGY") == "Enforce"
                )
                trusted_local = (
                    os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                    and self.get_parameter("trusted_local_control").value is True
                )
                if not (secured or trusted_local):
                    raise ValueError(
                        "Hardware requires enforced DDS security or explicit trusted localhost deployment"
                    )
                from gca.devices.factory import hardware_devices

                devices = hardware_devices(self._config)
            self.runtime = ControlRuntime(self._config, notify=self._guard.trigger, devices=devices)
            result = self.runtime.start()
            directory = self.get_parameter("record_directory").value
            if result.accepted and directory:
                from gca.observability import AsyncRecorder

                self._recorder = AsyncRecorder(directory)
                self._last_record_ns = 0
            return (
                TransitionCallbackReturn.SUCCESS
                if result.accepted
                else TransitionCallbackReturn.FAILURE
            )
        except Exception as exc:
            self.get_logger().error(f"Configuration failed: {type(exc).__name__}: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        if self.runtime is None:
            return TransitionCallbackReturn.FAILURE
        result = self.runtime.activate(True)
        self._active = result.accepted
        return (
            TransitionCallbackReturn.SUCCESS
            if result.accepted
            else TransitionCallbackReturn.FAILURE
        )

    def on_deactivate(self, state):
        self._active = False
        if self.runtime is None:
            return TransitionCallbackReturn.SUCCESS
        self.runtime.stop("DEACTIVATE", disarm=True)
        result = self.runtime.activate(False)
        deadline = time.monotonic() + self._config.stop_timeout_s + 0.1
        pause = threading.Event()
        while result.accepted and time.monotonic() < deadline:
            snapshot = self.runtime.snapshot()
            if snapshot.stop_progress == "CONFIRMED" and not snapshot.armed:
                return TransitionCallbackReturn.SUCCESS
            if snapshot.stop_progress == "UNKNOWN":
                break
            pause.wait(0.01)
        return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state):
        return self._shutdown_runtime()

    def on_shutdown(self, state):
        return self._shutdown_runtime()

    def on_error(self, state):
        self._active = False
        if self.runtime is not None:
            self.runtime.stop("LIFECYCLE_ERROR", disarm=True)
        return TransitionCallbackReturn.SUCCESS

    def _shutdown_runtime(self):
        self._active = False
        if self.runtime is not None:
            self.runtime.stop("LIFECYCLE_SHUTDOWN", disarm=True)
        self._arm_release.set()
        self._terminal_paused = False
        self._telemetry_release.set()
        if self.runtime is None:
            return TransitionCallbackReturn.SUCCESS
        closed = self.runtime.close(timeout_s=self._config.stop_timeout_s + 1.0)
        self._record()
        if self._recorder is not None:
            self._recorder.close(timeout_s=1.0)
            self._recorder = None
        if not closed.accepted:
            return TransitionCallbackReturn.ERROR
        self.runtime = None
        self._revoke_authority()
        return TransitionCallbackReturn.SUCCESS

    def _revoke_authority(self):
        grant = self._authority.grant
        self._authority.revoke()
        if grant is not None:
            self._quality.clear_session(grant.session_id, grant.generation)
            self._motion_permits.retire_generation(grant.session_id, grant.generation)

    def _authorized(self, context, token, expired=False):
        return self._authority.verify(
            token, context.session_id, context.generation, allow_expired=expired
        )

    def _restricted_authorized(self, context, token):
        try:
            return self._authorized(context, token, expired=True)
        except AccessError:
            return self._authority.verify_retired(token, context.session_id, context.generation)

    def _motion_permit(self, request, response):
        try:
            grant = self._authorized(request.context, request.token)
            envelope, deadline = self._motion_permits.issue(
                grant.token, request.context, request.lease_s, grant.deadline
            )
            response.accepted = True
            response.error_code = "OK"
            response.context = request.context
            response.token = envelope
            response.granted_lease_s = max(0.0, deadline - time.monotonic())
        except (AccessError, ValueError) as exc:
            response.error_code = str(exc)
        return response

    def _lease(self, request, response):
        try:
            if self.runtime is None:
                raise AccessError("NOT_CONFIGURED")
            context = request.context
            if request.release:
                self._restricted_authorized(context, request.token)
                if request.coordinator_key != self.get_parameter("coordinator_key").value:
                    raise AccessError("UNAUTHORIZED_COORDINATOR")
                snapshot = self._snapshot()
                if snapshot.armed or snapshot.stop_progress != "CONFIRMED":
                    self.runtime.stop("CONTROL_RELEASED", disarm=True)
                self._revoke_authority()
                response.accepted = True
                response.error_code = "OK"
                return response
            old = self._authority.grant
            if old is not None and time.monotonic() >= old.deadline:
                if self._snapshot().state in ("DISARMED", "FAULT", "READY"):
                    self._revoke_authority()
            if self._authority.grant is None and self._snapshot().armed:
                raise AccessError("EXECUTOR_STOPPING_OR_FAULTED")
            grant = self._authority.issue(
                request.coordinator_key,
                context.session_id,
                context.generation,
                context.sequence,
                request.lease_s,
                request.token,
            )
            if self._snapshot().armed:
                result = self.runtime.renew(
                    Lease(
                        grant.session_id,
                        grant.generation,
                        grant.sequence,
                        request.lease_s,
                        grant.token,
                    )
                )
                if not result.accepted:
                    self._revoke_authority()
                    raise AccessError(result.code)
            response.accepted = True
            response.error_code = "OK"
            response.token = grant.token
            response.granted_lease_s = max(0.0, grant.deadline - time.monotonic())
        except (AccessError, ValueError) as exc:
            response.error_code = str(exc)
            response.message = "Execution lease rejected"
        return response

    def _manage(self, operation, request, response):
        expected_serial = self.runtime.stop_serial if self.runtime is not None else 0
        try:
            if operation in ("request_stop", "disarm"):
                with self._authority.transaction_lock:
                    self._restricted_authorized(request.context, request.token)
                    if self.runtime is None:
                        raise AccessError("NOT_CONFIGURED")
                    result = self._stop_runtime(
                        operation.upper(), request.context, operation == "disarm"
                    )
                response.accepted, response.error_code, response.message = (
                    result.accepted,
                    result.code,
                    result.message,
                )
                response.state = self._state_msg()
                return response
            grant = self._authorized(request.context, request.token)
            if self.runtime is None:
                raise AccessError("NOT_CONFIGURED")
            if operation == "arm":
                if not self._arm_release.is_set():
                    self._arm_waiting = True
                    self._publish_arm_phase("WAITING", request.context)
                    try:
                        released = self._arm_release.wait(timeout=1.0)
                    finally:
                        self._arm_waiting = False
                    if not released:
                        self._publish_arm_phase("TIMEOUT", request.context)
                        self._motion_permits.fence(request.context)
                        self.runtime.stop(
                            "ARM_PAUSE_TIMEOUT",
                            disarm=True,
                            context=context_from_ros(request.context),
                        )
                        raise AccessError("ARM_PAUSE_TIMEOUT")
                    self._publish_arm_phase("RELEASED", request.context)
                self._motion_permits.check_fence(request.context)
                if not self._active or self._config_pending:
                    raise AccessError("NOT_ACTIVE_OR_CONFIG_PENDING")
                if self._requested_revision != self._snapshot().config_revision:
                    raise AccessError("CONFIG_REVISION_MISMATCH")
                result = self.runtime.arm(
                    Lease(
                        grant.session_id,
                        grant.generation,
                        grant.sequence,
                        max(0.0, grant.deadline - time.monotonic()),
                        grant.token,
                    ),
                    expected_stop_serial=expected_serial,
                )
            elif operation == "set_reference":
                result = self.runtime.set_reference(
                    request.axis_mask, request.linear_m * 1000.0, math.degrees(request.rotation_rad)
                )
            elif operation == "reset_fault":
                result = self.runtime.reset()
            else:
                self._motion_permits.fence(request.context)
                result = self.runtime.stop(
                    operation.upper(),
                    disarm=operation == "disarm",
                    context=context_from_ros(request.context),
                )
                self._arm_release.set()
            response.accepted = result.accepted
            response.error_code = result.code
            response.message = result.message
            response.operation_id = str(uuid.uuid4())
        except (AccessError, ValueError) as exc:
            response.error_code = str(exc)
            response.message = "Management request rejected"
        response.state = self._state_msg()
        return response

    def _publish_arm_phase(self, phase, context):
        self._arm_phase_pub.publish(
            String(
                data=json.dumps(
                    {
                        "phase": phase,
                        "boot_id": self._snapshot().boot_id,
                        "command_id": context.command_id,
                        "stop_serial": getattr(self.runtime, "stop_serial", 0),
                    }
                )
            )
        )

    def _stop_runtime(self, reason, context, disarm=False):
        self._motion_permits.fence(context)
        # The adapter has authenticated the source. During handover the core
        # still has its old Lease while the ROS authority knows the new one.
        # Always latch the serial first: a future-gen contextual stop alone
        # would fail core lease matching and miss a queued ARM.
        runtime = self.runtime
        if runtime is None:
            return OperationResult(False, "NOT_CONFIGURED")
        result = runtime.stop(reason, disarm=disarm)
        current = runtime.snapshot()
        if current.session_id == context.session_id and current.generation:
            bound = copy.deepcopy(context)
            bound.generation = current.generation
            self._motion_permits.fence(bound)
            runtime.stop(reason, disarm=disarm, context=context_from_ros(bound))
        self._arm_release.set()
        return result

    def _coordinator_stop(self, request, response, disarm=False):
        try:
            with self._authority.transaction_lock:
                known = self._authority.coordinator_stop(request.token, request.context.session_id)
                if self.runtime is None:
                    raise AccessError("NOT_CONFIGURED")
                # Fence both receiver-authorized and source handover generations.
                for generation in {known.generation, request.context.generation}:
                    context = copy.deepcopy(request.context)
                    context.generation = generation
                    self._motion_permits.fence(context)
                result = self._stop_runtime("COORDINATOR_STOP", request.context, disarm)
            response.accepted, response.error_code = result.accepted, result.code
            response.message = result.message
        except (AccessError, ValueError) as exc:
            response.error_code = str(exc)
        response.state = self._state_msg()
        return response

    def _deny(self, command_id, code):
        self._denials[command_id] = code
        while len(self._denials) > self._config.history_capacity:
            self._denials.popitem(last=False)
        return GoalResponse.REJECT

    def _goal(self, request):
        command_id = request.context.command_id
        try:
            grant = self._authorized(request.context, capability_part(request.token))
            if not self._active or self.runtime is None or self._config_pending:
                raise AccessError("NOT_ACTIVE")
            command = motion_from_ros(request)
            with self._lock:
                if command_id in self._goals:
                    previous = self._goals[command_id].request
                    return self._deny(
                        command_id,
                        "DUPLICATE_COMMAND"
                        if same_motion_payload(previous, request)
                        else "ID_CONFLICT",
                    )
                deadline = self._motion_permits.consume(request.token, request.context)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AccessError("MOTION_PERMIT_EXPIRED")
                command = replace(
                    command,
                    freshness_token=grant.token,
                    admission_timeout_s=min(command.admission_timeout_s, remaining),
                    admission_deadline_ns=int(deadline * 1_000_000_000),
                )
                result = self.runtime.reserve(command)
                if not result.accepted:
                    return self._deny(command_id, result.code)
                self._goals[command_id] = _Goal(
                    command, accepted_at=time.monotonic(), request=copy.deepcopy(request)
                )
            return GoalResponse.ACCEPT
        except (AccessError, ValueError, OverflowError) as exc:
            return self._deny(command_id, str(exc))

    def _accepted(self, handle):
        command_id = handle.request.context.command_id
        with self._lock:
            record = self._goals[command_id]
            record.handle = handle
            record.future = self.executor.create_future()
            result = self.runtime.start_motion(record.command)
            if not result.accepted:
                # Queue the failure so terminal submission occurs only in the guard.
                self._local_failures.put(
                    MotionEvent(command_id, "FAILED", result.code, result.message)
                )
                self._guard.trigger()
            handle.execute()

    async def _execute(self, handle):
        return await self._goals[handle.request.context.command_id].future

    def _cancel(self, handle):
        record = self._goals.get(handle.request.context.command_id)
        if record is None or record.committed:
            return CancelResponse.REJECT
        record.canceled = True
        self.runtime.stop("ACTION_CANCEL", command_id=handle.request.context.command_id)
        return CancelResponse.ACCEPT

    @property
    def _local_failures(self):
        if not hasattr(self, "_failure_queue"):
            self._failure_queue = SimpleQueue()
        return self._failure_queue

    def _drain(self):
        events = self.runtime.drain_events() if self.runtime is not None else []
        while not self._local_failures.empty():
            events.append(self._local_failures.get())
        # Simulation test barrier: retain candidates without committing them.
        # The queue is bounded independently from telemetry; no control work waits.
        self._pending_terminal_events.extend(events)
        if len(self._pending_terminal_events) > self._config.queue_capacity:
            self._terminal_paused = False
            if self.runtime is not None:
                self.runtime.stop("TERMINAL_BARRIER_OVERFLOW", disarm=True)
        if self._terminal_paused:
            return
        events = list(self._pending_terminal_events)
        self._pending_terminal_events.clear()
        for event in events:
            record = self._goals.get(event.command_id)
            if record is None or record.committed or record.handle is None:
                continue
            if record.canceled and event.outcome == "SUCCEEDED":
                continue
            result = ExecuteMotion.Result()
            result.outcome = {"SUCCEEDED": 1, "CANCELED": 2, "FAILED": 3}.get(event.outcome, 3)
            result.error_code = event.code
            result.message = event.message
            result.final_state = self._state_msg()
            result.outputs_held = event.outputs_held
            result.stop_confirmed = event.stop_confirmed
            if event.outcome == "SUCCEEDED" and not record.canceled:
                record.handle.succeed()
            elif (
                record.canceled
                and event.outcome != "FAILED"
                and event.stop_confirmed
                and record.handle.is_cancel_requested
            ):
                result.outcome = 2
                record.handle.canceled()
            else:
                record.handle.abort()
            record.committed = True
            record.future.set_result(result)
            if self.runtime is not None:
                self.runtime.commit_terminal(event.command_id, event.outcome)
        # Retain bounded terminal records for idempotency; active records are never evicted.
        for command_id, record in list(self._goals.items()):
            if len(self._goals) <= self._config.history_capacity:
                break
            if record.committed:
                del self._goals[command_id]

    def _jog(self, request):
        try:
            if not request.deadman:
                base = capability_part(request.token) if "." in request.token else request.token
                with self._authority.transaction_lock:
                    self._restricted_authorized(request.context, base)
                    if self.runtime is not None:
                        self._stop_runtime("DEADMAN_RELEASED", request.context)
                return
            grant = self._authorized(request.context, capability_part(request.token))
            deadline = self._motion_permits.consume(request.token, request.context)
            input_deadline = self._quality.validate(
                quality_from_ros(request.quality),
                time.monotonic_ns(),
                session_id=request.context.session_id,
                generation=request.context.generation,
            )
            deadline = min(deadline, input_deadline / 1e9)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise AccessError("MOTION_PERMIT_EXPIRED")
            if self.runtime is not None and self._active:
                command = replace(
                    jog_from_ros(request),
                    freshness_token=grant.token,
                    ttl_s=min(request.ttl_s, remaining),
                    admission_deadline_ns=int(deadline * 1_000_000_000),
                )
                self.runtime.jog(command)
        except (AccessError, ValueError):
            # Rejected messages do not extend the previous jog deadline.
            return

    def _query(self, request, response):
        try:
            grant = self._authority.grant
            if grant is None:
                raise AccessError("NO_SESSION")
            self._authority.verify(
                request.token, request.session_id, grant.generation, allow_expired=True
            )
            result = self.runtime.get_command_status(request.command_id)
            response.found = result.accepted or request.command_id in self._denials
            response.status = result.code
            response.error_code = self._denials.get(request.command_id, result.code)
            response.message = result.message
        except (AccessError, AttributeError) as exc:
            response.error_code = str(exc)
        response.state = self._state_msg()
        return response

    def _capabilities(self, request, response):
        response.backend = self._config.backend
        response.simulation = self._config.backend == "simulation"
        verified = response.simulation or (
            self._config.hardware_verified and self._snapshot().configured
        )
        response.can_linear_motion = verified
        response.can_rotation_motion = verified
        response.can_stop = verified
        response.can_measure_linear = verified
        response.can_measure_rotation = False
        response.limits.max_linear_m_s = self._config.max_linear_mm_s / 1000.0
        response.limits.max_linear_m_s2 = self._config.max_linear_mm_s2 / 1000.0
        response.limits.max_rotation_rad_s = math.radians(self._config.max_rotation_deg_s)
        response.limits.max_rotation_rad_s2 = math.radians(self._config.max_rotation_deg_s2)
        if not response.simulation:
            response.unverified_capabilities = ["H01_IDENTITY", "H02_PULSES", "H03_STOP"]
        return response

    def set_parameters_atomically(self, parameters):
        # Keep validation + post-set enqueue atomic with motion reservations.
        # The validators themselves remain free from device/controller side effects.
        with self._lock:
            return super().set_parameters_atomically(parameters)

    def _validate_parameters(self, parameters):
        changed = {p.name: p.value for p in parameters if p.name in self._config_names}
        protected = {
            "coordinator_key",
            "trusted_local_control",
            "record_directory",
            "config_file",
            "configuration_file",
            "executor_threads",
            "telemetry_hz",
        }
        if any(p.name in protected for p in parameters):
            return SetParametersResult(successful=False, reason="Restart required")
        if not changed:
            return SetParametersResult(successful=True)
        with self._lock:
            if (
                self._config_pending
                or self._snapshot().state != "DISARMED"
                or any(not rec.committed for rec in self._goals.values())
            ):
                return SetParametersResult(
                    successful=False, reason="DISARMED and no reservations required"
                )
            try:
                values = self._config.as_dict()
                hot = {"revision", "kp", "ki", "kd", "kaw", "integral_limit_mm_s"}
                if any(
                    name not in hot and value != values[name] for name, value in changed.items()
                ):
                    raise ValueError(
                        "RECONFIGURE_REQUIRED: only controller gains/revision may be updated"
                    )
                values.update(changed)
                candidate = RuntimeConfig.from_dict(values)
                if candidate.revision == self._config.revision and values != self._config.as_dict():
                    raise ValueError("new revision required")
            except (ValueError, TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _queue_parameters(self, parameters):
        changed = {p.name: p.value for p in parameters if p.name in self._config_names}
        if changed:
            values = self._config.as_dict()
            values.update(changed)
            candidate = RuntimeConfig.from_dict(values)
            with self._lock:
                self._config_pending = True
                self._requested_revision = candidate.revision
                self._config_changes.put(candidate)
            self._config_guard.trigger()

    def _apply_pending_config(self):
        while not self._config_changes.empty():
            candidate = self._config_changes.get()
            result = (
                self.runtime.configure(candidate)
                if self.runtime is not None
                else OperationResult(False, "NOT_CONFIGURED")
            )
            if result.accepted:
                self._config = candidate
                self._config_application_error = ""
            else:
                self._config_application_error = result.code + ": " + result.message
            self._config_pending = False

    def _apply_config(self, request, response):
        try:
            self._authorized(request.context, request.token)
            if len(request.configuration_yaml.encode("utf-8")) > 65536:
                raise ValueError("CONFIG_TOO_LARGE")
            values = yaml.safe_load(request.configuration_yaml)
            if isinstance(values, dict) and "runtime" in values:
                document = values
                values = dict(document["runtime"])
                if "devices" in document:
                    values["hardware"] = document["devices"]
            candidate = RuntimeConfig.from_dict(values)
            if candidate.hardware != self._config.hardware:
                raise ValueError(
                    "RECONFIGURE_REQUIRED: device wiring cannot change through parameters"
                )
            result = self.set_parameters_atomically(
                [
                    Parameter(name, value=value)
                    for name, value in candidate.as_dict().items()
                    if name in self._config_names
                ]
            )
            response.accepted = result.successful
            response.error_code = "ACCEPTED" if result.successful else "INVALID_CONFIG"
            response.message = result.reason
        except (AccessError, ValueError, TypeError, yaml.YAMLError) as exc:
            response.error_code = str(exc)
        response.requested_revision = self._requested_revision
        response.applied_revision = self._snapshot().config_revision
        return response

    def _fault(self, request, response):
        try:
            if self._config.backend != "simulation":
                raise AccessError("SIMULATION_ONLY")
            if request.enabled:
                self._authorized(request.context, request.token)
            else:
                self._restricted_authorized(request.context, request.token)
            if request.name == "arm_pause":
                if request.enabled:
                    self._arm_release.clear()
                else:
                    self._arm_release.set()
                result = OperationResult(True)
            elif request.name == "terminal_pause":
                self._terminal_paused = request.enabled
                if not request.enabled:
                    self._guard.trigger()
                result = OperationResult(True)
            elif request.name == "telemetry_pause":
                if request.enabled:
                    self._telemetry_release.clear()
                else:
                    self._telemetry_release.set()
                result = OperationResult(True)
            else:
                result = self.runtime.inject_fault(request.name, request.enabled)
            response.accepted, response.error_code, response.message = (
                result.accepted,
                result.code,
                result.message,
            )
        except (AccessError, ValueError) as exc:
            response.error_code = str(exc)
        return response

    def _telemetry(self):
        # Simulation-only service controls this bounded publication stall. Its
        # management/action groups and all local I/O owners remain runnable.
        if not self._telemetry_release.wait(timeout=1.0):
            return
        state = self._state_msg()
        self._state_pub.publish(state)
        with self._lock:
            records = list(self._goals.values())
        for record in records:
            if record.handle is not None and not record.committed:
                feedback = ExecuteMotion.Feedback()
                feedback.phase = state.state
                feedback.state = state
                feedback.elapsed_s = time.monotonic() - record.accepted_at
                feedback.stop = state.stop
                record.handle.publish_feedback(feedback)
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name, status.hardware_id = "gca_control", state.device_epoch
        status.level = DiagnosticStatus.ERROR if state.fault_code else DiagnosticStatus.OK
        status.message = state.fault_code or state.state
        status.values = [
            KeyValue(key="sample_age_s", value=str(state.feedback_age_s)),
            KeyValue(key="stop_progress", value=state.stop.progress),
            KeyValue(key="arm_waiting", value=str(self._arm_waiting)),
            KeyValue(key="config_last_error", value=self._config_application_error),
        ]
        if self._config.backend == "simulation" and self.runtime is not None:
            status.values.append(
                KeyValue(
                    key="simulation_io_evidence",
                    value=json.dumps(self.runtime.simulation_evidence(), allow_nan=False),
                )
            )
        recorder = self._recorder
        if recorder is not None:
            log_status = recorder.status()
            status.values.extend(
                [
                    KeyValue(key="log_dropped", value=str(log_status.dropped)),
                    KeyValue(key="log_errors", value=str(log_status.errors)),
                    KeyValue(key="log_last_error", value=log_status.last_error),
                ]
            )
        array.status = [status]
        self._diagnostics.publish(array)
        # Bound the orphan reservation created if a goal response cannot be sent.
        with self._lock:
            for cid, record in list(self._goals.items()):
                if (
                    record.handle is None
                    and time.monotonic() > record.accepted_at + record.command.admission_timeout_s
                ):
                    self.runtime.release_reservation(cid)
                    del self._goals[cid]

    def _record(self):
        runtime, recorder = self.runtime, self._recorder
        if runtime is None or recorder is None:
            return
        for record in runtime.records():
            if record["time_ns"] > self._last_record_ns:
                recorder.submit(record)
                self._last_record_ns = record["time_ns"]


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GcaControlNode()
    executor = MultiThreadedExecutor(num_threads=node.executor_threads)
    executor.add_node(node)
    stopping = threading.Event()

    def request_shutdown(signum, frame):
        stopping.set()
        executor.wake()

    previous = {
        sig: signal.signal(sig, request_shutdown) for sig in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node._shutdown_runtime()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
