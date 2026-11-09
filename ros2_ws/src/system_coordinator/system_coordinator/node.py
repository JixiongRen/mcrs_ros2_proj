"""Single source authority and GCA-only Action proxy with bounded async RPCs."""

import copy
import functools
import math
import signal
import threading
import time
import uuid
from collections import OrderedDict, deque
from dataclasses import dataclass

import rclpy
from gca.domain.manual import QualityError, QualityTracker
from gca_ros.conversion import quality_from_ros
from gca_ros.permits import MotionPermits, PermitError, capability_part, same_motion_payload
from gca_system_interfaces.action import ExecuteMotion
from gca_system_interfaces.msg import CommandContext, GcaState, JogCommand, ServoJogCommand
from gca_system_interfaces.srv import (
    ApplyConfig,
    ControlSession,
    ExecutionLease,
    GetCommandStatus,
    InjectFault,
    ManageDevice,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from .authority import SessionError, SourceAuthority, bounded_positive_seconds


@dataclass
class _Task:
    request: object
    source: object
    handle: object = None
    future: object = None
    downstream: object = None
    canceled: bool = False
    committed: bool = False
    feedback: object = None
    started: float = 0.0
    stopping_after_result: bool = False
    permit_deadline: float = 0.0


class SystemCoordinator(Node):
    """Forward requests only while their source and device capabilities are valid."""

    _profile = "gca_only"

    def __init__(self):
        super().__init__("system_coordinator")
        self.declare_parameter("coordinator_key", "")
        self.declare_parameter("source_lease_max_s", 5.0)
        self.declare_parameter("executor_lease_s", 2.0)
        self.declare_parameter("rpc_timeout_s", 2.0)
        self.declare_parameter("handover_timeout_s", 2.0)
        self.declare_parameter("profile", "gca_only")
        self.declare_parameter("servo_config_file", "")
        self.declare_parameter("control_mode", "", ParameterDescriptor(read_only=True))
        self._profile = self.get_parameter("profile").value
        if self._profile not in ("gca_only", "servo"):
            raise ValueError("Only explicit gca_only and servo profiles are enabled")
        self._lock = threading.RLock()
        self._closing = False
        self._rpc_waits = {}
        self._rpc_requests = {}
        self._background_futures = set()
        self._background_clients = {}
        self._background_errors = deque(maxlen=16)
        self._authority = SourceAuthority(
            float(self.get_parameter("source_lease_max_s").value),
            handover_s=self.get_parameter("handover_timeout_s").value,
        )
        self._executor_lease_s = float(self.get_parameter("executor_lease_s").value)
        self._rpc_timeout = bounded_positive_seconds(
            self.get_parameter("rpc_timeout_s").value, "rpc_timeout_s"
        )
        self._key = self.get_parameter("coordinator_key").value
        if not self._key:
            raise ValueError("coordinator_key must be explicitly provisioned by launch/deployment")
        if not math.isfinite(self._executor_lease_s) or not 0.1 <= self._executor_lease_s <= 5.0:
            raise ValueError("executor_lease_s must be finite between 0.1 and 5")
        self.action_group = MutuallyExclusiveCallbackGroup()
        self.management_group = MutuallyExclusiveCallbackGroup()
        # Management awaits downstream RPCs; do not queue permits behind them.
        self.permit_group = MutuallyExclusiveCallbackGroup()
        self.stop_group = MutuallyExclusiveCallbackGroup()
        self.fault_group = MutuallyExclusiveCallbackGroup()
        self.clients_group = ReentrantCallbackGroup()
        self.telemetry_group = MutuallyExclusiveCallbackGroup()
        self._tasks = OrderedDict()
        self._results = deque(maxlen=64)
        self._denials = OrderedDict()
        self._highwater = {}
        self._motion_permits = MotionPermits()
        servo_config_file = self.get_parameter("servo_config_file").value
        requested_mode = self.get_parameter("control_mode").value
        self._control_mode = ""
        self._control_policy = "LEGACY_DUAL_AXIS"
        self._held_servo_input = False
        if servo_config_file:
            if self._profile != "servo":
                raise ValueError("servo_config_file requires the servo profile")
            from gca.servo_config import load_servo_settings

            settings = load_servo_settings(servo_config_file)
            if requested_mode and requested_mode != settings.control_mode:
                raise ValueError("control_mode must match the validated servo snapshot")
            self._control_mode = settings.control_mode
            self._control_policy = settings.control_policy
            self._held_servo_input = settings.manual_hold_enabled
            self._quality = QualityTracker(
                backend=settings.backend,
                max_age_s=settings.manual_input_max_age_s,
                capacity=settings.history_capacity,
                allow_joydev_hold=settings.manual_hold_enabled,
            )
        else:
            if self._profile == "servo":
                # Previous manual entry points did not pass a snapshot to the coordinator.
                if requested_mode not in ("", "manual"):
                    raise ValueError("position control requires a validated servo snapshot")
                self._control_mode = "manual"
                self._control_policy = "MANUAL_SERVO_VELOCITY"
            elif requested_mode:
                raise ValueError("control_mode requires the servo profile")
            self._quality = QualityTracker()
        self._stop_epoch = 0
        self._pending_jog = None
        self._jog_forwarding = False
        self._jog_epoch = 0
        self._jog_stopping = None
        self._state = GcaState()
        self._state.state = "INITIALIZING"
        self._state.control_policy = self._control_policy
        self._retired_boots = deque(maxlen=64)
        self._guard = self.create_guard_condition(self._drain, callback_group=self.action_group)
        self._device_action = ActionClient(
            self, ExecuteMotion, "/gca/execute_motion", callback_group=self.clients_group
        )
        self._lease_client = self.create_client(
            ExecutionLease, "/gca/execution_lease", callback_group=self.clients_group
        )
        self._permit_client = self.create_client(
            ControlSession, "/gca/get_motion_permit", callback_group=self.clients_group
        )
        self.create_service(
            ControlSession,
            "/system/get_motion_permit",
            self._motion_permit,
            callback_group=self.permit_group,
        )
        self._manage_clients = {
            name: self.create_client(
                ManageDevice, "/gca/" + name, callback_group=self.clients_group
            )
            for name in ("arm", "disarm", "reset_fault", "request_stop", "set_reference")
        }
        self._priority_stop_clients = {
            name: self.create_client(
                ManageDevice, "/gca/coordinator_" + name, callback_group=self.clients_group
            )
            for name in ("stop", "disarm")
        }
        self.create_service(
            ControlSession,
            "/system/acknowledge_generation",
            self._acknowledge,
            callback_group=self.stop_group,
        )
        self._config_client = self.create_client(
            ApplyConfig, "/gca/apply_config", callback_group=self.clients_group
        )
        self._fault_client = self.create_client(
            InjectFault, "/gca/inject_fault", callback_group=self.clients_group
        )
        self._query_client = self.create_client(
            GetCommandStatus, "/gca/get_command_status", callback_group=self.clients_group
        )
        for operation in ("claim_control", "renew_control", "release_control"):
            self.create_service(
                ControlSession,
                "/system/" + operation,
                functools.partial(self._control, operation),
                callback_group=self.management_group,
            )
        for operation in self._manage_clients:
            self.create_service(
                ManageDevice,
                "/system/" + operation,
                functools.partial(self._manage, operation),
                callback_group=self.stop_group
                if operation in ("request_stop", "disarm")
                else self.management_group,
            )
        self.create_service(
            ApplyConfig, "/system/apply_config", self._config, callback_group=self.management_group
        )
        self.create_service(
            InjectFault, "/system/inject_fault", self._fault, callback_group=self.fault_group
        )
        self.create_service(
            GetCommandStatus,
            "/system/get_command_status",
            self._query,
            callback_group=self.management_group,
        )
        self._action = ActionServer(
            self,
            ExecuteMotion,
            "/system/execute_motion",
            self._execute,
            callback_group=self.action_group,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            handle_accepted_callback=self._accepted,
            result_timeout=30,
        )
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
        )
        jog_type = ServoJogCommand if self._profile == "servo" else JogCommand
        jog_topic = "/gca/servo_jog" if self._profile == "servo" else "/gca/jog"
        manual_topic = (
            "/system/servo_manual_target" if self._profile == "servo" else "/system/manual_target"
        )
        self._jog_pub = self.create_publisher(jog_type, jog_topic, qos)
        self.create_subscription(
            jog_type, manual_topic, self._jog, qos, callback_group=self.action_group
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(GcaState, "/system/state", state_qos)
        self.create_subscription(
            GcaState,
            "/gca/state",
            self._receive_state,
            state_qos,
            callback_group=self.telemetry_group,
        )
        self._watchdog_timer = self.create_timer(
            0.05, self._watchdog, callback_group=self.telemetry_group
        )

    def _bounded(self, original, timeout=None):
        """Complete an executor future exactly once, even if a late RPC arrives."""
        wrapped = self.executor.create_future()
        lock = threading.Lock()
        deadline = time.monotonic() + (self._rpc_timeout if timeout is None else timeout)
        timer = None

        def complete(source=None):
            nonlocal timer
            with lock:
                if wrapped.done():
                    if source is not None:
                        # Late responses belong to a timed-out/canceled RPC;
                        # retrieve their result before releasing its resources.
                        try:
                            source.result()
                        except Exception as exc:
                            self._background_errors.append(type(exc).__name__)
                    return
                if source is not None:
                    try:
                        wrapped.set_result(source.result())
                    except Exception as exc:
                        wrapped.set_exception(exc)
                elif time.monotonic() >= deadline:
                    wrapped.set_exception(TimeoutError("RPC_TIMEOUT"))
                else:
                    return
                if timer is not None:
                    timer.cancel()
                    self.destroy_timer(timer)
                with self._lock:
                    self._rpc_waits.pop(wrapped, None)

        timer = self.create_timer(
            min(0.05, max(0.01, deadline - time.monotonic())),
            complete,
            callback_group=self.clients_group,
        )
        with self._lock:
            self._rpc_waits[wrapped] = (original, timer)
        original.add_done_callback(complete)
        return wrapped

    async def _call(self, client, request):
        if self._closing:
            raise SessionError("SHUTTING_DOWN")
        if not client.service_is_ready():
            raise SessionError("DEVICE_UNAVAILABLE")
        original = client.call_async(request)
        with self._lock:
            self._rpc_requests[original] = client
        try:
            return await self._bounded(original)
        except TimeoutError:
            client.remove_pending_request(original)
            raise
        finally:
            with self._lock:
                self._rpc_requests.pop(original, None)

    def _track_background(self, future, client=None):
        with self._lock:
            self._background_futures.add(future)
            if client is not None:
                self._background_clients[future] = client

        def completed(source):
            try:
                source.result()
            except Exception as exc:
                self._background_errors.append(type(exc).__name__)
            with self._lock:
                self._background_futures.discard(source)
                self._background_clients.pop(source, None)

        future.add_done_callback(completed)
        return future

    def _verify(self, context, token, expired=False):
        with self._lock:
            return self._authority.verify(context.session_id, context.generation, token, expired)

    def _restricted_verify(self, context, token):
        with self._lock:
            try:
                return self._authority.verify(
                    context.session_id, context.generation, token, allow_expired=True
                )
            except SessionError:
                return self._authority.verify_retired(context.session_id, context.generation, token)

    def _stop_verify(self, context, token):
        with self._lock:
            return self._authority.verify_stop(context.session_id, context.generation, token)

    def _acknowledge(self, request, response):
        try:
            with self._lock:
                source = self._authority.verify(
                    request.context.session_id, request.context.generation, request.token
                )
                previous = source.stop_alias_generation
                source = self._authority.acknowledge(
                    request.context.session_id, request.context.generation, request.token
                )
                if previous:
                    self._motion_permits.retire_generation(source.session_id, previous)
                    self._quality.clear_session(source.session_id, previous)
            response.accepted, response.error_code = True, "OK"
            response.context = self._domain_context(source)
            response.granted_lease_s = max(0.0, source.deadline - time.monotonic())
        except SessionError as exc:
            response.error_code = str(exc)
        return response

    def _fence_stop(self, context, source):
        # No awaits and no regular-management lock is held across remote calls.
        with self._lock:
            # Authentication may have completed before another callback released
            # that owner or acknowledged its generation. Revalidate while holding
            # the same lock that commits the shared STOP epochs and mailbox.
            source = self._authority.verify_stop(
                context.session_id, context.generation, source.token
            )
            self._stop_epoch += 1
            self._jog_epoch += 1
            self._pending_jog = None
            for generation in {context.generation, source.generation}:
                bound = copy.deepcopy(context)
                bound.generation = generation
                self._motion_permits.fence(bound)

    def _check_arm(self, source, epoch):
        with self._lock:
            if (
                self._authority.current is not source
                or self._stop_epoch != epoch
                or time.monotonic() >= source.deadline
                or self._closing
            ):
                raise SessionError("ARM_INTERRUPTED_BY_STOP")

    def _priority_stop_request(self, source, context):
        forwarded = ManageDevice.Request(context=copy.deepcopy(context), token=self._key)
        forwarded.context.generation = source.generation
        return forwarded

    def _domain_context(self, source, command_id=""):
        return CommandContext(
            session_id=source.session_id,
            generation=source.generation,
            sequence=source.renewal_sequence,
            command_id=command_id or str(uuid.uuid4()),
            task_id=str(uuid.uuid4()),
        )

    def _lease_request(self, source, duration, release=False):
        self._validate_source_mode(source.mode)
        request = ExecutionLease.Request()
        request.coordinator_key = self._key
        request.context = self._domain_context(source)
        request.token = source.executor_token
        request.lease_s = duration
        request.release = release
        request.control_policy = self._control_policy
        return request

    def _validate_source_mode(self, mode):
        if self._profile == "servo":
            expected = "AUTO" if self._control_mode == "position" else "MANUAL"
            if mode != expected:
                raise SessionError("CONTROL_MODE_MISMATCH")

    def _motion_permit(self, request, response):
        try:
            with self._lock:
                source = self._verify(request.context, request.token)
                envelope, deadline = self._motion_permits.issue(
                    source.token, request.context, request.lease_s, source.deadline
                )
            response.accepted = True
            response.error_code = "OK"
            response.context = request.context
            response.token = envelope
            response.granted_lease_s = max(0.0, deadline - time.monotonic())
        except (SessionError, ValueError) as exc:
            response.error_code = str(exc)
        return response

    async def _device_motion_permit(self, source, context, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise SessionError("MOTION_PERMIT_EXPIRED")
        request = ControlSession.Request(
            context=context, token=source.executor_token, lease_s=float(min(remaining, 1.0))
        )
        granted = await self._call(self._permit_client, request)
        if not granted.accepted:
            raise SessionError(granted.error_code)
        if time.monotonic() >= deadline:
            raise SessionError("MOTION_PERMIT_EXPIRED")
        return granted

    async def _control(self, operation, request, response):
        source = None
        try:
            if operation == "claim_control":
                with self._lock:
                    self._validate_source_mode(request.mode or "AUTO")
                    if any(not task.committed for task in self._tasks.values()):
                        raise SessionError("CONTROL_BUSY")
                    source = self._authority.claim(
                        request.owner, request.mode or "AUTO", request.lease_s
                    )
                duration = min(self._executor_lease_s, source.deadline - time.monotonic())
                grant = await self._call(self._lease_client, self._lease_request(source, duration))
                if not grant.accepted:
                    raise SessionError(grant.error_code)
                source.executor_token = grant.token
                with self._lock:
                    if self._authority.current is not source or time.monotonic() >= source.deadline:
                        self._track_background(
                            self._lease_client.call_async(
                                self._lease_request(source, 0.0, release=True)
                            )
                        )
                        raise SessionError("SESSION_EXPIRED_DURING_CLAIM")
            elif operation == "renew_control":
                source = self._verify(request.context, request.token)
                if request.context.sequence <= source.renewal_sequence:
                    raise SessionError("STALE_SEQUENCE")
                duration = self._authority.duration(request.lease_s)
                # Source deadline is not moved unless device renewal succeeds.
                source.renewal_sequence = request.context.sequence
                grant = await self._call(
                    self._lease_client,
                    self._lease_request(source, min(duration, self._executor_lease_s)),
                )
                if not grant.accepted:
                    raise SessionError(grant.error_code)
                with self._lock:
                    if self._authority.current is not source or time.monotonic() >= source.deadline:
                        source.executor_token = grant.token
                        self._track_background(
                            self._lease_client.call_async(
                                self._lease_request(source, 0.0, release=True)
                            )
                        )
                        raise SessionError("SESSION_EXPIRED_DURING_RENEWAL")
                source.deadline = time.monotonic() + duration
            else:
                source = self._restricted_verify(request.context, request.token)
                self._fence_stop(request.context, source)
                grant = await self._call(
                    self._lease_client, self._lease_request(source, 0.0, release=True)
                )
                if not grant.accepted:
                    raise SessionError(grant.error_code)
                with self._lock:
                    self._authority.release()
                    self._highwater.pop(source.session_id, None)
                    self._quality.clear_session(source.session_id)
                    self._motion_permits.retire_session(source.session_id)
            response.accepted = True
            response.error_code = "OK"
            response.context = self._domain_context(source)
            response.token = source.token if operation != "release_control" else ""
            response.granted_lease_s = max(0.0, source.deadline - time.monotonic())
        except (SessionError, ValueError, TimeoutError) as exc:
            response.error_code = str(exc)
            response.message = "Control operation rejected"
            if operation == "claim_control" and source is not None:
                with self._lock:
                    if self._authority.current is source:
                        self._authority.release(remember=False)
        return response

    async def _manage(self, operation, request, response):
        source = None
        with self._lock:
            arm_epoch = self._stop_epoch
        try:
            stopping = operation in ("request_stop", "disarm")
            source = (
                self._stop_verify(request.context, request.token)
                if stopping
                else self._verify(request.context, request.token)
            )
            if stopping:
                self._fence_stop(request.context, source)
                disarm = operation == "disarm" or bool(source.handover_deadline)
                if disarm:
                    source.needs_reauthorization = True
                device = await self._call(
                    self._priority_stop_clients["disarm" if disarm else "stop"],
                    self._priority_stop_request(source, request.context),
                )
            else:
                if operation == "arm":
                    if source.handover_deadline:
                        raise SessionError("HANDOVER_ACK_REQUIRED")
                    self._motion_permits.check_fence(request.context)
                    self._check_arm(source, arm_epoch)
                if operation == "arm" and source.needs_reauthorization:
                    if self._state.state != "DISARMED" or self._state.stop.progress != "CONFIRMED":
                        raise SessionError("WAIT_FOR_DISARMED")
                    released = await self._call(
                        self._lease_client, self._lease_request(source, 0.0, release=True)
                    )
                    self._check_arm(source, arm_epoch)
                    if not released.accepted:
                        raise SessionError(released.error_code)
                    with self._lock:
                        self._authority.rotate_generation(source)
                    duration = min(self._executor_lease_s, source.deadline - time.monotonic())
                    granted = await self._call(
                        self._lease_client, self._lease_request(source, duration)
                    )
                    self._check_arm(source, arm_epoch)
                    if not granted.accepted:
                        raise SessionError(granted.error_code)
                    source.executor_token = granted.token
                forwarded = copy.deepcopy(request)
                forwarded.context.generation = source.generation
                forwarded.token = source.executor_token
                if operation == "arm":
                    self._check_arm(source, arm_epoch)
                device = await self._call(self._manage_clients[operation], forwarded)
                if operation == "arm":
                    self._check_arm(source, arm_epoch)
            response.accepted, response.error_code, response.message = (
                device.accepted,
                device.error_code,
                device.message,
            )
            response.operation_id, response.state = device.operation_id, device.state
            response.state.generation = source.generation
            if device.accepted and operation in ("disarm", "reset_fault"):
                source.needs_reauthorization = True
            elif operation == "arm":
                source.needs_reauthorization = not device.accepted
        except (SessionError, ValueError, TimeoutError) as exc:
            response.error_code = str(exc)
            response.state = copy.deepcopy(self._state)
            if source is not None:
                response.state.generation = source.generation
                if operation == "arm":
                    source.needs_reauthorization = True
        return response

    async def _config(self, request, response):
        try:
            source = self._verify(request.context, request.token)
            forwarded = copy.deepcopy(request)
            forwarded.token = source.executor_token
            device = await self._call(self._config_client, forwarded)
            for name in (
                "accepted",
                "error_code",
                "message",
                "requested_revision",
                "applied_revision",
            ):
                setattr(response, name, getattr(device, name))
        except (SessionError, TimeoutError) as exc:
            response.error_code = str(exc)
        return response

    async def _fault(self, request, response):
        try:
            source = (
                self._verify(request.context, request.token)
                if request.enabled
                else self._restricted_verify(request.context, request.token)
            )
            forwarded = copy.deepcopy(request)
            forwarded.token = source.executor_token
            device = await self._call(self._fault_client, forwarded)
            response.accepted, response.error_code, response.message = (
                device.accepted,
                device.error_code,
                device.message,
            )
        except (SessionError, TimeoutError) as exc:
            response.error_code = str(exc)
        return response

    async def _query(self, request, response):
        try:
            with self._lock:
                source = self._authority.current
                if source is None:
                    raise SessionError("NO_SESSION")
                self._authority.verify(
                    request.session_id, source.generation, request.token, allow_expired=True
                )
            if request.command_id in self._denials:
                response.found = True
                response.status = "REJECTED"
                response.error_code = self._denials[request.command_id]
                response.state = self._state
                return response
            forwarded = copy.deepcopy(request)
            forwarded.token = source.executor_token
            device = await self._call(self._query_client, forwarded)
            for name in ("found", "status", "error_code", "message", "outcome", "state"):
                setattr(response, name, getattr(device, name))
        except (SessionError, TimeoutError) as exc:
            response.error_code = str(exc)
        return response

    def _deny(self, command_id, error):
        self._denials[command_id] = error
        while len(self._denials) > 256:
            self._denials.popitem(last=False)
        return GoalResponse.REJECT

    def _goal(self, request):
        try:
            source = self._verify(request.context, capability_part(request.token))
            if self._profile == "servo":
                if source.mode != "AUTO":
                    raise SessionError("AUTO_MODE_REQUIRED")
                self._validate_source_mode(source.mode)
                if (
                    request.target.axis_mask != 1
                    or request.target.rotation_rad != 0.0
                    or request.end_policy != 1
                ):
                    raise SessionError("POSITION_LINEAR_STOP_REQUIRED")
            if not self._state.lifecycle_active or not self._state.armed:
                raise SessionError("NOT_ARMED")
            if self._state.state != "READY":
                raise SessionError("CONTROL_BUSY")
            if request.target.mode not in (1, 2):
                raise SessionError("INVALID_MODE")
            if request.target.reference_id != self._state.reference_id:
                raise SessionError("REFERENCE_MISMATCH")
            numeric = (
                request.target.linear_m,
                request.target.rotation_rad,
                request.limits.max_linear_m_s,
                request.limits.max_linear_m_s2,
                request.limits.max_rotation_rad_s,
                request.limits.max_rotation_rad_s2,
                request.admission_timeout_s,
                request.execution_timeout_s,
            )
            if not all(math.isfinite(value) for value in numeric):
                raise SessionError("NONFINITE_ARGUMENT")
            if (
                not 0 < request.admission_timeout_s <= 5.0
                or not 0 < request.execution_timeout_s <= 600.0
            ):
                raise SessionError("INVALID_TIMEOUT")
            with self._lock:
                if request.context.command_id in self._tasks:
                    previous = self._tasks[request.context.command_id].request
                    raise SessionError(
                        "DUPLICATE_COMMAND"
                        if same_motion_payload(previous, request)
                        else "ID_CONFLICT"
                    )
                if any(not task.committed for task in self._tasks.values()):
                    raise SessionError("CONTROL_BUSY")
                if request.context.sequence <= self._highwater.get(source.session_id, 0):
                    raise SessionError("STALE_SEQUENCE")
                if not self._device_action.server_is_ready():
                    raise SessionError("DEVICE_UNAVAILABLE")
                if request.end_policy not in (1, 2) or request.target.axis_mask not in (1, 2, 3):
                    raise SessionError("INVALID_ARGUMENT")
                self._tasks[request.context.command_id] = _Task(
                    copy.deepcopy(request),
                    source,
                    started=time.monotonic(),
                    permit_deadline=self._motion_permits.consume(request.token, request.context),
                )
                self._highwater[source.session_id] = request.context.sequence
            return GoalResponse.ACCEPT
        except (SessionError, ValueError) as exc:
            return self._deny(request.context.command_id, str(exc))

    def _accepted(self, handle):
        record = self._tasks[handle.request.context.command_id]
        record.handle = handle
        record.future = self.executor.create_future()
        handle.execute()
        self.executor.create_task(self._forward, record)

    async def _execute(self, handle):
        return await self._tasks[handle.request.context.command_id].future

    def _cancel(self, handle):
        record = self._tasks.get(handle.request.context.command_id)
        if record is None or record.committed:
            return CancelResponse.REJECT
        record.canceled = True
        if record.downstream is not None:
            self._track_background(record.downstream.cancel_goal_async())
        return CancelResponse.ACCEPT

    async def _forward(self, record):
        try:
            self._verify(record.request.context, capability_part(record.request.token))
            if time.monotonic() >= record.permit_deadline:
                raise SessionError("MOTION_PERMIT_EXPIRED")
            if record.canceled:
                # No device command was submitted; obtain explicit stop state first.
                stop = ManageDevice.Request()
                stop.context = record.request.context
                stop.token = record.source.executor_token
                response = await self._call(self._manage_clients["request_stop"], stop)
                result = ExecuteMotion.Result(
                    outcome=1,
                    error_code="CANCELED_BEFORE_START",
                    final_state=response.state,
                    stop_confirmed=response.state.stop.progress == "CONFIRMED",
                )
            else:
                forwarded = copy.deepcopy(record.request)
                permit = await self._device_motion_permit(
                    record.source, forwarded.context, record.permit_deadline
                )
                forwarded.token = permit.token
                forwarded.admission_timeout_s = float(
                    min(
                        forwarded.admission_timeout_s,
                        permit.granted_lease_s,
                        record.permit_deadline - time.monotonic(),
                    )
                )

                def feedback(message):
                    record.feedback = message.feedback

                record.downstream = await self._bounded(
                    self._device_action.send_goal_async(forwarded, feedback_callback=feedback)
                )
                if not record.downstream.accepted:
                    raise SessionError("DEVICE_GOAL_REJECTED")
                if record.canceled:
                    self._track_background(record.downstream.cancel_goal_async())
                wrapped = await self._bounded(
                    record.downstream.get_result_async(),
                    timeout=record.request.execution_timeout_s + 3.0,
                )
                result = wrapped.result
            self._enqueue(record, result)
        except (SessionError, TimeoutError, ValueError) as exc:
            if record.downstream is not None and record.downstream.accepted:
                self._track_background(record.downstream.cancel_goal_async())
            result = ExecuteMotion.Result(
                outcome=3,
                error_code=str(exc),
                message="Device task failed",
                final_state=self._state,
                stop_confirmed=False,
            )
            self._enqueue(record, result)

    def _enqueue(self, record, result):
        with self._lock:
            if len(self._results) == self._results.maxlen:
                # Capacity exhaustion revokes authority; never silently overwrite a terminal.
                self._expire_source("RESULT_QUEUE_FULL")
                return
            self._results.append((record, result))
        self._guard.trigger()

    def _drain(self):
        with self._lock:
            results = list(self._results)
            self._results.clear()
        for record, result in results:
            if record.committed:
                continue
            if record.canceled:
                if result.outcome == 1 and (result.outputs_held or not result.stop_confirmed):
                    if not record.stopping_after_result:
                        record.stopping_after_result = True
                        self.executor.create_task(self._stop_after_candidate, record, result)
                    continue
                if result.stop_confirmed and result.outcome != 3:
                    result.outcome = 2
                    result.outputs_held = False
                    if record.handle.is_cancel_requested:
                        record.handle.canceled()
                    else:
                        result.outcome = 3
                        result.error_code = "CANCEL_STATE_MISMATCH"
                        record.handle.abort()
                else:
                    result.outcome = 3
                    record.handle.abort()
            elif result.outcome == 1:
                record.handle.succeed()
            else:
                record.handle.abort()
            record.committed = True
            # Action result is authoritative feedback too. Advance the cache
            # before exposing its terminal future so a subsequent command is
            # not refused by an older DDS EXECUTING snapshot.
            self._update_state(result.final_state, terminal=True)
            record.future.set_result(result)
        for cid, record in list(self._tasks.items()):
            if len(self._tasks) <= 256:
                break
            if record.committed:
                del self._tasks[cid]

    async def _stop_after_candidate(self, record, result):
        """A success candidate cannot bypass a cancellation accepted during HOLD handoff."""
        try:
            request = ManageDevice.Request(
                context=record.request.context, token=record.source.executor_token
            )
            response = await self._call(self._manage_clients["request_stop"], request)
            if not response.accepted:
                raise SessionError(response.error_code)
            deadline = time.monotonic() + self._rpc_timeout
            settled = self.executor.create_future()
            timer = None

            def check():
                nonlocal timer
                state = self._state
                if state.stop.progress == "CONFIRMED" and state.state != "HOLDING":
                    settled.set_result(state)
                elif state.stop.progress == "UNKNOWN" or time.monotonic() >= deadline:
                    settled.set_exception(TimeoutError("CANCEL_STOP_UNCONFIRMED"))
                else:
                    return
                timer.cancel()
                self.destroy_timer(timer)

            timer = self.create_timer(0.02, check, callback_group=self.clients_group)
            result.final_state = await settled
            result.stop_confirmed = True
            result.outputs_held = False
            result.outcome = 2
        except (SessionError, TimeoutError) as exc:
            result.outcome = 3
            result.stop_confirmed = False
            result.error_code = str(exc)
        self._enqueue(record, result)

    def _jog(self, request):
        if self._profile == "servo" and self._control_mode != "manual":
            return
        stage = "authorization"
        try:
            if not request.deadman:
                stage = "deadman_authorization"
                base = capability_part(request.token) if "." in request.token else request.token
                source = self._stop_verify(request.context, base)
                with self._lock:
                    stage = "deadman_fence"
                    self._fence_stop(request.context, source)
                    identity = (source.session_id, source.generation)
                    if self._jog_stopping is None or self._jog_stopping[:2] != identity:
                        # An old owner's/generation's RPC cannot suppress the
                        # current STOP or clear its worker on exit. Capture the
                        # generation now; SourceSession is mutable during ARM.
                        work = (*identity, object())
                        self._jog_stopping = work
                        self.executor.create_task(
                            self._stop_jog, copy.deepcopy(request), source, work
                        )
                return
            source = self._verify(request.context, capability_part(request.token))
            stage = "input_policy"
            if source.mode != "MANUAL":
                raise SessionError("MANUAL_MODE_REQUIRED")
            if request.quality.source_kind == "JOYDEV_HELD_ARM":
                raise SessionError("ARM_PROOF_CANNOT_JOG")
            if self._profile == "servo" and self._held_servo_input:
                if request.quality.source_kind != "JOYDEV_HELD_STATE":
                    raise SessionError("JOYDEV_HELD_STATE_REQUIRED")
            stage = "motion_permit"
            deadline = self._motion_permits.consume(request.token, request.context)
            stage = "input_quality"
            input_deadline = self._quality.validate(
                quality_from_ros(request.quality),
                time.monotonic_ns(),
                session_id=request.context.session_id,
                generation=request.context.generation,
            )
            deadline = min(deadline, input_deadline / 1e9)
            with self._lock:
                stage = "mailbox_authorization"
                source = self._verify(request.context, capability_part(request.token))
                stage = "mailbox_fence"
                self._motion_permits.check_fence(request.context)
                self._pending_jog = (copy.deepcopy(request), source, deadline, self._jog_epoch)
                if not self._jog_forwarding:
                    self._jog_forwarding = True
                    self.executor.create_task(self._forward_jog)
        except (SessionError, ValueError) as exc:
            code = (
                str(exc)
                if isinstance(exc, (SessionError, PermitError, QualityError))
                else "INVALID_ARGUMENT"
            )
            self.get_logger().warning(
                f"JOG_REJECTED stage={stage} sequence={request.context.sequence} "
                f"monotonic_ns={time.monotonic_ns()} code={code}"
            )
            return

    async def _forward_jog(self):
        """One in-flight permit RPC plus one latest mailbox; never a velocity FIFO."""
        try:
            while True:
                with self._lock:
                    pending, self._pending_jog = self._pending_jog, None
                    if pending is None:
                        return
                request, source, deadline, epoch = pending
                stage = "device_permit"
                try:
                    permit = await self._device_motion_permit(source, request.context, deadline)
                    stage = "forward_authorization"
                    self._verify(request.context, capability_part(request.token))
                    stage = "forward_fence"
                    self._motion_permits.check_fence(request.context)
                    with self._lock:
                        if epoch != self._jog_epoch:
                            self.get_logger().warning(
                                "JOG_REJECTED stage=forward_epoch "
                                f"sequence={request.context.sequence} "
                                f"monotonic_ns={time.monotonic_ns()} code=JOG_EPOCH_CHANGED"
                            )
                            continue
                    stage = "forward_deadline"
                    request.token = permit.token
                    request.ttl_s = float(
                        min(request.ttl_s, permit.granted_lease_s, deadline - time.monotonic())
                    )
                    if request.ttl_s > 0:
                        stage = "publish"
                        self._jog_pub.publish(request)
                    else:
                        self.get_logger().warning(
                            f"JOG_REJECTED stage={stage} sequence={request.context.sequence} "
                            f"monotonic_ns={time.monotonic_ns()} code=NO_REMAINING_JOG_TTL"
                        )
                except (SessionError, TimeoutError, ValueError) as exc:
                    if isinstance(exc, (SessionError, PermitError)):
                        code = str(exc)
                    elif isinstance(exc, TimeoutError):
                        code = "RPC_TIMEOUT"
                    else:
                        code = "INVALID_ARGUMENT"
                    self.get_logger().warning(
                        f"JOG_REJECTED stage={stage} sequence={request.context.sequence} "
                        f"monotonic_ns={time.monotonic_ns()} code={code}"
                    )
                    continue
        finally:
            self._finish_jog_worker()

    def _finish_jog_worker(self):
        # Release worker ownership exactly once. An ingress callback can enqueue
        # after the empty check, so atomically hand off to one successor.
        with self._lock:
            self._jog_forwarding = False
            if self._pending_jog is not None and not self._closing:
                self._jog_forwarding = True
                self.executor.create_task(self._forward_jog)

    async def _stop_jog(self, request, source, work):
        try:
            stop = self._priority_stop_request(source, request.context)
            await self._call(self._priority_stop_clients["stop"], stop)
        except (SessionError, TimeoutError) as exc:
            # Failure cannot renew an old velocity: both Jog and executor leases
            # remain finite at the device, even when the stop RPC is unavailable.
            self._background_errors.append("JOG_STOP_RPC: " + str(exc))
        finally:
            with self._lock:
                if self._jog_stopping is work:
                    self._jog_stopping = None

    def _receive_state(self, state):
        if self._closing:
            return
        if self._update_state(state):
            self._state_pub.publish(self._state)

    def _update_state(self, state, terminal=False):
        with self._lock:
            current = self._state
            if state.boot_id == current.boot_id and state.state_seq < current.state_seq:
                return False
            if (
                state.boot_id == current.boot_id
                and state.state_seq == current.state_seq
                and current.boot_id
                and not terminal
            ):
                # A frozen control writer does not advance state_seq, while
                # the independent observer continues aging its last sample.
                # Merge quality only, preserving committed terminal/domain data.
                identity = (
                    "state",
                    "active_command_id",
                    "output_generation",
                    "generation",
                    "session_id",
                    "sample_seq",
                )
                newer_stop = self._profile == "servo" and state.stop_serial > current.stop_serial
                if any(getattr(state, name) != getattr(current, name) for name in identity):
                    return False
                if state.feedback_age_s <= current.feedback_age_s and not newer_stop:
                    return False
                updated = copy.deepcopy(current)
                updated.feedback_age_s = state.feedback_age_s
                updated.linear_valid = state.linear_valid
                updated.linear_velocity_valid = state.linear_velocity_valid
                if self._profile == "servo":
                    updated.servo = copy.deepcopy(state.servo)
                    if newer_stop:
                        updated.stop_serial = state.stop_serial
                        updated.stop = copy.deepcopy(state.stop)
                    else:
                        # A frozen control owner cannot refresh stop evidence.
                        # Preserve committed progress, but revoke aged observations.
                        updated.stop.servo_confirmed &= state.stop.servo_confirmed
                        updated.stop.linear_confirmed &= state.stop.linear_confirmed
                updated.stamp = state.stamp
                self._state = updated
                return True
            if state.boot_id != current.boot_id:
                if state.boot_id in self._retired_boots:
                    return False
                if current.boot_id:
                    self._retired_boots.append(current.boot_id)
            self._state = state
            return True

    def _expire_source(self, reason):
        source = self._authority.current
        if source is None:
            return
        context = self._domain_context(source)
        context.sequence = (
            max(
                source.renewal_sequence,
                self._motion_permits.highwater(source.session_id, source.generation),
            )
            + 1
        )
        self._fence_stop(context, source)
        self._track_background(
            self._priority_stop_clients["disarm"].call_async(
                self._priority_stop_request(source, context)
            ),
            self._priority_stop_clients["disarm"],
        )
        if self._lease_client.service_is_ready():
            self._track_background(
                self._lease_client.call_async(self._lease_request(source, 0.0, release=True))
            )
        self._authority.release()
        self._highwater.pop(source.session_id, None)
        self._quality.clear_session(source.session_id)
        self._motion_permits.retire_session(source.session_id)

    def _watchdog(self):
        if self._closing:
            return
        with self._lock:
            source = self._authority.current
            if source is not None and (
                time.monotonic() >= source.deadline
                or (source.handover_deadline and time.monotonic() >= source.handover_deadline)
            ):
                self._expire_source("SOURCE_EXPIRED")
        for record in list(self._tasks.values()):
            if record.handle is None and not record.committed:
                if time.monotonic() > record.started + max(0.1, record.request.admission_timeout_s):
                    record.committed = True
            if record.handle is not None and not record.committed and record.feedback is not None:
                record.handle.publish_feedback(record.feedback)
                record.feedback = None

    def close(self):
        with self._lock:
            self._closing = True
            self._watchdog_timer.cancel()
            self._expire_source("COORDINATOR_SHUTDOWN")
        for record in list(self._tasks.values()):
            if not record.committed and record.handle is not None:
                if record.downstream is not None and record.downstream.accepted:
                    self._track_background(record.downstream.cancel_goal_async())
                self._enqueue(
                    record,
                    ExecuteMotion.Result(
                        outcome=3,
                        error_code="COORDINATOR_SHUTDOWN",
                        final_state=self._state,
                        stop_confirmed=False,
                    ),
                )

    def finish_shutdown(self):
        """Resolve owned waiters while the executor and service handles still exist."""
        with self._lock:
            waits = list(self._rpc_waits.items())
            self._rpc_waits.clear()
            requests = list(self._rpc_requests.items())
            self._rpc_requests.clear()
            background = [(f, self._background_clients.get(f)) for f in self._background_futures]
        for future, client in requests:
            client.remove_pending_request(future)
        for wrapped, (original, timer) in waits:
            timer.cancel()
            self.destroy_timer(timer)
            if not wrapped.done():
                wrapped.set_exception(SessionError("SHUTTING_DOWN"))
            if not original.done():
                original.cancel()
        for future, client in background:
            if not future.done():
                if client is not None:
                    client.remove_pending_request(future)
                future.cancel()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SystemCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    stopping = threading.Event()

    def stop(signum, frame):
        stopping.set()
        executor.wake()

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.close()
        # Give the bounded release transaction a chance to reach the executor.
        until = time.monotonic() + 0.2
        while time.monotonic() < until and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
        node.finish_shutdown()
        until = time.monotonic() + 0.2
        while time.monotonic() < until and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
