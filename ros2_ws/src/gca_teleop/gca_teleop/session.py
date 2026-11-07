"""Single MANUAL owner with asynchronous motion and independent STOP clients.

Blocking administrative calls run only on the local IPC worker. The context
lock is never held across an RPC; STOP remains available during ARM/renewal.
"""

from __future__ import annotations

import copy
import math
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass

from gca.domain.manual import ManualInputQuality
from gca_ros.conversion import quality_to_ros
from gca_system_interfaces.msg import CommandContext, GcaState
from gca_system_interfaces.srv import ControlSession, GetCapabilities, InjectFault, ManageDevice
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class _Credentials:
    session_id: str
    generation: int
    token: str
    sequence: int


class ManualSession:
    def __init__(
        self,
        node,
        *,
        notify=lambda event: None,
        timeout_s=3.0,
        control_profile="dual_axis",
        servo_settings=None,
    ):
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        self.node, self.notify, self.timeout_s = node, notify, float(timeout_s)
        if control_profile not in ("dual_axis", "servo"):
            raise ValueError("UNKNOWN_CONTROL_PROFILE")
        if (control_profile == "servo") != (servo_settings is not None):
            raise ValueError("SERVO_SETTINGS_PROFILE_MISMATCH")
        self.control_profile, self.servo_settings = control_profile, servo_settings
        self._servo_capable = False
        self._lock = threading.RLock()
        self._management_lock = threading.Lock()
        self._closing = threading.Event()
        self._credentials = None
        self._state = None
        self._state_ns = 0
        self._retired_boots = deque(maxlen=64)
        self._owned_futures = {}
        self._stop_required_epoch = 0
        self._stop_confirmed_epoch = 0
        self._stop_receipt = None
        self._stop_epoch = 0
        self._stop_future = None
        self._stop_started_ns = 0
        self._stop_deadline_ns = 0
        self._stop_timed_out_epoch = 0
        self._stop_latest = None
        self._renew_future = None
        self._renew_started_ns = 0
        self._renew_after_ns = 0
        self._management_active = False
        self._release_in_progress = False
        self._backend = ""
        self._backend_boot = ""
        self._capabilities_future = None
        self._capabilities_started_ns = 0
        self.error = ""
        self._normal_group = ReentrantCallbackGroup()
        self._stop_group = ReentrantCallbackGroup()
        self._state_group = ReentrantCallbackGroup()
        self._clients = {}
        for name in (
            "claim_control",
            "renew_control",
            "release_control",
            "get_motion_permit",
            "acknowledge_generation",
        ):
            self._clients[name] = node.create_client(
                ControlSession, "/system/" + name, callback_group=self._normal_group
            )
        for name in ("set_reference", "arm", "disarm", "reset_fault"):
            self._clients[name] = node.create_client(
                ManageDevice, "/system/" + name, callback_group=self._normal_group
            )
        self._clients["request_stop"] = node.create_client(
            ManageDevice, "/system/request_stop", callback_group=self._stop_group
        )
        self._clients["get_state"] = node.create_client(
            GetState, "/gca_control_node/get_state", callback_group=self._normal_group
        )
        self._clients["change_state"] = node.create_client(
            ChangeState, "/gca_control_node/change_state", callback_group=self._normal_group
        )
        self._clients["capabilities"] = node.create_client(
            GetCapabilities, "/gca/get_capabilities", callback_group=self._normal_group
        )
        self._clients["fault"] = node.create_client(
            InjectFault, "/system/inject_fault", callback_group=self._stop_group
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = node.create_subscription(
            GcaState, "/system/state", self._receive_state, qos, callback_group=self._state_group
        )
        self._renew_timer = node.create_timer(
            0.05, self._renew_tick, callback_group=self._normal_group
        )

    def _receive_state(self, state):
        with self._lock:
            self._merge_state_locked(state)

    def _merge_state_locked(self, state, *, authoritative=False):
        if not state.boot_id:
            return False
        current = self._state
        if current is not None and state.boot_id != current.boot_id:
            if state.boot_id in self._retired_boots:
                return False
            if current.boot_id:
                self._retired_boots.append(current.boot_id)
            self._backend = self._backend_boot = ""
            self._servo_capable = False
        elif current is not None:
            if state.state_seq < current.state_seq:
                return False
            if state.stop_serial < current.stop_serial:
                return False
            if state.state_seq == current.state_seq and not authoritative:
                # A synchronous ARM reply can expose the updated domain before
                # the current tick advances state_seq. Older DDS from that
                # same tick may age feedback, but must not undo that reply.
                identity = (
                    "state",
                    "armed",
                    "generation",
                    "session_id",
                    "reference_id",
                    "active_command_id",
                    "applied_config_revision",
                    "control_policy",
                )
                if any(getattr(state, key) != getattr(current, key) for key in identity):
                    if state.stop_serial <= current.stop_serial:
                        return False
                    merged = copy.deepcopy(current)
                    merged.stop_serial, merged.stop = state.stop_serial, copy.deepcopy(state.stop)
                    state = merged
        if current is None or state.boot_id != current.boot_id:
            self._backend = self._backend_boot = ""
            self._servo_capable = False
        self._state = copy.deepcopy(state)
        if (
            self.control_profile != "servo"
            or current is None
            or state.boot_id != current.boot_id
            or state.state_seq > current.state_seq
        ):
            # Servo telemetry freshness measures progress, not repeated delivery.
            self._state_ns = time.monotonic_ns()
        self._confirm_stop_locked()
        return True

    def _confirm_stop_locked(self):
        if self._expire_stop_confirmation_locked(time.monotonic_ns()):
            return
        receipt, state = self._stop_receipt, self._state
        if (
            receipt is not None
            and state is not None
            and receipt[0] >= self._stop_required_epoch
            and state.boot_id == receipt[1]
            and state.stop_serial >= receipt[2]
            and state.stop.progress == "CONFIRMED"
        ):
            self._stop_confirmed_epoch = max(self._stop_confirmed_epoch, receipt[0])

    def _expire_stop_confirmation_locked(self, now):
        if (
            self.control_profile != "servo"
            or self._stop_required_epoch <= self._stop_confirmed_epoch
            or not self._stop_deadline_ns
            or now < self._stop_deadline_ns
        ):
            return False
        if self._stop_timed_out_epoch != self._stop_required_epoch:
            self._stop_timed_out_epoch = self._stop_required_epoch
            self.error = "STOP_CONFIRMATION_TIMEOUT"
            self.notify({"kind": "stop_error", "error": self.error, "monotonic_ns": now})
        return True

    def _stop_settled_locked(self):
        return (
            self._stop_future is None
            and self._stop_latest is None
            and self._stop_confirmed_epoch >= self._stop_required_epoch
        )

    def _send(self, name, request):
        """Own every client request, including asynchronously canceled permits."""
        with self._lock:
            if self._closing.is_set():
                raise RuntimeError("SESSION_CLOSING")
            client = self._clients[name]
            future = client.call_async(request)
            self._owned_futures[future] = (client, time.monotonic_ns(), name)

        def finished(done):
            with self._lock:
                self._owned_futures.pop(done, None)
            # Client cancellation is local resource cleanup, never cancellation
            # of a remotely executing ARM; the STOP path handles its effects.
            client.remove_pending_request(done)
            if not done.cancelled():
                done.exception()  # Retrieve failures even if its consumer exited.

        future.add_done_callback(finished)
        return future

    def _abandon(self, future):
        with self._lock:
            owned = self._owned_futures.pop(future, None)
        if owned is not None:
            owned[0].remove_pending_request(future)
        if not future.done():
            future.cancel()

    @property
    def backend(self):
        with self._lock:
            return self._backend

    @property
    def state(self):
        with self._lock:
            return self._state

    @property
    def latest_stop_confirmed(self):
        """Only the latest requested STOP's acknowledged serial can confirm."""
        with self._lock:
            self._confirm_stop_locked()
            if self.control_profile == "servo" and (
                self._state is None
                or self._state.stop.progress != "CONFIRMED"
                or time.monotonic_ns() - self._state_ns >= 500_000_000
            ):
                return False
            return self._stop_settled_locked()

    @property
    def stop_epoch(self):
        with self._lock:
            return self._stop_epoch

    def identity(self):
        with self._lock:
            c = self._credentials
            return (c.session_id, c.generation) if c else ("", 0)

    def gate(self):
        with self._lock:
            self._expire_stop_confirmation_locked(time.monotonic_ns())
            state, c = self._state, self._credentials
            if state is None:
                return dict(
                    armed=False, ready=False, reference_id="", session_key="", hardware=False
                )
            if self.control_profile == "servo":
                axes_ready = (
                    self._servo_capable
                    and state.control_policy == "MANUAL_SERVO_VELOCITY"
                    and state.servo.configured
                    and state.servo.valid
                    and not state.servo.hardware_error
                    and not state.servo.error_code
                    and state.servo.sample_seq > 0
                    and bool(state.servo.device_epoch)
                    and state.applied_config_revision == self.servo_settings.revision
                )
            else:
                axes_ready = (
                    state.linear_reference_valid
                    and state.rotation_reference_valid
                    and state.rotation_estimate_valid
                )
            available = bool(
                c
                and self._backend
                and not self.error
                and not self._management_active
                and state.session_id == c.session_id
                and state.generation == c.generation
                and time.monotonic_ns() - self._state_ns < 500_000_000
                and state.lifecycle_active
                and axes_ready
                and state.requested_config_revision == state.applied_config_revision
            )
            ready = bool(
                available
                and self._stop_settled_locked()
                and state.stop.progress == "CONFIRMED"
                and state.state in ("READY", "EXECUTING")
            )
            result = dict(
                armed=state.armed,
                ready=ready,
                reference_id=state.reference_id,
                session_key=f"{c.session_id}:{c.generation}:{state.boot_id}:{state.applied_config_revision}"
                if c
                else "",
                hardware=self._backend == "hardware",
            )
            if self.control_profile == "servo":
                # Device-local validity already enforces feedback_max_age_s.
                # Only an otherwise healthy, owned STOP may retain a new LT edge.
                result.update(
                    stop_pending=bool(
                        available
                        and state.armed
                        and not self._stop_settled_locked()
                        and state.state in ("READY", "EXECUTING", "STOPPING")
                        and state.stop.progress in ("CONFIRMED", "REQUESTED", "IN_PROGRESS")
                    ),
                    stop_epoch=self._stop_required_epoch,
                )
                result["session_key"] += ":" + state.servo.device_epoch
            return result

    def context(self):
        with self._lock:
            c = self._credentials
            if c is None:
                raise RuntimeError("NO_MANUAL_SESSION")
            c.sequence += 1
            return CommandContext(
                session_id=c.session_id,
                generation=c.generation,
                sequence=c.sequence,
                task_id=str(uuid.uuid4()),
                command_id=str(uuid.uuid4()),
            ), c.token

    def _wait(self, future, timeout_s=None):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while not done.wait(min(0.05, max(0.0, deadline - time.monotonic()))):
            if self._closing.is_set():
                raise RuntimeError("SESSION_CLOSING")
            if time.monotonic() >= deadline:
                raise TimeoutError("MANAGEMENT_RPC_TIMEOUT")
        result = future.result()
        if result is None:
            raise RuntimeError("EMPTY_ROS_RESPONSE")
        return result

    def _call(self, name, request):
        client = self._clients[name]
        deadline = time.monotonic() + self.timeout_s
        while not client.service_is_ready():
            if self._closing.wait(0.02) or time.monotonic() >= deadline:
                raise TimeoutError("SERVICE_UNAVAILABLE:" + name)
        future = self._send(name, request)
        try:
            return self._wait(future, max(0.01, deadline - time.monotonic()))
        finally:
            if not future.done():
                self._abandon(future)

    @staticmethod
    def _require(response):
        if not response.accepted:
            raise RuntimeError(response.error_code + ":" + response.message)
        return response

    def permit_async(self, context, token, ttl_s):
        client = self._clients["get_motion_permit"]
        if not client.service_is_ready():
            raise RuntimeError("PERMIT_SERVICE_UNAVAILABLE")
        return self._send(
            "get_motion_permit",
            ControlSession.Request(context=context, token=token, lease_s=float(ttl_s)),
        )

    def request_stop(self, reason="OPERATOR_STOP"):
        with self._lock:
            if self.control_profile == "servo" and self._release_in_progress:
                # Releasing has already fenced all producer output. Its final
                # STOP remains pending/confirmed by its real receipt; a held B
                # or finished script must not keep moving that receipt's goal.
                return {
                    "accepted": True,
                    "stop_epoch": self._stop_required_epoch,
                    "coalesced_with_release": True,
                }
            self._stop_epoch += 1
            epoch = self._stop_epoch
            if self._credentials is None:
                self.notify(
                    {
                        "kind": "stop_local",
                        "reason": reason,
                        "stop_epoch": epoch,
                        "monotonic_ns": time.monotonic_ns(),
                    }
                )
                return {"accepted": True, "stop_epoch": epoch, "local_only": True}
            self._stop_required_epoch = epoch
            if self.control_profile == "servo":
                self._stop_deadline_ns = time.monotonic_ns() + int(self.timeout_s * 1e9)
            context, token = self.context()
            self._stop_latest = (ManageDevice.Request(context=context, token=token), reason, epoch)
            self._start_stop_locked()
            return {"accepted": True, "stop_epoch": epoch, "sequence": context.sequence}

    def _start_stop_locked(self):
        if self._stop_future is not None or self._stop_latest is None:
            return
        request, reason, epoch = self._stop_latest
        self._stop_latest = None
        client = self._clients["request_stop"]
        if not client.service_is_ready():
            self.error = "STOP_SERVICE_UNAVAILABLE"
            self.notify(
                {"kind": "stop_error", "error": self.error, "monotonic_ns": time.monotonic_ns()}
            )
            return
        try:
            future = self._send("request_stop", request)
        except Exception as exc:
            self.error = "STOP_RPC:" + type(exc).__name__
            return
        self._stop_future = future
        self._stop_started_ns = time.monotonic_ns()
        self.notify(
            {
                "kind": "stop_request",
                "reason": reason,
                "sequence": request.context.sequence,
                "generation": request.context.generation,
                "stop_epoch": epoch,
                "monotonic_ns": time.monotonic_ns(),
            }
        )

        def complete(f):
            with self._lock:
                if f is not self._stop_future:
                    return
                try:
                    response = self._require(f.result())
                    if (
                        self._credentials is not None
                        and request.context.session_id == self._credentials.session_id
                    ):
                        if not response.state.boot_id or response.state.stop_serial < 1:
                            raise RuntimeError("STOP_RECEIPT_MISSING")
                        self._stop_receipt = (
                            epoch,
                            response.state.boot_id,
                            response.state.stop_serial,
                        )
                        self._merge_state_locked(response.state, authoritative=True)
                        self._confirm_stop_locked()
                    self.notify(
                        {
                            "kind": "stop_accepted",
                            "sequence": request.context.sequence,
                            "stop_progress": response.state.stop.progress,
                            "stop_serial": response.state.stop_serial,
                            "boot_id": response.state.boot_id,
                            "state_seq": response.state.state_seq,
                            "stop_epoch": epoch,
                            "monotonic_ns": time.monotonic_ns(),
                        }
                    )
                except Exception as exc:
                    self.error = "STOP_FAILED:" + str(exc)[:160]
                    self.notify(
                        {
                            "kind": "stop_error",
                            "error": self.error,
                            "monotonic_ns": time.monotonic_ns(),
                        }
                    )
                finally:
                    self._stop_future = None
                    self._start_stop_locked()

        future.add_done_callback(complete)

    def _renew_tick(self):
        now = time.monotonic_ns()
        abandoned = []
        try:
            with self._lock:
                abandoned.extend(self._supervise_pending_locked(now))
                self._query_backend_locked()
                if self._closing.is_set() or self._credentials is None:
                    return
                if self._renew_future is not None:
                    if now - self._renew_started_ns > int(self.timeout_s * 1e9):
                        expired, self._renew_future = self._renew_future, None
                        abandoned.append(expired)
                        self.error = "RENEW_TIMEOUT"
                        self.request_stop(self.error)
                    return
                if self.error or self._management_active or now < self._renew_after_ns:
                    return
                context, token = self.context()
                client = self._clients["renew_control"]
                if not client.service_is_ready():
                    self.error = "RENEW_SERVICE_UNAVAILABLE"
                    self.request_stop(self.error)
                    return
                self._renew_started_ns = now
                self._renew_future = self._send(
                    "renew_control",
                    ControlSession.Request(context=context, token=token, lease_s=4.0),
                )
                future = self._renew_future
        finally:
            # An unbound rclpy Future invokes done callbacks synchronously on
            # cancel. Never let such callbacks inherit the session mutex and
            # reverse the dispatcher -> session lock order.
            for expired in abandoned:
                self._abandon(expired)

        def complete(f):
            with self._lock:
                if f is not self._renew_future:
                    return
                self._renew_future = None
                self._renew_after_ns = time.monotonic_ns() + 350_000_000
                if self.identity() != (context.session_id, context.generation):
                    return  # Never overwrite a newer ARM generation with late renewal.
                try:
                    self._require(f.result())
                except Exception as exc:
                    self.error = "RENEW_FAILED:" + str(exc)[:160]
                    self.request_stop(self.error)

        future.add_done_callback(complete)

    def _supervise_pending_locked(self, now):
        """Detach expired requests; the caller cancels them after releasing _lock."""
        abandoned = []
        budget = int(self.timeout_s * 1e9)
        confirmation_expired = self._expire_stop_confirmation_locked(now)
        if self._stop_future is not None and now - self._stop_started_ns >= budget:
            expired, self._stop_future = self._stop_future, None
            abandoned.append(expired)
            if not confirmation_expired:
                self.error = "STOP_TIMEOUT"
                self.notify({"kind": "stop_error", "error": self.error, "monotonic_ns": now})
            self._start_stop_locked()
        if self._capabilities_future is not None and now - self._capabilities_started_ns >= budget:
            expired, self._capabilities_future = self._capabilities_future, None
            abandoned.append(expired)
            self.notify(
                {"kind": "backend_error", "error": "CAPABILITIES_TIMEOUT", "monotonic_ns": now}
            )
        return abandoned

    def _query_backend_locked(self):
        if self._closing.is_set() or self._state is None or self._backend:
            return
        if self._capabilities_future is not None:
            return
        client = self._clients["capabilities"]
        if not client.service_is_ready():
            return
        boot = self._state.boot_id
        future = self._send("capabilities", GetCapabilities.Request())
        self._capabilities_future = future
        self._capabilities_started_ns = time.monotonic_ns()

        def complete(f):
            with self._lock:
                if f is not self._capabilities_future:
                    return
                self._capabilities_future = None
                try:
                    response = f.result()
                    if (
                        self._state is not None
                        and self._state.boot_id == boot
                        and response.backend in ("simulation", "hardware")
                    ):
                        if response.simulation != (response.backend == "simulation"):
                            raise RuntimeError("BACKEND_CAPABILITY_CONFLICT")
                        if response.control_profile != self.control_profile:
                            raise RuntimeError("CONTROL_PROFILE_MISMATCH")
                        if self.control_profile == "servo":
                            settings = self.servo_settings
                            if (
                                response.backend != settings.backend
                                or response.servo_config_sha256 != settings.sha256
                                or not response.can_servo_velocity
                                or not response.can_measure_servo
                                or not math.isclose(
                                    response.max_servo_velocity_rad_s,
                                    settings.max_rpm * 2.0 * math.pi / 60.0,
                                    rel_tol=1e-12,
                                    abs_tol=1e-12,
                                )
                            ):
                                raise RuntimeError("SERVO_CAPABILITY_CONFIG_MISMATCH")
                            self._servo_capable = True
                        self._backend = response.backend
                        self._backend_boot = boot
                except Exception:
                    self._backend = ""
                    self._servo_capable = False

        future.add_done_callback(complete)

    def manage(self, operation, *, linear_mm=0.0, rotation_deg=0.0, input_quality=None):
        if input_quality is not None and (
            operation != "arm"
            or self.control_profile != "servo"
            or not isinstance(input_quality, ManualInputQuality)
        ):
            raise ValueError("UNEXPECTED_ARM_INPUT_QUALITY")
        if not self._management_lock.acquire(blocking=False):
            raise RuntimeError("MANAGEMENT_BUSY")
        with self._lock:
            self._management_active = True
            epoch = self._stop_epoch
        try:
            if operation in ("configure", "activate"):
                current = self._call("get_state", GetState.Request()).current_state.id
                expected, target, transition = (
                    (1, (2, 3), 1) if operation == "configure" else (2, (3,), 3)
                )
                if current == expected:
                    request = ChangeState.Request()
                    request.transition.id = transition
                    if not self._call("change_state", request).success:
                        raise RuntimeError("LIFECYCLE_REJECTED")
                elif current not in target:
                    raise RuntimeError("LIFECYCLE_STATE_MISMATCH")
                return
            if operation == "claim":
                if self.identity()[0]:
                    raise RuntimeError("SESSION_ALREADY_OWNED")
                response = self._require(
                    self._call(
                        "claim_control",
                        ControlSession.Request(owner="gca_teleop", mode="MANUAL", lease_s=4.0),
                    )
                )
                with self._lock:
                    self._credentials = _Credentials(
                        response.context.session_id,
                        response.context.generation,
                        response.token,
                        response.context.sequence,
                    )
                    self.error = ""
                    self._stop_required_epoch = self._stop_confirmed_epoch = 0
                    self._stop_receipt = None
                    self._renew_after_ns = time.monotonic_ns() + 350_000_000
                return
            if operation == "release":
                with self._lock:
                    self.request_stop("OWNER_RELEASE")
                    self._release_in_progress = self.control_profile == "servo"
                if self.control_profile == "servo":
                    self._wait_stop_confirmed()
                    context, token = self.context()
                    response = self._require(
                        self._call(
                            "disarm",
                            ManageDevice.Request(context=context, token=token, axis_mask=0),
                        )
                    )
                    with self._lock:
                        self._merge_state_locked(response.state, authoritative=True)
                    self._wait_stop_confirmed(disarmed=True)
                context, token = self.context()
                try:
                    self._require(
                        self._call(
                            "release_control", ControlSession.Request(context=context, token=token)
                        )
                    )
                except Exception as exc:
                    self.error = "RELEASE_FAILED:" + str(exc)[:160]
                    raise
                finally:
                    # Explicit release also withdraws this process's authority
                    # when the peer retired/rotated it or its response was lost.
                    # Report uncertainty, stop renewing, and allow an explicit
                    # later claim rather than trapping a stale local session.
                    with self._lock:
                        self._credentials = None
                        self._stop_latest = None
                        pending = (self._stop_future, self._renew_future)
                        self._stop_future = self._renew_future = None
                    for future in pending:
                        if future is not None:
                            self._abandon(future)
                return
            service = {
                "reference": "set_reference",
                "arm": "arm",
                "disarm": "disarm",
                "reset": "reset_fault",
            }.get(operation)
            if service is None:
                raise ValueError("UNKNOWN_MANAGEMENT_OPERATION")
            if operation == "reference" and self.control_profile == "servo":
                raise ValueError("EXTERNAL_POSITION_CONTROL_UNAVAILABLE")
            if operation == "arm":
                with self._lock:
                    if not self._stop_settled_locked():
                        raise RuntimeError("WAIT_FOR_STOP_ACK")
                    if self.control_profile == "servo" and (
                        not self._servo_capable
                        or self._backend != self.servo_settings.backend
                        or self._state is None
                        or self._state.applied_config_revision != self.servo_settings.revision
                    ):
                        raise RuntimeError("SERVO_CAPABILITY_CONFIG_MISMATCH")
                    if self.control_profile == "servo" and self._backend == "hardware":
                        held = self.servo_settings.manual_hold_enabled
                        if input_quality is None:
                            raise RuntimeError("HARDWARE_INPUT_NOT_QUALIFIED")
                        if held and (
                            input_quality.source_kind != "JOYDEV_HELD_ARM"
                            or input_quality.link_evidence != "UNKNOWN"
                            or input_quality.qualification_id
                            or input_quality.episode_end_ns != 0
                            or input_quality.valid_until_ns - input_quality.observation_ns
                            > int(self.servo_settings.manual_input_max_age_s * 1e9)
                        ):
                            raise RuntimeError("HARDWARE_INPUT_NOT_QUALIFIED")
                        if not held and (
                            input_quality.source_kind != "QUALIFIED_STATE_CONTRACT"
                            or not input_quality.qualification_id
                        ):
                            raise RuntimeError("HARDWARE_INPUT_NOT_QUALIFIED")
                        if time.monotonic_ns() >= input_quality.valid_until_ns:
                            raise RuntimeError("ARM_INPUT_EXPIRED")
            if operation == "disarm":
                self.request_stop("MANAGEMENT_" + operation.upper())
                epoch = self.stop_epoch
            # ARM may rotate generation. Pause renewals, but never hold the
            # context mutex while awaiting management; B uses the STOP alias.
            context, token = self.context()
            import math

            request = ManageDevice.Request(
                context=context,
                token=token,
                axis_mask=0 if self.control_profile == "servo" else 3,
                linear_m=float(linear_mm) / 1000.0,
                rotation_rad=math.radians(float(rotation_deg)),
            )
            if input_quality is not None:
                request.has_input_quality = True
                request.input_quality = quality_to_ros(input_quality)
            response = self._call(service, request)
            with self._lock:
                self._merge_state_locked(response.state, authoritative=True)
                c = self._credentials
                if (
                    c is not None
                    and c.session_id == context.session_id
                    and response.state.generation
                ):
                    c.generation = response.state.generation
            self._require(response)
            if operation == "arm":
                if self.stop_epoch != epoch:
                    self.request_stop("ARM_INTERRUPTED")
                    raise RuntimeError("ARM_INTERRUPTED_BY_STOP")
                ack_context, ack_token = self.context()
                self._require(
                    self._call(
                        "acknowledge_generation",
                        ControlSession.Request(context=ack_context, token=ack_token),
                    )
                )
                if self.stop_epoch != epoch:
                    self.request_stop("ARM_ACK_INTERRUPTED")
                    raise RuntimeError("ARM_INTERRUPTED_BY_STOP")
            if operation in ("reset", "reference"):
                self.error = ""
        except Exception:
            if operation == "arm":
                self.request_stop("ARM_FAILED")
                self._ack_failed_arm_receipt()
            raise
        finally:
            with self._lock:
                self._management_active = False
                self._release_in_progress = False
            self._management_lock.release()

    def _ack_failed_arm_receipt(self):
        """Acknowledge receipt only after STOP, never recover ARM or input gates."""
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline and not self._closing.is_set():
            with self._lock:
                self._confirm_stop_locked()
                stopped = (
                    self._stop_settled_locked()
                    and self._state is not None
                    and self._state.stop.progress == "CONFIRMED"
                )
            if stopped:
                try:
                    context, token = self.context()
                    self._require(
                        self._call(
                            "acknowledge_generation",
                            ControlSession.Request(context=context, token=token),
                        )
                    )
                    self.notify(
                        {
                            "kind": "canceled_arm_generation_received",
                            "generation": context.generation,
                            "monotonic_ns": time.monotonic_ns(),
                        }
                    )
                except Exception:
                    pass  # Handover timeout remains a stop/revocation fallback.
                return
            self._closing.wait(0.01)

    def _wait_stop_confirmed(self, *, disarmed=False):
        deadline = time.monotonic() + self.timeout_s
        while not self._closing.is_set():
            with self._lock:
                self._confirm_stop_locked()
                if (
                    self._stop_settled_locked()
                    and self._state is not None
                    and self._state.stop.progress == "CONFIRMED"
                    and (not disarmed or not self._state.armed)
                    and time.monotonic_ns() - self._state_ns < 500_000_000
                ):
                    return
            if time.monotonic() >= deadline:
                raise TimeoutError("STOP_CONFIRMATION_TIMEOUT")
            self._closing.wait(0.01)
        raise RuntimeError("SESSION_CLOSING")

    def close(self):
        # Keep the executor alive while this bounded stop/release runs.
        clean = True
        if self.identity()[0]:
            self.request_stop("OWNER_EXIT")
            try:
                self.manage("release")
            except Exception as exc:
                self.error = "RELEASE_FAILED:" + str(exc)[:160]
                clean = False
                self.notify(
                    {
                        "kind": "shutdown_error",
                        "error": self.error,
                        "monotonic_ns": time.monotonic_ns(),
                    }
                )
        self._closing.set()
        self._renew_timer.cancel()
        with self._lock:
            outstanding = list(self._owned_futures)
            self._stop_future = self._renew_future = self._capabilities_future = None
            self._stop_latest = None
        for future in outstanding:
            self._abandon(future)
        return clean

    def simulation_fault(self, name, enabled):
        if (
            self.backend != "simulation"
            or name not in ("arm_pause", "control_freeze", "telemetry_pause", "encoder_freeze")
            or type(enabled) is not bool
        ):
            raise ValueError("SIMULATION_FAULT_REJECTED")
        context, token = self.context()
        return self._require(
            self._call(
                "fault",
                InjectFault.Request(context=context, token=token, name=name, enabled=enabled),
            )
        )
