"""ROS-independent native-RPM feed runtime with one physical I/O owner.

There is no external encoder, pulse engine, reference or position controller.
The control owner writes business state; the I/O owner independently expires
outputs and disables torque even if control computation stops progressing.
"""

from collections import OrderedDict, deque
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from math import isfinite, ulp
from queue import Empty, Full, Queue
from secrets import token_urlsafe
from threading import Event, Lock, Thread
from time import monotonic_ns
from uuid import uuid4

from gca.acquisition.buffers import Latest
from gca.algorithms.control import slew
from gca.domain.manual import QualityTracker
from gca.domain.models import DeviceError, OperationResult
from gca.domain.servo import (
    MANUAL_SERVO_VELOCITY,
    FeedRuntimeState,
    ServoJogCommand,
    ServoState,
)
from gca.servo_config import ServoSettings

from .authority import RuntimeAuthority


@dataclass(frozen=True)
class _VelocityDemand:
    rpm: float
    seq: int
    generation: int
    expires_ns: int
    feedback_ns: int
    stop_serial: int


class FeedAxisRuntime(RuntimeAuthority):
    """One native servo, shared executor authority, no automatic motion.

    Construction is disconnected. Host adapters authenticate leases before
    arm/renew and use the existing motion-permit layer before jog. Control
    policy is independently checked here. All device operations, including
    initialization, mode writes and final cleanup, belong to one I/O thread.
    """

    allowed_control_policies = (MANUAL_SERVO_VELOCITY,)

    def __init__(self, config: ServoSettings, notify=None, drive=None, clock=monotonic_ns):
        if not isinstance(config, ServoSettings):
            raise ValueError("validated ServoSettings required")
        self.config, self.notify, self.clock = config, notify, clock
        self.drive = drive
        self._lock, self._stop_lock = Lock(), Lock()
        self._wake, self._shutdown, self._ready = Event(), Event(), Event()
        self._control_paused, self._cleanup_failed = Event(), Event()
        self._threads = []
        self._started = False
        self._freeze_control = False
        self._start_error = None
        self._active_lifecycle = False
        self._lease, self._lease_deadline = None, 0
        self._revoked_generations, self._stop_fences = OrderedDict(), OrderedDict()
        self._history, self._highwater = OrderedDict(), {}
        self._quality_tracker = QualityTracker(
            backend=config.backend,
            max_age_s=config.manual_input_max_age_s,
            capacity=config.history_capacity,
            allow_joydev_hold=config.manual_hold_enabled,
        )
        self._commands, self._io_events = Queue(config.queue_capacity), Queue(config.queue_capacity)
        self._sample, self._demand = Latest(), Latest()
        self._records = deque(maxlen=4000)
        self._dropped_records = 0
        self._stop_serial = self._handled_stop_serial = self._armed_stop_serial = 0
        self._stop_request = None
        self._stopping = None
        self._permit = self._torque_requested = self._servo_enabled = False
        self._output_generation = self._output_seq = 0
        self._input_deadline = self._jog_deadline = 0
        self._arm_input_deadline = 0
        self._jog = None
        self._limited_rpm = self._accepted_rpm = 0.0
        self._last_output_ns = 0
        self._emergency = ""
        self._last_stop_reason = ""
        self._device_epoch = ""
        self._state = FeedRuntimeState(
            boot_id=str(uuid4()),
            backend=config.backend,
            config_revision=config.revision,
            requested_config_revision=config.revision,
            applied_config_revision=config.revision,
            calibration_id="",
        )

    @property
    def stop_serial(self):
        return self._stop_serial

    def start(self):
        if self._started:
            return OperationResult(True)
        if self._shutdown.is_set():
            return OperationResult(False, "REINITIALIZE_REQUIRED")
        thread = Thread(target=self._io_owner, name="gca-feed-servo-io", daemon=True)
        self._threads.append(thread)
        thread.start()
        if not self._ready.wait(3.0) or self._start_error:
            code, message = self._start_error or ("INITIALIZATION_TIMEOUT", "")
            self._shutdown.set()
            thread.join(self.config.stop_timeout_s)
            self._state = replace(
                self._state,
                state="FAULT",
                error_code=code,
                message=message,
                stop_progress="UNKNOWN",
                stop_evidence="UNKNOWN",
            )
            return OperationResult(False, code, message)
        sample = self._sample.get()
        if (
            not self._fresh(sample, self.clock())
            or sample.enabled
            or abs(sample.velocity_rpm) > self.config.velocity_tolerance_rpm
        ):
            self._shutdown.set()
            thread.join(self.config.stop_timeout_s)
            self._state = replace(
                self._state,
                state="FAULT",
                error_code="INITIAL_FEEDBACK_INVALID",
                stop_progress="UNKNOWN",
                stop_evidence="UNKNOWN",
            )
            return OperationResult(False, "INITIAL_FEEDBACK_INVALID")
        self._started = True
        self._state = replace(
            self._state,
            state="DISARMED",
            configured=True,
            stop_progress="CONFIRMED",
            stop_evidence="MEASURED_SERVO_ZERO_TORQUE_OFF",
        )
        self._records.append(
            {
                "time_ns": self.clock(),
                "kind": "servo_initialized",
                "backend": self.config.backend,
                "config_sha256": self.config.sha256,
                "identity": asdict(self.drive.identity),
                "configuration_readback": dict(self.drive.configuration_readback),
            }
        )
        self._publish_state(self.clock())
        control = Thread(target=self._control_owner, name="gca-feed-control", daemon=True)
        self._threads.append(control)
        control.start()
        return OperationResult(True)

    def _invoke(self, kind, value=None, timeout_s=0.5):
        if not self._started or self._shutdown.is_set():
            return OperationResult(False, "NOT_RUNNING")
        done, holder = Event(), []
        try:
            self._commands.put_nowait((kind, value, done, holder, self.stop_serial))
        except Full:
            return OperationResult(False, "QUEUE_FULL")
        self._wake.set()
        if not done.wait(timeout_s):
            done.set()
            return OperationResult(False, "CONTROL_REQUEST_TIMEOUT")
        return holder[0]

    def activate(self, active=True):
        if type(active) is not bool:
            return OperationResult(False, "INVALID_ARGUMENT")
        if not active:
            self.stop("DEACTIVATE", disarm=True)
        return self._invoke("active", active)

    set_active = activate

    def arm(self, lease, expected_stop_serial=None, input_deadline_ns=0):
        expected = self.stop_serial if expected_stop_serial is None else expected_stop_serial
        return self._invoke("arm", (lease, expected, input_deadline_ns))

    def renew(self, lease):
        return self._invoke("renew", lease)

    def authorize(self, lease):
        return self.renew(lease) if self._lease else self.arm(lease)

    def jog(self, command):
        if not isinstance(command, ServoJogCommand):
            return OperationResult(False, "SERVO_COMMAND_REQUIRED")
        if command.deadman is False:
            result = self._validate_manual_permission(
                command.context,
                command.freshness_token,
                self.clock(),
            )
            return self.stop("DEADMAN_RELEASED", context=command.context) if result else result
        return self._invoke("jog", command)

    def stop(self, reason="STOP", command_id="", disarm=False, *, context=None):
        if not self._started or self._shutdown.is_set():
            return OperationResult(False, "NOT_RUNNING")
        if command_id and self._jog and command_id != self._jog.context.command_id:
            return OperationResult(False, "COMMAND_MISMATCH")
        if (
            context is not None
            and self._lease is not None
            and (
                context.session_id != self._lease.session_id
                or context.generation != self._lease.generation
            )
        ):
            return OperationResult(False, "INVALID_PERMISSION")
        self._latch_stop(reason, disarm=disarm, context=context)
        return OperationResult(True, "STOP_ACCEPTED")

    def _latch_stop(self, reason, disarm=False, context=None, fault=False):
        self._permit = self._torque_requested = False
        self._demand.put(None)
        with self._stop_lock:
            # Reapply inside the transaction fence: a command writer may have
            # held the lock between the immediate revocation and this point.
            self._permit = self._torque_requested = False
            self._demand.put(None)
            if context is not None:
                key = (context.session_id, context.generation)
                self._stop_fences[key] = max(context.sequence, self._stop_fences.get(key, -1))
                while len(self._stop_fences) > self.config.history_capacity:
                    self._stop_fences.popitem(last=False)
            if self._stop_request is not None:
                if self._stop_request[2]:
                    reason = self._stop_request[0]
                disarm = disarm or self._stop_request[1]
                fault = fault or self._stop_request[2]
            self._stop_serial += 1
            self._stop_request = (reason, disarm, fault, self._stop_serial)
        self._wake.set()

    def disarm(self):
        return self.stop("DISARM", disarm=True)

    def reset(self):
        return self._invoke("reset")

    reset_fault = reset

    def set_reference(self, axis_mask=3, linear_mm=0.0, rotation_deg=0.0):
        return OperationResult(False, "EXTERNAL_ENCODER_REQUIRED")

    def reserve(self, command):
        return OperationResult(False, "POSITION_CONTROL_UNAVAILABLE")

    start_motion = reserve

    def release_reservation(self, command_id):
        return OperationResult(False, "POSITION_CONTROL_UNAVAILABLE")

    def configure(self, config):
        if self._state.armed or self._state.stop_progress != "CONFIRMED":
            return OperationResult(False, "CONFIGURATION_BUSY")
        return OperationResult(
            False, "RECONFIGURE_REQUIRED", "export snapshot and lifecycle cleanup/configure"
        )

    def drain_events(self):
        # Position Action is explicitly unavailable, hence no terminal events.
        return []

    pop_events = drain_events

    def commit_terminal(self, command_id, outcome):
        return OperationResult(False, "POSITION_CONTROL_UNAVAILABLE")

    def get_command_status(self, command_id):
        with self._lock:
            status = self._history.get(command_id)
        return OperationResult(bool(status), status or "UNKNOWN_COMMAND")

    command_status = get_command_status

    def snapshot(self):
        with self._lock:
            state = self._state
        now = self.clock()
        age = (now - state.servo.sampled_ns) / 1e9 if state.servo.sampled_ns else 1e9
        servo = replace(
            state.servo,
            sample_age_s=max(0.0, age),
            valid=state.servo.valid and 0 <= age <= self.config.feedback_max_age_s,
        )
        result = replace(
            state,
            servo=servo,
            sample_age_s=max(0.0, age),
            stop_serial=self.stop_serial,
            lease_remaining_s=max(0.0, (self._lease_deadline - now) / 1e9),
        )
        if self._handled_stop_serial < self.stop_serial:
            result = replace(
                result,
                stop_progress="REQUESTED",
                stop_evidence="",
                servo=replace(servo, requested_velocity_rpm=0.0),
            )
        return result

    def records(self):
        with self._lock:
            return list(self._records)

    def simulation_evidence(self):
        if self.config.backend != "simulation" or self.drive is None:
            return {"available": False, "evidence_kind": "SIMULATION_ONLY"}
        sample = self._sample.get()
        return {
            "available": True,
            "evidence_kind": "SIMULATED_SERVO_FEEDBACK",
            "sample": asdict(sample) if sample is not None else None,
            "control_paused": self._control_paused.is_set(),
        }

    def inject_fault(self, name, value=True):
        if self.config.backend != "simulation":
            return OperationResult(False, "SIMULATION_ONLY")
        if name == "control_freeze" and type(value) is bool:
            self._freeze_control = value
            self._wake.set()
            if value and not self._control_paused.wait(0.5):
                return OperationResult(False, "CONTROL_PAUSE_TIMEOUT")
            return OperationResult(True)
        return OperationResult(False, "UNKNOWN_FAULT")

    def close(self, timeout_s=2.0):
        deadline = monotonic_ns() + int(timeout_s * 1e9)
        if not self._started:
            self._shutdown.set()
            self._wake.set()
            for thread in self._threads:
                thread.join(max(0.0, (deadline - monotonic_ns()) / 1e9))
            if any(thread.is_alive() for thread in self._threads):
                return OperationResult(False, "SHUTDOWN_TIMEOUT")
            self._consume_io_receipts()
            self._publish_state(self.clock())
            if self._cleanup_failed.is_set():
                return OperationResult(False, "DEVICE_CLEANUP_FAILED")
            # Resource cleanup succeeds without asserting physical stop. A
            # failed initialization retains UNKNOWN in its state snapshot.
            return OperationResult(True, "NOT_RUNNING")
        self._freeze_control = False
        self.stop("SHUTDOWN", disarm=True)
        while self.snapshot().stop_progress not in ("CONFIRMED", "UNKNOWN"):
            if monotonic_ns() >= deadline:
                break
            self._shutdown.wait(0.01)
        confirmed = self.snapshot().stop_progress == "CONFIRMED"
        self._shutdown.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(max(0.0, (deadline - monotonic_ns()) / 1e9))
        self._started = False
        if any(thread.is_alive() for thread in self._threads):
            return OperationResult(False, "SHUTDOWN_TIMEOUT")
        # The I/O owner can finish a confirmed zero write during final cleanup
        # after the control thread exits. Preserve that receipt without turning
        # a write acknowledgment into physical stop confirmation.
        self._consume_io_receipts()
        self._publish_state(self.clock())
        if self._cleanup_failed.is_set():
            return OperationResult(False, "DEVICE_CLEANUP_FAILED")
        return OperationResult(confirmed, "OK" if confirmed else "STOP_UNKNOWN")

    def _measurement_fresh(self, sample, now):
        return bool(
            sample is not None
            and sample.valid
            and sample.seq > 0
            and sample.device_epoch == self._device_epoch
            and 0 <= now - sample.received_ns <= int(self.config.feedback_max_age_s * 1e9)
            and isfinite(sample.velocity_rpm)
        )

    def _fresh(self, sample, now):
        return bool(
            self._measurement_fresh(sample, now)
            and not sample.hardware_error
            and not sample.error_code
            and sample.watchdog_raw == self.config.device.watchdog_raw
        )

    def _handle(self, kind, value, now):
        if kind == "active":
            self._active_lifecycle = value
            self._state = replace(self._state, lifecycle_active=value)
            return OperationResult(True)
        if kind in ("arm", "renew"):
            expected = self.stop_serial
            if kind == "arm":
                value, expected, input_deadline = value
                if type(expected) is not int or expected != self.stop_serial:
                    return OperationResult(False, "STOP_SUPERSEDED")
                if type(input_deadline) is not int or input_deadline < 0:
                    return OperationResult(False, "INPUT_DEADLINE_INVALID")
                if self.config.backend == "hardware" and not now < input_deadline:
                    return OperationResult(False, "INPUT_NOT_QUALIFIED")
            result = self._validate_lease(value, now, renewal=kind == "renew")
            if not result:
                return result
            if kind == "arm":
                sample = self._sample.get()
                if not self._active_lifecycle or self._state.state != "DISARMED" or self._emergency:
                    return OperationResult(False, "ARM_REJECTED")
                if (
                    not self._fresh(sample, now)
                    or sample.enabled
                    or abs(sample.velocity_rpm) > self.config.velocity_tolerance_rpm
                ):
                    return OperationResult(False, "SERVO_NOT_STATIONARY")
                if self._state.stop_progress != "CONFIRMED":
                    return OperationResult(False, "STOP_NOT_CONFIRMED")
                value = replace(value, token=value.token or token_urlsafe(24))
                with self._stop_lock:
                    if expected != self.stop_serial or self._stop_request is not None:
                        return OperationResult(False, "STOP_SUPERSEDED")
                    self._lease = value
                    self._lease_deadline = now + int(value.ttl_s * 1e9)
                    self._armed_stop_serial = expected
                    self._arm_input_deadline = input_deadline
                    self._torque_requested = True
                    self._highwater = {
                        key: seq
                        for key, seq in self._highwater.items()
                        if key == (value.session_id, value.generation)
                    }
                    self._state = replace(
                        self._state,
                        armed=True,
                        state="READY",
                        session_id=value.session_id,
                        generation=value.generation,
                        error_code="",
                    )
            else:
                self._lease = value
                self._lease_deadline = now + int(value.ttl_s * 1e9)
            return OperationResult(True, token=value.token)
        if kind == "jog":
            return self._accept_jog(value, now)
        if kind == "reset":
            if self._stopping or self._state.stop_progress != "CONFIRMED":
                return OperationResult(False, "STOP_NOT_CONFIRMED")
            if self._state.state not in ("FAULT", "DISARMED"):
                return OperationResult(False, "RESET_BUSY")
            if not self._fresh(self._sample.get(), now) or not self._threads[0].is_alive():
                return OperationResult(False, "DEVICE_REINITIALIZE_REQUIRED")
            self._revoke_current_lease()
            self._lease, self._lease_deadline = None, 0
            self._emergency = ""
            self._state = replace(self._state, state="DISARMED", armed=False, error_code="")
            return OperationResult(True)
        return OperationResult(False, "UNKNOWN_OPERATION")

    def _accept_jog(self, command, now):
        serial = self.stop_serial
        if (
            not isinstance(command, ServoJogCommand)
            or self._stopping
            or self._state.state == "FAULT"
        ):
            return OperationResult(False, "CONTROL_BUSY")
        result = self._validate_manual_permission(command.context, command.freshness_token, now)
        if not result:
            return result
        if self._lease.control_policy != MANUAL_SERVO_VELOCITY:
            return OperationResult(False, "CONTROL_POLICY_MISMATCH")
        if (
            command.deadman is not True
            or type(command.velocity_rpm) not in (float, int)
            or not isfinite(command.velocity_rpm)
            # RPM -> rad/s -> RPM may move a boundary value by a few float
            # representable steps. Accept only that roundoff, then clamp below.
            or abs(command.velocity_rpm) > self.config.max_rpm + 4 * ulp(self.config.max_rpm)
            or type(command.ttl_s) not in (float, int)
            or not isfinite(command.ttl_s)
            or not 0 < command.ttl_s <= 0.5
            or type(command.admission_deadline_ns) is not int
            or command.admission_deadline_ns < 0
        ):
            return OperationResult(False, "INVALID_SERVO_JOG")
        command = replace(
            command,
            velocity_rpm=max(-self.config.max_rpm, min(self.config.max_rpm, command.velocity_rpm)),
        )
        if command.admission_deadline_ns and now >= command.admission_deadline_ns:
            return OperationResult(False, "COMMAND_EXPIRED")
        context = command.context
        key = (context.session_id, context.generation)
        if context.sequence <= self._highwater.get(key, -1):
            return OperationResult(False, "STALE_SEQUENCE")
        if not self._fresh(self._sample.get(), now):
            return OperationResult(False, "FEEDBACK_STALE")
        try:
            source_kind = command.quality.source_kind if command.quality else ""
            if source_kind == "JOYDEV_HELD_ARM":
                return OperationResult(False, "ARM_INPUT_CANNOT_JOG")
            expected_kind = (
                "JOYDEV_HELD_STATE"
                if self.config.manual_hold_enabled
                else "QUALIFIED_STATE_CONTRACT"
            )
            if (self.config.backend == "hardware" or self.config.manual_hold_enabled) and (
                source_kind != expected_kind
            ):
                return OperationResult(False, "INPUT_NOT_QUALIFIED")
            input_deadline = self._quality_tracker.validate(
                command.quality, now, session_id=context.session_id, generation=context.generation
            )
        except ValueError as exc:
            return OperationResult(False, getattr(exc, "code", "INVALID_MANUAL_QUALITY"), str(exc))
        with self._stop_lock:
            if serial != self.stop_serial or self._stop_request is not None:
                return OperationResult(False, "STOP_SUPERSEDED")
            self._highwater[key] = context.sequence
            if self._jog is None:
                self._output_generation += 1
                self._limited_rpm = 0.0
                self._last_output_ns = now
                self._armed_stop_serial = serial
            self._jog = command
            self._input_deadline = input_deadline
            self._jog_deadline = min(
                now + int(command.ttl_s * 1e9),
                input_deadline,
                self._lease_deadline,
                command.admission_deadline_ns or self._lease_deadline,
            )
            self._permit = self._torque_requested = True
            self._state = replace(
                self._state,
                state="EXECUTING",
                task_id=context.task_id,
                command_id=context.command_id,
            )
            self._history[context.command_id] = "EXECUTING"
            while len(self._history) > self.config.history_capacity:
                self._history.popitem(last=False)
        return OperationResult(True)

    def _control_owner(self):
        while not self._shutdown.is_set():
            if self._freeze_control:
                self._control_paused.set()
            else:
                self._control_paused.clear()
                try:
                    self._tick()
                except Exception as exc:
                    self._state = replace(self._state, message=str(exc))
                    self._latch_stop("CONTROL_EXCEPTION", disarm=True, fault=True)
            self._wake.wait(self.config.control_period_s)
            self._wake.clear()

    def _tick(self):
        now = self.clock()
        with self._stop_lock:
            pending = self._stop_request
            self._stop_request = None
        if pending:
            reason, disarm, fault, serial = pending
            self._begin_stop(reason, now, disarm=disarm, fault=fault)
            self._handled_stop_serial = serial
        for _ in range(self.config.queue_capacity):
            try:
                kind, value, done, holder, serial = self._commands.get_nowait()
            except Empty:
                break
            if done.is_set():
                continue
            try:
                result = (
                    OperationResult(False, "STOP_SUPERSEDED")
                    if kind in ("arm", "jog") and serial != self.stop_serial
                    else self._handle(kind, value, self.clock())
                )
            except Exception as exc:
                result = OperationResult(False, getattr(exc, "code", "INVALID_ARGUMENT"), str(exc))
            holder.append(result)
            done.set()
        self._consume_io_receipts()
        now = self.clock()
        if self._state.armed and now >= self._lease_deadline and self._stopping is None:
            self._latch_stop("LEASE_EXPIRED", disarm=True)
        if (
            self._state.armed
            and not self._fresh(self._sample.get(), now)
            and self._stopping is None
        ):
            self._latch_stop("FEEDBACK_STALE", disarm=True, fault=True)
        if self._stopping is not None:
            self._advance_stop(now)
        elif self._jog is not None and self._stop_request is None:
            if now >= self._jog_deadline:
                self._latch_stop("JOG_EXPIRED")
            else:
                dt = (now - self._last_output_ns) / 1e9
                if dt > self.config.output_ttl_s:
                    self._latch_stop("CONTROL_DT_EXPIRED", disarm=True, fault=True)
                elif dt > 0:
                    sample = self._sample.get()
                    self._limited_rpm = slew(
                        self._jog.velocity_rpm,
                        self._limited_rpm,
                        self.config.max_acceleration_rpm_s,
                        dt,
                    )
                    self._output_seq += 1
                    self._last_output_ns = now
                    self._demand.put(
                        _VelocityDemand(
                            self._limited_rpm,
                            self._output_seq,
                            self._output_generation,
                            min(self._jog_deadline, now + int(self.config.output_ttl_s * 1e9)),
                            sample.received_ns,
                            self.stop_serial,
                        )
                    )
        self._publish_state(now)

    def _consume_io_receipts(self):
        for _ in range(self.config.queue_capacity):
            try:
                seq, generation, serial, rpm = self._io_events.get_nowait()
            except Empty:
                break
            # One I/O owner enqueues these acknowledgments in device-write
            # order. Independent STOP can write zero before the control owner
            # advances output_generation, so its successful write must survive
            # that revocation. An earlier zero receipt cannot overwrite a later
            # consumed motion receipt because this channel is FIFO.
            stop_write = seq == 0 and serial <= self.stop_serial
            current_motion = generation == self._output_generation and serial == self.stop_serial
            if stop_write or current_motion:
                self._accepted_rpm = rpm

    def _begin_stop(self, reason, now, *, disarm=False, fault=False):
        self._permit = self._torque_requested = False
        self._demand.put(None)
        if self._stopping is not None:
            if fault and not self._stopping["fault"]:
                self._stopping["reason"] = reason
            self._stopping["disarm"] |= disarm or fault
            self._stopping["fault"] |= fault
            self._last_stop_reason = self._stopping["reason"] if self._stopping["fault"] else reason
            return
        latched = self._state.state == "FAULT"
        if latched:
            reason = self._state.error_code
        self._last_stop_reason = reason
        if latched and self._state.stop_progress == "CONFIRMED":
            if disarm:
                self._revoke_current_lease()
                self._lease_deadline = 0
                self._state = replace(self._state, armed=False)
            return
        if self._jog:
            self._history[self._jog.context.command_id] = reason
        self._jog = None
        self._input_deadline = self._jog_deadline = 0
        self._limited_rpm = 0.0
        self._output_generation += 1
        self._stopping = {
            "start": now,
            "deadline": now + int(self.config.stop_timeout_s * 1e9),
            "disarm": disarm or fault or not self._state.armed,
            "fault": fault or latched,
            "reason": self._state.error_code if latched else reason,
            "settled": None,
            "settled_seq": 0,
        }
        self._state = replace(
            self._state,
            state="FAULT" if fault or latched else "STOPPING",
            stop_progress="IN_PROGRESS",
            stop_evidence="",
            error_code=reason if fault else self._state.error_code,
        )

    def _advance_stop(self, now):
        stop, sample = self._stopping, self._sample.get()
        stationary = (
            self._measurement_fresh(sample, now)
            and sample.received_ns >= stop["start"]
            and not sample.enabled
            and abs(sample.velocity_rpm) <= self.config.velocity_tolerance_rpm
        )
        if stationary:
            if stop["settled"] is None:
                stop["settled"], stop["settled_seq"] = sample.received_ns, sample.seq
        else:
            stop["settled"] = None
        confirmed = bool(
            stationary
            and stop["settled"] is not None
            and sample.seq > stop["settled_seq"]
            and sample.received_ns - stop["settled"] >= int(self.config.settle_s * 1e9)
        )
        if not confirmed and now < stop["deadline"]:
            return
        disarm = stop["disarm"] or not confirmed
        if disarm:
            self._revoke_current_lease()
            self._lease_deadline = 0
        self._stopping = None
        self._state = replace(
            self._state,
            state="FAULT"
            if stop["fault"] or not confirmed
            else ("DISARMED" if disarm else "READY"),
            armed=not disarm,
            stop_progress="CONFIRMED" if confirmed else "UNKNOWN",
            stop_evidence="MEASURED_SERVO_ZERO_TORQUE_OFF" if confirmed else "UNKNOWN",
            error_code="STOP_TIMEOUT"
            if not confirmed
            else (stop["reason"] if stop["fault"] else ""),
        )

    def _publish_state(self, now):
        sample = self._sample.get()
        servo = self._state.servo
        if sample is not None:
            servo = ServoState(
                raw_position_count=sample.raw_position,
                shaft_angle_rad=sample.angle_rad,
                measured_velocity_rpm=sample.velocity_rpm,
                requested_velocity_rpm=self._jog.velocity_rpm if self._jog else 0.0,
                accepted_velocity_rpm=self._accepted_rpm,
                torque_enabled=sample.enabled,
                hardware_error=sample.hardware_error,
                sample_seq=sample.seq,
                sampled_ns=sample.received_ns,
                device_epoch=sample.device_epoch,
                valid=self._fresh(sample, now),
                configured=self._state.configured,
                sample_age_s=max(0.0, (now - sample.received_ns) / 1e9),
                error_code=sample.error_code,
                watchdog_raw=sample.watchdog_raw,
            )
        with self._lock:
            self._state = replace(
                self._state,
                servo=servo,
                sample_seq=servo.sample_seq,
                sample_received_ns=servo.sampled_ns,
                sample_age_s=servo.sample_age_s,
                device_epoch=servo.device_epoch,
                state_seq=self._state.state_seq + 1,
                output_generation=self._output_generation,
                output_seq=self._output_seq,
                generation=self._lease.generation if self._lease else 0,
                stop_serial=self.stop_serial,
                input_valid_until_ns=self._input_deadline,
                dropped_records=self._dropped_records,
            )
            if len(self._records) == self._records.maxlen:
                self._dropped_records += 1
            self._records.append(
                {
                    "time_ns": now,
                    **asdict(self._state),
                    "limited_velocity_rpm": self._limited_rpm,
                    "stop_reason": self._last_stop_reason,
                    "config_sha256": self.config.sha256,
                }
            )

    def _valid_output(self, demand, now):
        return bool(
            demand
            and self._permit
            and self._torque_requested
            and self._stop_request is None
            and self._armed_stop_serial == self.stop_serial
            and demand.stop_serial == self.stop_serial
            and self._active_lifecycle
            and self._state.armed
            and self._lease is not None
            and self._lease.control_policy == MANUAL_SERVO_VELOCITY
            and now < min(demand.expires_ns, self._lease_deadline, self._input_deadline)
            and demand.generation == self._output_generation
            and 0 <= now - demand.feedback_ns <= int(self.config.feedback_max_age_s * 1e9)
        )

    def _enable_allowed(self, deadline_ns, serial, lease_identity, for_jog):
        """Read-only final-write guard; called by the owning device transaction.

        STOP clears torque_requested before waiting on the transaction lock,
        so a long configuration read cannot subsequently start a new enable.
        """
        with self._stop_lock:
            now, sample = self.clock(), self._sample.get()
            return bool(
                self._torque_requested
                and self._active_lifecycle
                and self._state.armed
                and self._stop_request is None
                and serial == self.stop_serial
                and self._armed_stop_serial == serial
                and self._lease is not None
                and (self._lease.session_id, self._lease.generation) == lease_identity
                and self._lease.control_policy == MANUAL_SERVO_VELOCITY
                and now < min(deadline_ns, self._lease_deadline)
                and (
                    (self._jog is None and not for_jog)
                    or (
                        self._jog is not None
                        and self._permit
                        and now < min(self._input_deadline, self._jog_deadline)
                    )
                )
                and self._fresh(sample, now)
                and abs(sample.velocity_rpm) <= self.config.velocity_tolerance_rpm
            )

    @contextmanager
    def _velocity_write_guard(self, demand, deadline_ns):
        """Fence only the actual transmission; never hold it while awaiting ACK."""
        with self._stop_lock:
            now = self.clock()
            if now >= deadline_ns or not self._valid_output(demand, now):
                raise DeviceError("OUTPUT_SUPERSEDED", "velocity revoked before transmission")
            yield

    def _safe_stop_io(self):
        errors = []
        try:
            self.drive.stop(self.clock() + int(self.config.io_timeout_s * 1e9))
            self._io_events.put_nowait((0, self._output_generation, self.stop_serial, 0.0))
        except Exception as exc:
            errors.append(getattr(exc, "code", "SERVO_ZERO_FAILED"))
        try:
            self.drive.set_torque(False, self.clock() + int(self.config.io_timeout_s * 1e9))
            self._servo_enabled = False
        except Exception as exc:
            errors.append(getattr(exc, "code", "SERVO_DISABLE_FAILED"))
        return errors

    def _read_io(self, deadline_ns=None):
        if deadline_ns is None:
            deadline_ns = self.clock() + int(self.config.io_timeout_s * 1e9)
        sample = self.drive.read_shaft_state(deadline_ns)
        previous = self._sample.get()
        if sample.device_epoch != self._device_epoch:
            raise DeviceError("DEVICE_EPOCH_CHANGED")
        if previous is not None and (
            sample.seq <= previous.seq or sample.received_ns <= previous.received_ns
        ):
            raise DeviceError("SERVO_SAMPLE_REPLAY")
        if sample.received_ns > self.clock():
            raise DeviceError("SERVO_SAMPLE_TIME_INVALID")
        self._sample.put(sample)
        self._servo_enabled = sample.enabled
        return sample

    def _io_owner(self):
        last_seq, parked_serial = -1, -1
        opened = False
        try:
            if self.drive is None:
                if self.config.backend == "simulation":
                    from gca.simulation.servo import SimulatedVelocityDrive

                    self.drive = SimulatedVelocityDrive(self.config, clock=self.clock)
                else:
                    from gca.devices.dynamixel import DynamixelVelocityDrive

                    self.drive = DynamixelVelocityDrive(self.config.device, clock=self.clock)
            identity = self.drive.open_identified()
            opened = True
            self._device_epoch = identity.device_epoch
            self.drive.configure_velocity_mode()
            errors = self._safe_stop_io()
            if errors:
                raise DeviceError(errors[0])
            sample = self._read_io()
            if (
                not self._fresh(sample, self.clock())
                or sample.enabled
                or abs(sample.velocity_rpm) > self.config.velocity_tolerance_rpm
            ):
                raise DeviceError("INITIAL_FEEDBACK_INVALID")
            parked_serial = self.stop_serial
            self._ready.set()
            while not self._shutdown.is_set():
                cycle_started_ns = self.clock()
                io_operation, io_deadline_ns = "control_checks", 0
                try:
                    now = self.clock()
                    current_sample = self._sample.get()
                    if (
                        self._state.armed
                        and not self._fresh(current_sample, now)
                        and not self._emergency
                    ):
                        self._emergency = (
                            current_sample.error_code
                            if current_sample is not None and current_sample.error_code
                            else "FEEDBACK_STALE"
                        )
                        self._latch_stop(self._emergency, disarm=True, fault=True)
                    demand = self._demand.get()
                    if self._permit and (
                        demand is not None and not self._valid_output(demand, now)
                    ):
                        reason = "JOG_EXPIRED" if now >= self._jog_deadline else "OUTPUT_EXPIRED"
                        self._latch_stop(reason)
                    elif (
                        self._state.armed and now >= self._lease_deadline and self._torque_requested
                    ):
                        self._latch_stop("LEASE_EXPIRED", disarm=True)
                    want_torque = bool(
                        self._torque_requested
                        and self._active_lifecycle
                        and self._state.armed
                        and self._lease
                        and now < self._lease_deadline
                        and self._armed_stop_serial == self.stop_serial
                        and self._stop_request is None
                    )
                    if want_torque and not self._servo_enabled:
                        with self._stop_lock:
                            if self._lease is None:
                                raise DeviceError("STOP_SUPERSEDED")
                            enable_serial = self.stop_serial
                            enable_lease = (self._lease.session_id, self._lease.generation)
                            for_jog = self._jog is not None
                            # A zero-target handshake has a fixed transaction budget.
                            # Initial ARM retains its immutable original input proof;
                            # jog re-enable checks newly admitted inputs in the guard.
                            arm_deadline = min(
                                self.clock() + int(4 * self.config.io_timeout_s * 1e9),
                                self._lease_deadline,
                            )
                            if self.config.backend == "hardware" and not for_jog:
                                arm_deadline = min(arm_deadline, self._arm_input_deadline)

                        def enable_guard():
                            return self._enable_allowed(
                                arm_deadline,
                                enable_serial,
                                enable_lease,
                                for_jog,
                            )

                        if not enable_guard():
                            raise DeviceError("STOP_SUPERSEDED")
                        # Do not hold the command fence across serial I/O: control
                        # must be able to admit fresher input or process STOP here.
                        io_operation, io_deadline_ns = "enable", arm_deadline
                        self.drive.set_torque(True, arm_deadline, enable_guard=enable_guard)
                        self._servo_enabled = True
                    with self._stop_lock:
                        demand = self._demand.get()
                        now = self.clock()
                        write_deadline = now + int(self.config.io_timeout_s * 1e9)
                        # Leave near-expiry demands for a newer control update;
                        # do not start an ACK wait with a truncated I/O budget.
                        write_allowed = (
                            self._valid_output(demand, now)
                            and demand.seq != last_seq
                            and write_deadline
                            <= min(demand.expires_ns, self._input_deadline, self._lease_deadline)
                        )
                    if write_allowed:
                        io_operation, io_deadline_ns = "write_velocity", write_deadline
                        try:
                            accepted = self.drive.write_velocity_rpm(
                                demand.rpm,
                                write_deadline,
                                write_guard=lambda: self._velocity_write_guard(
                                    demand, write_deadline
                                ),
                            )
                        except DeviceError as exc:
                            if exc.code != "OUTPUT_SUPERSEDED":
                                raise
                            # Cancellation before TX is not a communication fault.
                            # Fresh accepted input may already have replaced it.
                            if self._permit and not self._valid_output(
                                self._demand.get(), self.clock()
                            ):
                                self._latch_stop("OUTPUT_EXPIRED")
                        else:
                            self._io_events.put_nowait(
                                (demand.seq, demand.generation, demand.stop_serial, accepted)
                            )
                            last_seq = demand.seq
                    if not self._torque_requested and (
                        parked_serial != self.stop_serial or self._servo_enabled
                    ):
                        io_operation, io_deadline_ns = "stop", 0
                        errors = self._safe_stop_io()
                        parked_serial = self.stop_serial
                        if errors:
                            raise DeviceError(errors[0])
                    io_operation = "read_feedback"
                    io_deadline_ns = self.clock() + int(self.config.io_timeout_s * 1e9)
                    self._read_io(io_deadline_ns)
                except Exception as exc:
                    reason = getattr(exc, "code", "SERVO_IO_ERROR")
                    with self._lock:
                        self._records.append(
                            {
                                "time_ns": self.clock(),
                                "kind": "servo_io_error",
                                "code": reason,
                                "message": str(exc),
                                "io_operation": io_operation,
                                "io_deadline_ns": io_deadline_ns,
                                "input_deadline_ns": self._input_deadline,
                                "arm_input_deadline_ns": self._arm_input_deadline,
                            }
                        )
                    if reason == "STOP_SUPERSEDED":
                        self._latch_stop(reason, disarm=True)
                    elif not self._emergency:
                        self._emergency = reason
                        self._latch_stop(reason, disarm=True, fault=True)
                    self._safe_stop_io()
                    parked_serial = self.stop_serial
                    try:
                        self._read_io()
                    except Exception:
                        pass  # Cached age grows; absent evidence cannot confirm stopping.
                remaining_s = self.config.io_period_s - (self.clock() - cycle_started_ns) / 1e9
                self._shutdown.wait(max(0.0, remaining_s))
        except Exception as exc:
            self._start_error = (getattr(exc, "code", "INITIALIZATION_FAILED"), str(exc))
            self._ready.set()
        finally:
            if self.drive is not None:
                # open_identified owns its failed-open cleanup. Do not issue
                # stop writes to a closed or unverified device identity.
                errors = self._safe_stop_io() if opened else []
                try:
                    self.drive.close()
                except Exception:
                    errors.append("SERVO_CLOSE_FAILED")
                if errors:
                    self._cleanup_failed.set()
