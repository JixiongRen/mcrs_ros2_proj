"""ROS-independent dual-axis runtime with bounded I/O ownership and leases.

Only the control thread mutates motion state. Public methods post short local
requests; each physical device is exclusively accessed by its own I/O owner.
The I/O owners validate command deadlines independently of control progress.
No ROS callback, controller calculation, or state lock performs device I/O.
"""

from __future__ import annotations

from collections import OrderedDict, deque
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from math import ceil, floor, isfinite
from queue import Empty, Full, Queue
from secrets import token_urlsafe
from threading import Event, Lock, Thread
from time import monotonic_ns
from typing import TypeAlias
from uuid import uuid4

from gca.acquisition.buffers import Latest
from gca.algorithms.control import PositionController, boundary_velocity, clamp, slew
from gca.algorithms.jog_rotation import JogRotationPlanner
from gca.algorithms.jog_velocity import JogVelocityController
from gca.algorithms.trajectory import PulseQuantizer, Trapezoid
from gca.config import RuntimeConfig
from gca.domain.manual import QualityTracker
from gca.domain.models import (
    CommandContext,
    DeviceError,
    JogCommand,
    Lease,
    MotionCommand,
    MotionEvent,
    OperationResult,
    PulseSegment,
    RuntimeState,
)
from gca.domain.ports import EncoderPort, PulseEngine, ServoPort
from gca.estimation.linear import LinearEstimator

from .io_owners import DeviceIOOwners

DeviceBundle: TypeAlias = (
    tuple[ServoPort, EncoderPort, PulseEngine] | tuple[ServoPort, EncoderPort, PulseEngine, object]
)


@dataclass(frozen=True)
class _Demand:
    velocity: float
    seq: int
    generation: int
    expires_ns: int
    feedback_ns: int
    stop_serial: int = 0


class ControlRuntime(DeviceIOOwners):
    """Facade consumed by ROS adapters; construction never connects hardware.

    ``devices`` optionally supplies (servo, encoder, pulse[, plant]). The clock
    is a callable returning local monotonic nanoseconds; device adapters must
    share that clock. Real ports must enforce their underlying transaction
    deadlines; the runtime cannot forcibly interrupt an unbounded SDK call.

    Management methods are safe to call from adapter threads: they queue short
    requests to the single control writer and return OperationResult rather
    than raising device failures. A false accepted flag/code describes refusal,
    queue overflow or the bounded local request timeout. Acceptance is never
    physical completion. start/close are lifecycle operations for the host,
    not motion callbacks; snapshot/records/event draining only hold short locks.
    """

    def __init__(
        self,
        config: RuntimeConfig | dict[str, object] | None = None,
        notify: Callable[[], None] | None = None,
        devices: DeviceBundle | None = None,
        clock: Callable[[], int] = monotonic_ns,
    ) -> None:
        """Construct disconnected runtime; clock returns local monotonic ns.

        Invalid configuration raises ValueError before any I/O. notify is a
        nonblocking adapter wakeup callback invoked for result availability.
        Device injection substitutes complete ports, never individual control
        decisions. No constructor or import opens hardware or produces output.
        """
        self.config = config or RuntimeConfig()
        if isinstance(self.config, dict):
            self.config = RuntimeConfig.from_dict(self.config)
        self.clock, self.notify = clock, notify
        self._devices = devices
        self._shutdown, self._wake = Event(), Event()
        self._cleanup_failed = Event()
        self._count_evidence_lost = Event()
        self._lock = Lock()
        self._stop_lock = Lock()
        self._stop_request = None
        self._stop_serial = 0
        self._handled_stop_serial = 0
        self._stop_fences = OrderedDict()
        self._armed_stop_serial = 0
        self._commands = Queue(self.config.queue_capacity)
        self._io_events = Queue(self.config.queue_capacity)
        self._results = deque()
        self._records = deque(maxlen=4000)
        self._state = RuntimeState(
            boot_id=str(uuid4()),
            backend=self.config.backend,
            calibration_id="simulation-only"
            if self.config.backend == "simulation"
            else self.config.revision,
            config_revision=self.config.revision,
            requested_config_revision=self.config.revision,
            applied_config_revision=self.config.revision,
        )
        self._sample, self._servo_sample = Latest(), Latest()
        self._velocity_demand, self._pulse_demand = Latest(), Latest()
        self._pulse_idle = True
        self._pulse_epoch = ""
        self._servo_enabled = False
        self._permit = False
        self._active_lifecycle = False
        self._lease = None
        self._lease_deadline = 0
        self._output_generation = 0
        self._emergency = ""
        self._pulse_stop = Event()
        self._ready = [Event(), Event(), Event()]
        self._threads = []
        self._started = False
        self._freeze_control = False
        self._control_paused = Event()
        self._estimator = LinearEstimator(
            self.config.encoder_mm_per_count,
            self.config.max_encoder_speed_mm_s,
            self.config.feedback_max_age_s,
            self.config.encoder_count_modulus,
        )
        self._pid = PositionController(
            self.config.kp,
            self.config.ki,
            self.config.kd,
            self.config.kaw,
            self.config.integral_limit_mm_s,
            self.config.feedback_max_age_s,
        )
        self._quantizer = PulseQuantizer(
            self.config.pulses_per_motor_rev, self.config.catheter_per_motor_ratio
        )
        self._jog_velocity = JogVelocityController(
            self.config.jog_velocity_kp,
            self.config.jog_velocity_ki,
            self.config.jog_velocity_kaw,
            self.config.jog_integral_limit_mm_s,
            self.config.jog_velocity_filter_s,
            self.config.jog_dt_max_s,
        )
        self._jog_rotation = JogRotationPlanner(self.config, self._quantizer)
        self._quality_tracker = QualityTracker(
            backend=self.config.backend, max_age_s=self.config.manual_input_max_age_s
        )
        self._input_deadline = 0
        self._jog_updated_ns = 0
        self._jog_feedback_ns = 0
        self._jog_feedback_seq = 0
        self._jog_last_desired = 0.0
        self._jog_feedback_retarget = False
        self._history, self._fingerprints, self._highwater = OrderedDict(), {}, {}
        self._revoked_generations = OrderedDict()
        self._reservation = None
        self._reservation_deadline = 0
        self._motion = None
        self._motion_started = 0
        self._linear_trajectory = self._rotation_trajectory = None
        self._pending_segment = None
        self._confirmed_pulses = self._reserved_pulses = 0
        self._rotation_offset = 0.0
        self._rotation_valid = self._rotation_referenced = False
        self._reference_id = ""
        self._output_seq = 0
        self._last_feedback_seq = 0
        self._last_feedback_ns = 0
        self._last_output_ns = 0
        self._last_velocity = 0.0
        self._settled_since = None
        self._stopping = None
        self._hold = None
        self._jog = None
        self._jog_deadline = 0
        self._terminal_pending = set()
        self._last_rotation_velocity = 0.0
        self._last_rotation_plan_ns = 0
        self._pulse_segments = {}
        self._diagnostics = {}
        self._dropped_records = 0

    def start(self) -> OperationResult:
        """Start exclusive I/O/control owners and enter DISARMED on success.

        Host lifecycle use only: readiness waits are bounded, while actual
        transactions run in I/O owners with device budgets. Initialization
        failure returns its code and FAULT/UNKNOWN; it never arms or sets zero
        references. Use a fresh runtime instance after close/failed startup.
        """
        if self._started:
            return OperationResult(True)
        try:
            if self._devices is None:
                if self.config.backend == "simulation":
                    from gca.simulation.devices import simulated_devices

                    self._devices = simulated_devices(self.config, self.clock)
                else:
                    from gca.devices.factory import hardware_devices

                    self._devices = hardware_devices(self.config, self.clock)
            self.servo, self.encoder, self.pulse = self._devices[:3]
            self.plant = self._devices[3] if len(self._devices) > 3 else None
            for target, name in (
                (self._servo_owner, "gca-servo-io"),
                (self._encoder_owner, "gca-encoder-io"),
                (self._pulse_owner, "gca-pulse-io"),
            ):
                thread = Thread(target=target, name=name, daemon=True)
                thread.start()
                self._threads.append(thread)
            ready_results = [event.wait(2.0) for event in self._ready]
            ready = all(ready_results)
            if not ready or self._emergency:
                raise DeviceError(self._emergency or "INITIALIZATION_TIMEOUT")
            self._started = True
            self._state = replace(self._state, configured=True, state="DISARMED")
            thread = Thread(target=self._control_owner, name="gca-control", daemon=True)
            thread.start()
            self._threads.append(thread)
            return OperationResult(True)
        except Exception as exc:
            self._emergency = getattr(exc, "code", "INITIALIZATION_FAILED")
            self._state = replace(
                self._state,
                state="FAULT",
                error_code=self._emergency,
                message=str(exc),
                stop_progress="UNKNOWN",
            )
            self._shutdown.set()
            for thread in self._threads:
                thread.join(0.2)
            return OperationResult(False, self._emergency, str(exc))

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
            done.set()  # A timed-out admission request must never execute later.
            return OperationResult(False, "CONTROL_REQUEST_TIMEOUT")
        return holder[0]

    def activate(self, active: bool = True) -> OperationResult:
        """Queue lifecycle availability; True preserves DISARMED until ARM.

        False revokes motion through local stopping. This short adapter-thread
        call returns request status; inspect snapshot for physical stop result.
        """
        return self._invoke("active", bool(active))

    set_active = activate

    @property
    def stop_serial(self) -> int:
        """Monotonic stop transaction fence; callers capture before ARM awaits."""
        return self._stop_serial

    def arm(self, lease: Lease, expected_stop_serial: int | None = None) -> OperationResult:
        """Grant a verified executor lease in seconds while active/DISARMED.

        The host authenticates Lease before this call. The control owner stores
        its local deadline/token and rejects revoked generations or wrong state;
        actual device enable remains exclusively in the I/O owner.
        """
        expected = self.stop_serial if expected_stop_serial is None else expected_stop_serial
        return self._invoke("arm", (lease, expected))

    def renew(self, lease: Lease) -> OperationResult:
        """Queue a same-session/token/generation renewal with increasing seq.

        ttl_s is relative seconds converted once to local monotonic ns. Expired
        or revoked permission cannot be revived; refusal returns STALE_LEASE.
        This method performs no device I/O and does not create a new motion.
        """
        return self._invoke("renew", lease)

    def authorize(self, lease: Lease) -> OperationResult:
        """Queue initial ARM or current-lease renewal; same conditions as above."""
        return self.renew(lease) if self._lease else self.arm(lease)

    def reserve(self, command: MotionCommand) -> OperationResult:
        """Atomically reserve one validated goal slot without any movement.

        Targets/limits are mm and degrees with seconds/ns deadlines. Requires
        active ARM, matching token/reference and selected-axis capabilities.
        Refusal preserves both axes; adapter threads receive explicit argument,
        permission, expiry, duplicate/conflict or busy status codes.
        """
        return self._invoke("reserve", command)

    def release_reservation(self, command_id: str) -> OperationResult:
        """Queue release of an unstarted slot; no I/O or sequence rollback."""
        return self._invoke("release", command_id)

    def start_motion(self, command: MotionCommand) -> OperationResult:
        """Queue execution of the identical reserved goal, rechecking expiry.

        Absolute/relative mm/degree targets resolve once in the control owner.
        Returns admission status promptly; completion/cancellation is delivered
        separately by drain_events and remains pending until commit_terminal.
        """
        return self._invoke("motion", command)

    def stop(
        self,
        reason: str = "STOP",
        command_id: str = "",
        disarm: bool = False,
        *,
        context: CommandContext | None = None,
    ) -> OperationResult:
        """Immediately revoke output and latch a bounded stop outside FIFO.

        Callable from adapter threads even with a full ordinary command queue.
        disarm upgrades a pending stop to permission withdrawal. Success means
        STOP_ACCEPTED only; snapshot.stop_progress/evidence later establishes
        CONFIRMED or UNKNOWN. No state lock or device call occurs here.
        """
        # This independent flag immediately revokes motion even if control is
        # paused; normal queued processing determines terminal result/evidence.
        if not self._started or self._shutdown.is_set():
            return OperationResult(False, "NOT_RUNNING")
        active = self._motion or self._hold
        if command_id and active and active.context.command_id != command_id:
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
        self._latch_stop(reason, command_id, disarm, context)
        return OperationResult(True, "STOP_ACCEPTED")

    def _latch_stop(self, reason, command_id="", disarm=False, context=None):
        self._permit = False
        self._pulse_stop.set()
        self._velocity_demand.put(None)
        # An independent latch cannot be crowded out by ordinary commands.
        with self._stop_lock:
            if context is not None:
                key = (context.session_id, context.generation)
                self._stop_fences[key] = max(context.sequence, self._stop_fences.get(key, -1))
                while len(self._stop_fences) > self.config.history_capacity:
                    self._stop_fences.popitem(last=False)
            if self._stop_request:
                disarm = disarm or self._stop_request[2]
            self._stop_serial += 1
            self._stop_request = (reason, command_id, disarm, self._stop_serial)
        self._wake.set()

    def disarm(self) -> OperationResult:
        """Latch stop plus permission withdrawal; inspect stop evidence later."""
        return self.stop("DISARM", disarm=True)

    def reset(self) -> OperationResult:
        """Queue explicit fault reset to DISARMED without replay or auto ARM.

        Requires confirmed stopping, no active/reserved goal and live owners.
        UNKNOWN output or dead I/O owners refuses reset; reconfigure hardware
        lifecycle as directed by the returned stable error code.
        """
        return self._invoke("reset")

    reset_fault = reset

    def set_reference(
        self, axis_mask: int = 3, linear_mm: float = 0.0, rotation_deg: float = 0.0
    ) -> OperationResult:
        """Queue software reference assignment in mm/degrees while DISARMED.

        Selected-axis valid feedback and confirmed stationary output are
        required. This never writes a hardware zero or moves toward an origin;
        successful reference_id invalidates old goals. Failure leaves references
        unchanged; the control owner performs the memory-only transformation.
        """
        return self._invoke("reference", (axis_mask, linear_mm, rotation_deg))

    def jog(self, command: JogCommand) -> OperationResult:
        """Queue latest authorized mm/s and deg/s demand with short TTL seconds.

        Requires fresh deadman input, valid selected references and no stopping
        or point-to-point task. The fixed receiver ns deadline caps queueing and
        duration. Release/expiry stops; old sequences cannot extend old output.
        """
        return self._invoke("jog", command)

    def configure(self, config: RuntimeConfig | dict[str, object]) -> OperationResult:
        """Queue a validated full configuration transaction while DISARMED.

        RuntimeConfig documents all units. Only explicit PID/revision fields
        can be applied live here; structural/device changes return
        RECONFIGURE_REQUIRED without falsely advancing applied revision.
        Validation does not perform hardware I/O in the caller thread.
        """
        try:
            if isinstance(config, dict):
                config = RuntimeConfig.from_dict(config)
        except (ValueError, TypeError) as exc:
            return OperationResult(False, "INVALID_CONFIGURATION", str(exc))
        return self._invoke("configure", config)

    def inject_fault(self, name: str, value: bool | float | int = True) -> OperationResult:
        """Inject a named simulation fault, never a hardware command.

        Fractions/displacements follow the named plant field's units; pause
        uses bool and a bounded acknowledgement. Unknown names/non-simulation
        backends refuse. Device I/O owners remain independent during pause.
        """
        if self.config.backend != "simulation":
            return OperationResult(False, "SIMULATION_ONLY")
        if name == "control_freeze":
            self._freeze_control = bool(value)
            self._wake.set()
            if value and not self._control_paused.wait(0.5):
                return OperationResult(False, "CONTROL_PAUSE_TIMEOUT")
            return OperationResult(True)
        return self._invoke("fault", (name, value))

    def snapshot(self) -> RuntimeState:
        """Return immutable current state with ages recomputed in local seconds.

        Safe for telemetry threads; no I/O or controller advancement occurs.
        Position uses mm, rotation degrees and raw sample times monotonic ns;
        validity/evidence must be checked instead of trusting stale numbers.
        """
        with self._lock:
            state = self._state
        now = self.clock()
        age = max(0.0, (now - state.sample_received_ns) / 1e9) if state.sample_received_ns else 1e9
        result = replace(
            state,
            sample_age_s=age,
            stop_serial=self.stop_serial,
            lease_remaining_s=max(0.0, (self._lease_deadline - now) / 1e9),
            linear_valid=state.linear_valid and age <= self.config.feedback_max_age_s,
        )
        if self._handled_stop_serial < self._stop_serial:
            result = replace(result, stop_progress="REQUESTED", requested_velocity_mm_s=0.0)
        if self._count_evidence_lost.is_set():
            result = replace(result, rotation_estimate_valid=False, rotation_reference_valid=False)
        return result

    def drain_events(self) -> list[MotionEvent]:
        """Atomically consume candidate results for one ROS bridge consumer.

        This short-lock operation does not commit Action state. The bridge
        resolves cancel/success ordering, then calls commit_terminal exactly
        once; callers must not discard execution evidence as ordinary telemetry.
        """
        with self._lock:
            items = list(self._results)
            self._results.clear()
            return items

    def commit_terminal(self, command_id: str, outcome: str) -> OperationResult:
        """Queue bridge acknowledgement of one submitted Action terminal state.

        outcome is SUCCEEDED/CANCELED/FAILED. The control owner clears only its
        matching goal context; an explicit HOLD context retains its own lease.
        This method produces no output and must follow bridge result submission.
        """
        return self._invoke("commit", (command_id, outcome))

    def get_command_status(self, command_id: str) -> OperationResult:
        """Read bounded command history; code carries status or UNKNOWN_COMMAND.

        Safe for adapter threads with a short memory lock. A cached result is
        observational and never grants permission or restarts a relative goal.
        """
        with self._lock:
            status = self._history.get(command_id)
        return OperationResult(bool(status), status or "UNKNOWN_COMMAND")

    command_status = get_command_status

    def simulation_evidence(self) -> dict[str, object]:
        """Observe accepted simulated I/O independently from the control writer.

        Never advances the plant, polls a device, or takes a control/STOP lock.
        The plant snapshot is internally consistent; mutable ingress metadata
        is bracketed and marked unavailable if it changes during observation.
        This is simulation truth, never evidence of physical hardware feedback.
        """
        if self.config.backend != "simulation" or getattr(self, "plant", None) is None:
            return {"available": False, "evidence_kind": "SIMULATION_ONLY"}
        result = {
            "available": False,
            "sampled_ns": self.clock(),
            "evidence_kind": "SIMULATED_ACCEPTED_IO_AND_PLANT",
        }
        before = (
            self._output_generation,
            self._output_seq,
            self._input_deadline,
            self._jog_deadline,
            self.stop_serial,
        )
        if not self.plant.lock.acquire(blocking=False):
            return dict(result, reason="PLANT_SNAPSHOT_BUSY")
        try:
            plant = self.plant.snapshot()
            plant_updated = self.plant._now
        finally:
            self.plant.lock.release()
        demand = self._velocity_demand.get()
        active = self.pulse._active  # SimPulseEngine immutable finite-segment reference.
        result.update(
            plant_position_mm=plant.position_mm,
            plant_velocity_mm_s=plant.velocity_mm_s,
            plant_angle_deg=plant.mechanical_angle_deg,
            accepted_velocity_mm_s=plant.accepted_velocity_mm_s,
            plant_updated_ns=plant_updated,
            servo_enabled=bool(self.servo.enabled),
            pulse_idle=active is None,
            pulse_active_expires_ns=active.expires_ns if active is not None else 0,
            demand_expires_ns=demand.expires_ns if demand is not None else 0,
            demand_seq=demand.seq if demand is not None else 0,
            control_state_seq=self._state.state_seq,
            control_paused=self._control_paused.is_set(),
            output_generation=before[0],
            output_seq=before[1],
            input_valid_until_ns=before[2],
            jog_valid_until_ns=before[3],
            stop_serial=before[4],
        )
        after = (
            self._output_generation,
            self._output_seq,
            self._input_deadline,
            self._jog_deadline,
            self.stop_serial,
        )
        result["available"] = before == after
        if not result["available"]:
            result["reason"] = "INGRESS_CHANGED_DURING_SNAPSHOT"
        return result

    def records(self) -> list[dict[str, object]]:
        """Copy bounded structured control records without disk I/O.

        Fields include mm/deg outputs, sample/output dt seconds and monotonic ns
        times. Telemetry/log consumers may read concurrently; eviction count is
        exposed in RuntimeState. Returned records are for observation, not replay.
        """
        with self._lock:
            return list(self._records)

    def close(self, timeout_s: float = 2.0) -> OperationResult:
        """Stop/disarm, then join owners within host wall-time seconds.

        Host lifecycle use only; no ROS Future/executor callback is awaited.
        Failed stopping, expired joins or device cleanup returns a false result
        and stable error code. Thread exit or zero-write acknowledgement alone
        does not constitute physical stopping evidence.
        """
        if not self._started:
            self._shutdown.set()
            return OperationResult(True)
        self._freeze_control = False
        self.stop("SHUTDOWN", disarm=True)
        deadline = monotonic_ns() + int(timeout_s * 1e9)
        while self.snapshot().stop_progress not in ("CONFIRMED", "UNKNOWN"):
            if monotonic_ns() >= deadline:
                break
            self._wake.wait(0.01)
            self._wake.clear()
        confirmed = self.snapshot().stop_progress == "CONFIRMED"
        self._shutdown.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(max(0.0, (deadline - monotonic_ns()) / 1e9))
        self._started = False
        if any(thread.is_alive() for thread in self._threads):
            return OperationResult(False, "SHUTDOWN_TIMEOUT")
        if self._cleanup_failed.is_set():
            self._state = replace(
                self._state,
                state="FAULT",
                stop_progress="UNKNOWN",
                error_code="DEVICE_CLEANUP_FAILED",
            )
            return OperationResult(False, "DEVICE_CLEANUP_FAILED")
        return OperationResult(confirmed, "OK" if confirmed else "STOP_UNKNOWN")

    def _control_owner(self):
        while not self._shutdown.is_set():
            if not self._freeze_control:
                self._control_paused.clear()
                try:
                    self._tick()
                except Exception as exc:
                    self._state = replace(self._state, message=str(exc))
                    self._begin_stop("CONTROL_EXCEPTION", fault=True)
            else:
                self._control_paused.set()
            self._wake.wait(self.config.control_period_s)
            self._wake.clear()

    def _tick(self):
        now = self.clock()
        if self._count_evidence_lost.is_set():
            self._rotation_valid = self._rotation_referenced = False
        with self._stop_lock:
            stop_request = self._stop_request
            self._stop_request = None
        if stop_request:
            reason, _, disarm, serial = stop_request
            self._begin_stop(reason, disarm=disarm, serial_already_set=True)
            self._handled_stop_serial = serial
        try:
            while True:
                request = self._commands.get_nowait()
                kind, value, done, holder = request[:4]
                serial = request[4] if len(request) > 4 else self.stop_serial
                if done.is_set():
                    continue
                try:
                    if kind in ("arm", "jog", "motion", "reserve") and serial != self.stop_serial:
                        result = OperationResult(False, "STOP_SUPERSEDED")
                    else:
                        result = self._handle(kind, value, self.clock())
                except Exception as exc:
                    result = OperationResult(
                        False, getattr(exc, "code", "INVALID_ARGUMENT"), str(exc)
                    )
                holder.append(result)
                done.set()
        except Empty:
            pass  # No pending local request; continue the same control tick.
        now = self.clock()
        self._consume_io(now)
        control_context = self._hold or self._motion or self._jog
        selected_linear = bool(control_context and control_context.axis_mask & 1)
        sample = self._sample.get()
        if sample is not None:
            self._diagnostics["encoder_raw_count"] = sample.raw_count
            previous = self._estimator.state
            estimate = self._estimator.update(sample)
            if (
                estimate.error
                and estimate.error != previous.error
                and self._state.armed
                and selected_linear
            ):
                self._begin_stop(estimate.error, fault=True)
        estimate = self._estimator.state
        fresh = estimate.valid and now - estimate.received_ns <= int(
            self.config.feedback_max_age_s * 1e9
        )
        if self._emergency and self._stopping is None and self._state.state != "FAULT":
            self._begin_stop(self._emergency, fault=True)
        if self._state.armed and now >= self._lease_deadline and self._stopping is None:
            self._begin_stop("LEASE_EXPIRED", disarm=True)
        running_motion = (
            self._motion and self._motion.context.command_id not in self._terminal_pending
        )
        if (
            selected_linear
            and (running_motion or self._hold or self._jog)
            and not fresh
            and self._stopping is None
        ):
            self._begin_stop("FEEDBACK_STALE", fault=True)
        if self._reservation and now >= self._reservation_deadline:
            self._history[self._reservation.context.command_id] = "RESERVATION_EXPIRED"
            self._reservation = None
        if self._stopping is not None:
            self._advance_stop(now)
        elif self._motion is not None or self._hold is not None or self._jog is not None:
            self._advance_motion(now, estimate)
        rotation = self._rotation_offset + self._quantizer.angle(self._confirmed_pulses)
        with self._lock:
            self._state = replace(
                self._state,
                position_mm=estimate.position_mm,
                velocity_mm_s=estimate.velocity_mm_s,
                linear_valid=fresh,
                velocity_valid=estimate.velocity_valid,
                linear_referenced=estimate.referenced,
                sample_seq=estimate.seq,
                sample_received_ns=estimate.received_ns,
                # Persist ages at this control record's monotonic timestamp.
                # snapshot() recomputes them for ROS readers, but raw records
                # must not retain the dataclass defaults during a stale fault.
                sample_age_s=max(0.0, (now - estimate.received_ns) / 1e9)
                if estimate.received_ns
                else 1e9,
                lease_remaining_s=max(0.0, (self._lease_deadline - now) / 1e9),
                device_epoch=estimate.epoch,
                reference_id=self._reference_id,
                rotation_deg=rotation,
                rotation_estimate_valid=self._rotation_valid,
                rotation_reference_valid=self._rotation_referenced,
                confirmed_pulses=self._confirmed_pulses,
                reserved_pulses=self._reserved_pulses,
                engine_idle=self._pulse_idle,
                generation=self._lease.generation if self._lease else 0,
                output_generation=self._output_generation,
                output_seq=self._output_seq,
                state_seq=self._state.state_seq + 1,
                stop_serial=self.stop_serial,
                input_valid_until_ns=self._input_deadline if self._jog else 0,
                jog_linear_velocity_mm_s=self._jog_velocity.target if self._jog else 0.0,
                jog_rotation_velocity_deg_s=self._jog_rotation.commanded_rate if self._jog else 0.0,
                rotation_planned_phase_deg=self._rotation_offset
                + self._jog_rotation.reference_phase / self._quantizer.pulses_per_deg,
                rotation_rate_deg_s=self._jog_rotation.wave_rate if self._jog else 0.0,
                jog_velocity_error_mm_s=self._jog_velocity.error if self._jog else 0.0,
                jog_velocity_integral_mm_s=self._jog_velocity.integral if self._jog else 0.0,
            )
            if len(self._records) == self._records.maxlen:
                self._dropped_records += 1
            self._state = replace(self._state, dropped_records=self._dropped_records)
            self._records.append({"time_ns": now, **asdict(self._state), **self._diagnostics})

    def _consume_io(self, now):
        try:
            while True:
                kind, value = self._io_events.get_nowait()
                if kind == "applied":
                    seq, generation, velocity = value
                    if generation == self._output_generation:
                        self._pid.applied(seq, generation, velocity)
                        if self._jog:
                            self._jog_velocity.applied(seq, generation, velocity)
                        self._diagnostics.update(
                            receipt_output_seq=seq,
                            receipt_output_generation=generation,
                            receipt_velocity_mm_s=velocity,
                        )
                        self._state = replace(self._state, applied_velocity_mm_s=velocity)
                elif kind == "zero_applied":
                    if value == self._output_generation or (
                        not self._permit and value < self._output_generation
                    ):
                        self._state = replace(self._state, applied_velocity_mm_s=0.0)
                        self._diagnostics.update(
                            receipt_output_generation=value,
                            receipt_velocity_mm_s=0.0,
                            receipt_kind="ZERO_SETPOINT_ACCEPTED",
                        )
                elif kind == "pulse_accepted":
                    if (
                        value.segment_id in self._pulse_segments
                        and value.generation == self._output_generation
                        and value.segment_id == self._pending_segment
                    ):
                        self._reserved_pulses = value.signed_pulses
                elif kind == "pulse_complete":
                    # A late completion records physical history but cannot
                    # schedule a successor from a revoked motion generation.
                    segment = self._pulse_segments.get(value.segment_id)
                    if segment is None:
                        continue  # Duplicate/unknown completion cannot change state.
                    if (
                        value.device_epoch != segment.device_epoch
                        or value.generation != segment.generation
                        or value.signed_pulses != segment.signed_pulses
                    ):
                        self._begin_stop("PULSE_COMPLETION_MISMATCH", fault=True)
                        continue
                    self._pulse_segments.pop(value.segment_id)
                    self._confirmed_pulses += value.signed_pulses
                    if self._pending_segment == value.segment_id:
                        self._pending_segment = None
                        self._reserved_pulses = 0
                        self._pulse_demand.put(None)
                elif kind == "pulse_rejected":
                    self._pulse_segments.pop(value.segment_id, None)
                    if self._pending_segment == value.segment_id:
                        self._pending_segment = None
                        self._reserved_pulses = 0
                        self._pulse_demand.put(None)
                elif kind == "pulse_aborted":
                    segment_id, receipt = value
                    segment = self._pulse_segments.pop(segment_id, None)
                    if segment is not None:
                        if receipt.confirmed_pulses is None:
                            self._rotation_valid = self._rotation_referenced = False
                        elif abs(receipt.confirmed_pulses) <= abs(segment.signed_pulses):
                            self._confirmed_pulses += receipt.confirmed_pulses
                        else:
                            self._rotation_valid = self._rotation_referenced = False
                        if self._pending_segment == segment_id:
                            self._pending_segment = None
                            self._reserved_pulses = 0
                elif kind == "jog_expired":
                    self._begin_stop("JOG_EXPIRED")
                elif kind == "fault":
                    self._begin_stop(value, fault=True)
                elif kind == "stop_unknown":
                    self._emergency = "OUTPUT_STATE_UNKNOWN"
                    if self._stopping:
                        self._stopping["unknown"] = True
        except Empty:
            return

    def _validate_lease(self, lease, now, renewal=False):
        if not isinstance(lease, Lease) or not lease.session_id or len(lease.session_id) > 128:
            return OperationResult(False, "INVALID_SESSION")
        if (
            type(lease.generation) is not int
            or type(lease.sequence) is not int
            or min(lease.generation, lease.sequence) < 0
            or type(lease.ttl_s) not in (int, float)
            or not isfinite(lease.ttl_s)
            or not 0 < lease.ttl_s <= self.config.lease_max_s
        ):
            return OperationResult(False, "INVALID_LEASE")
        if renewal:
            if (
                self._lease is None
                or now >= self._lease_deadline
                or lease.session_id != self._lease.session_id
                or lease.generation != self._lease.generation
                or lease.sequence <= self._lease.sequence
                or lease.token != self._lease.token
            ):
                return OperationResult(False, "STALE_LEASE")
        elif lease.generation <= self._revoked_generations.get(lease.session_id, -1):
            return OperationResult(False, "REVOKED_GENERATION")
        return OperationResult(True)

    def _validate_command(self, command, now, check_sequence=True):
        if not isinstance(command, MotionCommand):
            return OperationResult(False, "INVALID_ARGUMENT")
        if type(command.admission_deadline_ns) is not int or command.admission_deadline_ns < 0:
            return OperationResult(False, "INVALID_TIMEOUT")
        if command.admission_deadline_ns and now >= command.admission_deadline_ns:
            return OperationResult(False, "COMMAND_EXPIRED")
        context = command.context
        if context.sequence <= self._stop_fences.get((context.session_id, context.generation), -1):
            return OperationResult(False, "STOP_FENCED")
        if not all(
            isinstance(v, str) and 0 < len(v) <= 128
            for v in (context.session_id, context.task_id, context.command_id)
        ):
            return OperationResult(False, "INVALID_ID")
        if not self._state.armed or not self._active_lifecycle or self._lease is None:
            return OperationResult(False, "NOT_ARMED")
        if (
            now >= self._lease_deadline
            or context.session_id != self._lease.session_id
            or context.generation != self._lease.generation
            or command.freshness_token != self._lease.token
        ):
            return OperationResult(False, "INVALID_PERMISSION")
        if type(command.axis_mask) is not int or command.axis_mask not in (1, 2, 3):
            return OperationResult(False, "INVALID_AXIS_MASK")
        if (
            type(command.mode) is not int
            or type(command.end_policy) is not int
            or command.mode not in (1, 2)
            or command.end_policy not in (1, 2)
        ):
            return OperationResult(False, "INVALID_MODE_OR_END_POLICY")
        if command.reference_id != self._reference_id:
            return OperationResult(False, "REFERENCE_MISMATCH")
        estimate = self._estimator.state
        if command.axis_mask & 1 and (not estimate.referenced or not estimate.valid):
            return OperationResult(False, "LINEAR_REFERENCE_INVALID")
        if command.axis_mask & 2 and not (self._rotation_valid and self._rotation_referenced):
            return OperationResult(False, "ROTATION_REFERENCE_INVALID")
        values = (
            command.linear_mm,
            command.rotation_deg,
            command.admission_timeout_s,
            command.execution_timeout_s,
            *asdict(command.limits).values(),
        )
        if not all(type(v) in (float, int) and isfinite(v) for v in values):
            return OperationResult(False, "NONFINITE_ARGUMENT")
        if not 0 < command.admission_timeout_s <= 5 or not 0 < command.execution_timeout_s <= 600:
            return OperationResult(False, "INVALID_TIMEOUT")
        limits = command.limits
        selected_limits = []
        if command.axis_mask & 1:
            selected_limits.extend(("max_linear_mm_s", "max_linear_mm_s2"))
        if command.axis_mask & 2:
            selected_limits.extend(("max_rotation_deg_s", "max_rotation_deg_s2"))
        for name in selected_limits:
            if not 0 < getattr(limits, name) <= getattr(self.config, name):
                return OperationResult(False, "LIMIT_EXCEEDED")
        linear, rotation = command.linear_mm, command.rotation_deg
        if command.mode == 2:
            linear += estimate.position_mm
            rotation += self._rotation_offset + self._quantizer.angle(self._confirmed_pulses)
        if (
            command.axis_mask & 1
            and not self.config.linear_min_mm <= linear <= self.config.linear_max_mm
        ):
            return OperationResult(False, "LINEAR_LIMIT")
        if (
            command.axis_mask & 2
            and not self.config.rotation_min_deg <= rotation <= self.config.rotation_max_deg
        ):
            return OperationResult(False, "ROTATION_LIMIT")
        if (
            type(context.sequence) is not int
            or type(context.generation) is not int
            or min(context.sequence, context.generation) < 0
        ):
            return OperationResult(False, "INVALID_SEQUENCE")
        key = (context.session_id, context.generation)
        if check_sequence and context.sequence <= self._highwater.get(key, -1):
            return OperationResult(False, "STALE_SEQUENCE")
        return OperationResult(True)

    def _handle(self, kind, value, now):
        if kind == "active":
            self._active_lifecycle = value
            self._state = replace(self._state, lifecycle_active=value)
            if not value and self._state.armed:
                self._begin_stop("DEACTIVATE", disarm=True)
            return OperationResult(True)
        if kind in ("arm", "renew"):
            expected = self.stop_serial
            if kind == "arm" and isinstance(value, tuple):
                value, expected = value
            if kind == "arm" and (type(expected) is not int or expected < 0):
                return OperationResult(False, "INVALID_ARGUMENT")
            if kind == "arm" and expected != self.stop_serial:
                return OperationResult(False, "STOP_SUPERSEDED")
            result = self._validate_lease(value, now, renewal=kind == "renew")
            if not result.accepted:
                return result
            if kind == "arm":
                if not self._active_lifecycle or self._state.state != "DISARMED" or self._emergency:
                    return OperationResult(False, "ARM_REJECTED")
                if self.config.backend == "hardware" and not self.config.hardware_verified:
                    return OperationResult(False, "HARDWARE_NOT_VERIFIED")
                value = replace(value, token=value.token or token_urlsafe(24))
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
                    error_code="",
                )
                self._armed_stop_serial = expected
            self._lease = value
            self._lease_deadline = now + int(value.ttl_s * 1e9)
            return OperationResult(True, token=value.token)
        if kind == "reference":
            mask, linear, rotation = value
            if (
                self._state.armed
                or self._state.state != "DISARMED"
                or self._emergency
                or self._motion
                or self._reservation
            ):
                return OperationResult(False, "REFERENCE_BUSY")
            if (
                type(mask) is not int
                or mask not in (1, 2, 3)
                or not all(type(v) in (int, float) and isfinite(v) for v in (linear, rotation))
            ):
                return OperationResult(False, "INVALID_REFERENCE")
            servo = self._servo_sample.get()
            if (
                servo is None
                or not servo.valid
                or now - servo.received_ns > int(self.config.feedback_max_age_s * 1e9)
                or abs(servo.velocity_mm_s) > self.config.velocity_tolerance_mm_s
                or not self._pulse_idle
            ):
                return OperationResult(False, "NOT_STATIONARY")
            if mask & 1:
                estimate = self._estimator.state
                if not estimate.valid or now - estimate.received_ns > int(
                    self.config.feedback_max_age_s * 1e9
                ):
                    return OperationResult(False, "FEEDBACK_STALE")
                self._estimator.set_reference(linear)
            if mask & 2:
                self._rotation_offset = rotation
                self._confirmed_pulses = 0
                self._rotation_valid = self._rotation_referenced = True
                self._count_evidence_lost.clear()
            self._reference_id = str(uuid4())
            return OperationResult(True, reference_id=self._reference_id)
        if kind == "reserve":
            command = value
            result = self._validate_command(command, now, check_sequence=False)
            if not result.accepted:
                return result
            # Authentication and receiver-owned freshness budgets describe this
            # delivery attempt, not a second physical motion. They are checked
            # above but excluded from logical-command idempotency comparison.
            fingerprint = repr(
                replace(
                    command, freshness_token="", admission_deadline_ns=0, admission_timeout_s=0.0
                )
            )
            cid = command.context.command_id
            if cid in self._fingerprints:
                code = (
                    "DUPLICATE_COMMAND" if self._fingerprints[cid] == fingerprint else "ID_CONFLICT"
                )
                return OperationResult(False, code)
            sequence_key = (command.context.session_id, command.context.generation)
            if command.context.sequence <= self._highwater.get(sequence_key, -1):
                return OperationResult(False, "STALE_SEQUENCE")
            if self._reservation or self._motion or self._hold or self._jog or self._stopping:
                return OperationResult(False, "CONTROL_BUSY")
            self._reservation = command
            self._reservation_deadline = min(
                self._lease_deadline,
                now + int(command.admission_timeout_s * 1e9),
                command.admission_deadline_ns or self._lease_deadline,
            )
            self._fingerprints[cid] = fingerprint
            self._history[cid] = "ACCEPTED"
            self._highwater[(command.context.session_id, command.context.generation)] = (
                command.context.sequence
            )
            self._trim_history()
            return OperationResult(True)
        if kind == "release":
            if self._reservation and self._reservation.context.command_id == value:
                self._history[value] = "RELEASED"
                self._reservation = None
            return OperationResult(True)
        if kind == "motion":
            command = value
            if (
                self._reservation is None
                or self._reservation != command
                or now >= self._reservation_deadline
            ):
                return OperationResult(False, "RESERVATION_INVALID")
            result = self._validate_command(command, now, check_sequence=False)
            if not result.accepted:
                self._reservation = None
                return result
            if command.mode == 2:
                command = replace(
                    command,
                    mode=1,
                    linear_mm=command.linear_mm + self._estimator.state.position_mm,
                    rotation_deg=command.rotation_deg
                    + self._rotation_offset
                    + self._quantizer.angle(self._confirmed_pulses),
                )
            self._motion, self._reservation, self._motion_started = command, None, now
            self._armed_stop_serial = self.stop_serial
            self._hold = self._jog = None
            self._pid.reset()
            self._output_generation += 1
            self._permit = True
            self._last_feedback_seq = 0
            self._last_feedback_ns = 0
            self._last_output_ns = now
            self._last_velocity = 0.0
            self._last_rotation_velocity = 0.0
            self._last_rotation_plan_ns = now
            self._settled_since = None
            self._linear_trajectory = Trapezoid(
                self._estimator.state.position_mm,
                command.linear_mm if command.axis_mask & 1 else self._estimator.state.position_mm,
                command.limits.max_linear_mm_s
                if command.axis_mask & 1
                else self.config.max_linear_mm_s,
                command.limits.max_linear_mm_s2
                if command.axis_mask & 1
                else self.config.max_linear_mm_s2,
                now,
            )
            rotation = self._rotation_offset + self._quantizer.angle(self._confirmed_pulses)
            quantized = (
                (
                    self._rotation_offset
                    + self._quantizer.angle(self._pulse_target(command.rotation_deg))
                )
                if command.axis_mask & 2
                else rotation
            )
            self._rotation_trajectory = Trapezoid(
                rotation,
                quantized,
                command.limits.max_rotation_deg_s
                if command.axis_mask & 2
                else self.config.max_rotation_deg_s,
                command.limits.max_rotation_deg_s2
                if command.axis_mask & 2
                else self.config.max_rotation_deg_s2,
                now,
            )
            self._history[command.context.command_id] = "EXECUTING"
            self._state = replace(
                self._state,
                state="EXECUTING",
                task_id=command.context.task_id,
                command_id=command.context.command_id,
                target_linear_mm=command.linear_mm,
                target_rotation_deg=quantized,
                stop_progress="CONFIRMED",
                stop_evidence="",
            )
            return OperationResult(True)
        if kind == "stop":
            reason, cid, disarm = value
            if cid and self._motion and self._motion.context.command_id != cid:
                return OperationResult(False, "COMMAND_MISMATCH")
            self._begin_stop(reason, disarm=disarm)
            return OperationResult(True)
        if kind == "reset":
            if self._stopping or self._state.stop_progress == "UNKNOWN":
                return OperationResult(False, "STOP_NOT_CONFIRMED")
            if (
                self._state.state not in ("FAULT", "DISARMED")
                or self._motion
                or self._hold
                or self._jog
                or self._reservation
            ):
                return OperationResult(False, "RESET_BUSY")
            if any(not thread.is_alive() for thread in self._threads[:3]):
                return OperationResult(False, "DEVICE_REINITIALIZE_REQUIRED")
            self._revoke_current_lease()
            self._emergency = ""
            self._lease = None
            self._lease_deadline = 0
            self._permit = False
            self._state = replace(self._state, state="DISARMED", armed=False, error_code="")
            return OperationResult(True)
        if kind == "commit":
            cid, outcome = value
            self._history[cid] = outcome
            self._terminal_pending.discard(cid)
            if self._motion and self._motion.context.command_id == cid:
                self._motion = None
            self._trim_history()
            return OperationResult(True)
        if kind == "jog":
            return self._handle_jog(value, now)
        if kind == "configure":
            if self._state.armed or self._motion or self._reservation or self._stopping:
                return OperationResult(False, "CONFIGURATION_BUSY")
            if not isinstance(value, RuntimeConfig):
                return OperationResult(False, "INVALID_CONFIGURATION")
            # Rewiring/recalibration requires a fresh lifecycle configure so no
            # owner can continue with only half the hardware configuration.
            hot_parameters = {"revision", "kp", "ki", "kd", "kaw", "integral_limit_mm_s"}
            structural = set(value.as_dict()) - hot_parameters
            if any(getattr(value, name) != getattr(self.config, name) for name in structural):
                return OperationResult(False, "RECONFIGURE_REQUIRED")
            self.config = value
            self._pid = PositionController(
                value.kp,
                value.ki,
                value.kd,
                value.kaw,
                value.integral_limit_mm_s,
                value.feedback_max_age_s,
            )
            self._state = replace(
                self._state,
                config_revision=value.revision,
                requested_config_revision=value.revision,
                applied_config_revision=value.revision,
            )
            return OperationResult(True)
        if kind == "fault":
            return self._inject(*value)
        return OperationResult(False, "UNKNOWN_OPERATION")

    def _trim_history(self):
        while len(self._history) > self.config.history_capacity:
            cid = next(iter(self._history))
            if self._motion and cid == self._motion.context.command_id:
                self._history.move_to_end(cid)
                continue
            self._history.pop(cid)
            self._fingerprints.pop(cid, None)

    def _revoke_current_lease(self):
        if self._lease:
            session = self._lease.session_id
            self._revoked_generations[session] = max(
                self._lease.generation, self._revoked_generations.get(session, -1)
            )
            self._revoked_generations.move_to_end(session)
            self._quality_tracker.clear_session(session, self._lease.generation)
            while len(self._revoked_generations) > self.config.history_capacity:
                self._revoked_generations.popitem(last=False)

    def _begin_stop(
        self, reason, disarm=False, fault=False, outcome="CANCELED", serial_already_set=False
    ):
        latched_state = self._state.state if self._state.state in ("FAULT", "ESTOP") else ""
        if latched_state:
            fault = True
            reason = self._state.error_code or reason
            if (
                self._stopping is None
                and self._state.stop_progress == "CONFIRMED"
                and not self._permit
                and self._pulse_idle
                and self._pending_segment is None
            ):
                # Repeated ordinary parking cannot clear a fault or discard
                # already established stopping evidence. Only RESET unlatches.
                if disarm:
                    self._revoke_current_lease()
                    self._lease_deadline = 0
                    self._state = replace(self._state, armed=False)
                return
        if self._stopping:
            if disarm:
                self._stopping["disarm"] = True
            if fault:
                self._stopping["latched_state"] = latched_state or "FAULT"
                self._stopping["fault"] = True
                self._stopping["outcome"] = "FAILED"
                self._stopping["reason"] = reason
                self._stopping["disarm"] = True
            return
        self._permit = False
        if not serial_already_set:
            with self._stop_lock:
                self._stop_serial += 1
                self._handled_stop_serial = self._stop_serial
        self._output_generation += 1
        self._velocity_demand.put(None)
        # Preserve the queued segment until its exclusive I/O owner rejects it
        # or settles an already in-flight submit/abort. Clearing this mailbox
        # here would discard the only reject-ACK path for a planned segment,
        # leaving _pending_segment permanently unsettled despite an idle engine.
        self._pulse_stop.set()
        self._reservation = None
        self._hold = self._jog = None
        self._input_deadline = 0
        self._jog_velocity.reset()
        self._jog_rotation.reset(self.clock(), self._confirmed_pulses)
        self._last_velocity = 0.0
        self._last_output_ns = self.clock()
        self._stopping = dict(
            start=self.clock(),
            reason=reason,
            deadline=self.clock() + int(self.config.stop_timeout_s * 1e9),
            # Parking an unarmed device must never create an ARM transition.
            disarm=disarm or fault or not self._state.armed,
            fault=fault,
            latched_state=latched_state,
            unknown=False,
            outcome="FAILED" if fault else outcome,
            settled=None,
        )
        self._state = replace(
            self._state,
            state=latched_state or ("FAULT" if fault else "STOPPING"),
            requested_velocity_mm_s=0.0,
            stop_progress="IN_PROGRESS",
            stop_evidence="",
            error_code=reason if fault else self._state.error_code,
        )

    def _advance_stop(self, now):
        stop = self._stopping
        servo = self._servo_sample.get()
        measured = bool(
            servo
            and servo.valid
            and servo.received_ns >= stop["start"]
            and now - servo.received_ns <= int(self.config.feedback_max_age_s * 1e9)
            and abs(servo.velocity_mm_s) <= self.config.velocity_tolerance_mm_s
        )
        if measured and self._pulse_idle and self._pending_segment is None:
            if stop["settled"] is None:
                stop["settled"] = now
        else:
            stop["settled"] = None
        confirmed = stop["settled"] is not None and now - stop["settled"] >= int(
            self.config.settle_s * 1e9
        )
        failed = stop["unknown"] or now >= stop["deadline"]
        if not confirmed and not failed:
            return
        self._stopping = None
        if stop["disarm"] or failed:
            self._revoke_current_lease()
            self._lease_deadline = 0
        state = stop.get("latched_state") or (
            "FAULT" if stop["fault"] or failed else ("DISARMED" if stop["disarm"] else "READY")
        )
        self._state = replace(
            self._state,
            state=state,
            armed=not (stop["disarm"] or failed),
            stop_progress="CONFIRMED" if confirmed else "UNKNOWN",
            stop_evidence="MEASURED_SERVO_ZERO_AND_PULSE_IDLE" if confirmed else "UNKNOWN",
            error_code=(stop["reason"] if stop["fault"] else "STOP_TIMEOUT")
            if failed or stop["fault"]
            else "",
        )
        if self._motion:
            outcome = stop["outcome"] if confirmed else "FAILED"
            self._emit(
                MotionEvent(
                    self._motion.context.command_id,
                    outcome,
                    stop["reason"] if confirmed else "STOP_TIMEOUT",
                    stop_confirmed=confirmed,
                )
            )

    def _advance_motion(self, now, estimate):
        if self._jog:
            if now >= self._jog_deadline:
                self._begin_stop("JOG_EXPIRED")
                return
            command = self._jog
            dt = (now - self._jog_updated_ns) / 1e9
            if dt <= 0:
                return
            desired = command.linear_velocity_mm_s if command.axis_mask & 1 else 0.0
            if (
                desired == 0
                or desired * self._last_velocity < 0
                or abs(desired) < abs(self._jog_last_desired)
            ):
                self._jog_feedback_retarget = True
            self._jog_last_desired = desired
            try:
                self._jog_velocity.shape(desired, self.config.max_linear_mm_s2, dt)
                self._jog_rotation.update(
                    command.rotation_velocity_deg_s if command.axis_mask & 2 else 0.0,
                    now,
                    self._rotation_offset + self._quantizer.angle(self._confirmed_pulses),
                )
            except ValueError:
                self._begin_stop("JOG_DT_INVALID", fault=True)
                return
            self._jog_updated_ns = now
            if (
                command.axis_mask & 1
                and estimate.seq != self._jog_feedback_seq
                and estimate.velocity_valid
            ):
                sample_dt = (estimate.received_ns - self._jog_feedback_ns) / 1e9
                self._jog_feedback_ns, self._jog_feedback_seq = estimate.received_ns, estimate.seq
                self._jog_feedback_retarget = False
                requested = self._jog_velocity.request(
                    estimate.velocity_mm_s, sample_dt, neutral=desired == 0
                )
                limited = self._output_velocity(requested, now, estimate, None)
                self._jog_velocity.commit(
                    self._output_seq,
                    self._output_generation,
                    requested,
                    limited,
                    sample_dt,
                    neutral=desired == 0,
                )
                self._diagnostics.update(
                    sample_dt_s=sample_dt,
                    jog_raw_mm_s=requested,
                    jog_filtered_velocity_mm_s=self._jog_velocity.filtered,
                )
            elif not command.axis_mask & 1:
                self._output_velocity(0, now, estimate, None)
            elif self._jog_feedback_retarget:
                # A neutral/reverse/reduced intent may not prolong an old
                # direction while waiting for the next valid measurement.
                # Existing slew/braking limits apply; no PI integration or
                # quantization receipt is replayed without a new sample.
                if abs(self._jog_velocity.target - self._last_velocity) > 1e-12:
                    self._output_velocity(self._jog_velocity.target, now, estimate, None)
                else:
                    self._refresh_jog_demand(now)
            else:
                self._refresh_jog_demand(now)
            if command.axis_mask & 2:
                self._plan_jog_pulse(now, command.context.command_id)
            return
        command = self._hold or self._motion
        if command is None:
            return
        if self._hold is None and command.context.command_id in self._terminal_pending:
            return
        if self._hold is None and now - self._motion_started > int(
            command.execution_timeout_s * 1e9
        ):
            self._begin_stop("EXECUTION_TIMEOUT", fault=True)
            return
        linear = self._linear_trajectory.at(now)
        rotation = self._rotation_trajectory.at(now + int(self.config.pulse_segment_s * 1e9))
        self._state = replace(
            self._state,
            reference_linear_mm=linear.position,
            reference_rotation_deg=rotation.position,
        )
        if command.axis_mask & 1:
            if estimate.seq != self._last_feedback_seq and estimate.velocity_valid:
                dt = (
                    (estimate.received_ns - self._last_feedback_ns) / 1e9
                    if self._last_feedback_ns
                    else self.config.encoder_period_s
                )
                self._last_feedback_seq, self._last_feedback_ns = estimate.seq, estimate.received_ns
                if not 0 < dt <= self.config.feedback_max_age_s:
                    self._begin_stop("SAMPLE_DT_INVALID", fault=True)
                    return
                demand = self._pid.request(
                    linear.position, linear.velocity, estimate.position_mm, estimate.velocity_mm_s
                )
                limited = self._output_velocity(demand.requested, now, estimate, command)
                self._diagnostics.update(
                    sample_dt_s=dt,
                    pid_raw_mm_s=demand.requested,
                    pid_p_mm_s=demand.proportional,
                    pid_i_mm_s=demand.integral,
                    pid_d_mm_s=demand.derivative,
                    limited_velocity_mm_s=limited,
                )
                self._pid.commit(
                    self._output_seq,
                    self._output_generation,
                    linear.position - estimate.position_mm,
                    demand.requested,
                    limited,
                    dt,
                )
        else:
            self._output_velocity(0.0, now, estimate, command)
        if command.axis_mask & 2 and self._hold is None:
            self._plan_pulse(rotation.position, now, command.context.command_id)
        final_pulses = self._pulse_target(command.rotation_deg)
        linear_done = not command.axis_mask & 1 or (
            linear.complete
            and estimate.velocity_valid
            and abs(command.linear_mm - estimate.position_mm) <= self.config.position_tolerance_mm
            and abs(estimate.velocity_mm_s) <= self.config.velocity_tolerance_mm_s
        )
        rotation_done = not command.axis_mask & 2 or (
            rotation.complete
            and self._confirmed_pulses == final_pulses
            and self._pending_segment is None
            and self._pulse_idle
        )
        if self._hold is not None:
            return
        if linear_done and rotation_done:
            if self._settled_since is None:
                self._settled_since = now
            if now - self._settled_since >= int(self.config.settle_s * 1e9):
                if command.end_policy == 2:
                    self._hold = command
                    self._output_generation += 1
                    self._pid.reset()
                    self._state = replace(self._state, state="HOLDING")
                    self._emit(
                        MotionEvent(
                            command.context.command_id,
                            "SUCCEEDED",
                            outputs_held=True,
                            stop_confirmed=False,
                        )
                    )
                else:
                    self._begin_stop("TARGET_REACHED", outcome="SUCCEEDED")
        else:
            self._settled_since = None

    def _refresh_jog_demand(self, now):
        """Refresh permission metadata while the control writer is progressing.

        Holding a still-valid measured sample is not new PI data. Preserve
        velocity, output sequence, feedback time and STOP serial so the I/O
        owner neither re-writes a setpoint nor settles quantization twice.
        A frozen control writer cannot call this method; no source deadline
        or receiver nonce deadline is restarted here.
        """
        demand = self._velocity_demand.get()
        if (
            demand is None
            or demand.generation != self._output_generation
            or demand.stop_serial != self.stop_serial
            or not self._permit
        ):
            return
        expiry = min(
            now + round(self.config.output_ttl_s * 1e9),
            self._jog_deadline,
            self._input_deadline,
            self._lease_deadline,
        )
        self._velocity_demand.put(replace(demand, expires_ns=expiry))

    def _output_velocity(self, requested, now, estimate, command):
        maximum = (
            command.limits.max_linear_mm_s
            if command and command.axis_mask & 1
            else self.config.max_linear_mm_s
        )
        acceleration = (
            command.limits.max_linear_mm_s2
            if command and command.axis_mask & 1
            else self.config.max_linear_mm_s2
        )
        dt = (
            max(1e-6, (now - self._last_output_ns) / 1e9)
            if self._last_output_ns
            else self.config.control_period_s
        )
        limited = slew(clamp(requested, -maximum, maximum), self._last_velocity, acceleration, dt)
        limited = boundary_velocity(
            limited,
            estimate.position_mm,
            self.config.linear_min_mm,
            self.config.linear_max_mm,
            acceleration,
            self.config.feedback_max_age_s + self.config.io_timeout_s,
        )
        self._last_output_ns, self._last_velocity = now, limited
        self._diagnostics["output_dt_s"] = dt
        self._output_seq += 1
        expiry = min(self._lease_deadline, now + int(self.config.output_ttl_s * 1e9))
        if self._jog:
            expiry = min(expiry, self._input_deadline, self._jog_deadline)
        self._velocity_demand.put(
            _Demand(
                limited,
                self._output_seq,
                self._output_generation,
                expiry,
                estimate.received_ns,
                self.stop_serial,
            )
        )
        self._state = replace(self._state, requested_velocity_mm_s=limited)
        return limited

    def _pulse_target(self, angle_deg):
        """Quantize once without placing an executable edge past a soft limit."""
        scale = self._quantizer.pulses_per_deg
        low = ceil((self.config.rotation_min_deg - self._rotation_offset) * scale)
        high = floor((self.config.rotation_max_deg - self._rotation_offset) * scale)
        return min(high, max(low, self._quantizer.pulses(angle_deg - self._rotation_offset)))

    def _plan_pulse(self, target_deg, now, command_id):
        if self._pending_segment is not None or not self._pulse_idle:
            return
        target = self._pulse_target(target_deg)
        pulses = target - self._confirmed_pulses
        if not pulses:
            return
        duration = self.config.pulse_segment_s
        command = self._motion or self._hold
        max_velocity = (
            command.limits.max_rotation_deg_s if command else self.config.max_rotation_deg_s
        )
        acceleration = (
            command.limits.max_rotation_deg_s2 if command else self.config.max_rotation_deg_s2
        )
        desired_velocity = self._quantizer.angle(pulses) / duration
        dt = max(duration, (now - self._last_rotation_plan_ns) / 1e9)
        velocity = slew(
            clamp(desired_velocity, -max_velocity, max_velocity),
            self._last_rotation_velocity,
            acceleration,
            dt,
        )
        # Quantization introduces at most half an edge per segment. Never let
        # scheduler lag turn accumulated trajectory error into unbounded rate.
        step = self._quantizer.pulses(velocity * duration)
        if not step:
            return
        if (step > 0) != (pulses > 0):
            # Reversal cannot jump immediately through the previous direction.
            self._last_rotation_velocity = velocity
            self._last_rotation_plan_ns = now
            return
        pulses = min(pulses, step) if pulses > 0 else max(pulses, step)
        expiry = min(self._lease_deadline, now + int(self.config.output_ttl_s * 1e9))
        if now + int(duration * 1e9) >= expiry:
            return
        segment = PulseSegment(
            str(uuid4()),
            pulses,
            duration,
            self._output_generation,
            self._pulse_epoch,
            command_id,
            expires_ns=expiry,
            stop_serial=self.stop_serial,
        )
        self._pending_segment = segment.segment_id
        self._pulse_segments[segment.segment_id] = segment
        self._last_rotation_velocity = self._quantizer.angle(pulses) / duration
        self._last_rotation_plan_ns = now
        self._pulse_demand.put((segment, expiry))

    def _handle_jog(self, command, now):
        ingress_stop_serial = self.stop_serial
        if (
            not isinstance(command, JogCommand)
            or self._motion
            or self._reservation
            or self._hold
            or self._stopping
            or self._terminal_pending
        ):
            return OperationResult(False, "CONTROL_BUSY")
        context = command.context
        if (
            not isinstance(context, CommandContext)
            or not all(
                isinstance(v, str) and 0 < len(v) <= 128
                for v in (context.session_id, context.task_id, context.command_id)
            )
            or any(type(v) is not int or v < 0 for v in (context.generation, context.sequence))
        ):
            return OperationResult(False, "INVALID_CONTEXT")
        if type(command.deadman) is not bool:
            return OperationResult(False, "INVALID_JOG")
        if context.sequence <= self._stop_fences.get((context.session_id, context.generation), -1):
            return OperationResult(False, "STOP_FENCED")
        if (
            not self._state.armed
            or not self._active_lifecycle
            or self._lease is None
            or now >= self._lease_deadline
            or context.session_id != self._lease.session_id
            or context.generation != self._lease.generation
            or command.freshness_token != self._lease.token
        ):
            return OperationResult(False, "INVALID_PERMISSION")
        if not command.deadman:
            self._latch_stop("DEADMAN_RELEASED", context=context)
            self._begin_stop("DEADMAN_RELEASED", serial_already_set=True)
            with self._stop_lock:
                self._stop_request = None
                self._handled_stop_serial = self._stop_serial
            return OperationResult(True)
        if type(command.admission_deadline_ns) is not int or command.admission_deadline_ns < 0:
            return OperationResult(False, "INVALID_TIMEOUT")
        if command.admission_deadline_ns and now >= command.admission_deadline_ns:
            return OperationResult(False, "COMMAND_EXPIRED")
        if (
            type(command.axis_mask) is not int
            or command.axis_mask not in (1, 2, 3)
            or type(command.ttl_s) not in (int, float)
            or not 0 < command.ttl_s <= 0.5
            or not all(
                type(v) in (int, float) and isfinite(v)
                for v in (command.linear_velocity_mm_s, command.rotation_velocity_deg_s)
            )
            or abs(command.linear_velocity_mm_s) > self.config.max_linear_mm_s
            or abs(command.rotation_velocity_deg_s) > self.config.max_rotation_deg_s
        ):
            return OperationResult(False, "INVALID_JOG")
        if command.axis_mask & 2 and not (self._rotation_valid and self._rotation_referenced):
            return OperationResult(False, "ROTATION_REFERENCE_INVALID")
        estimate = self._estimator.state
        if command.axis_mask & 1 and (
            not estimate.referenced
            or not estimate.valid
            or now - estimate.received_ns > int(self.config.feedback_max_age_s * 1e9)
        ):
            return OperationResult(False, "LINEAR_REFERENCE_INVALID")
        key = (context.session_id, context.generation)
        if context.sequence <= self._highwater.get(key, -1):
            return OperationResult(False, "STALE_SEQUENCE")
        try:
            input_deadline = self._quality_tracker.validate(
                command.quality, now, session_id=context.session_id, generation=context.generation
            )
        except ValueError as exc:
            return OperationResult(False, getattr(exc, "code", "INVALID_MANUAL_QUALITY"), str(exc))
        with self._stop_lock:
            if ingress_stop_serial != self.stop_serial or self._stop_request is not None:
                return OperationResult(False, "STOP_SUPERSEDED")
            self._highwater[key] = context.sequence
            if self._jog is None:
                self._armed_stop_serial = self.stop_serial
                self._output_generation += 1
                self._jog_velocity.reset(self._estimator.state.velocity_mm_s)
                self._jog_rotation.reset(now, self._confirmed_pulses)
                self._last_velocity = 0.0
                self._last_output_ns = now
                self._jog_updated_ns = now
                self._jog_feedback_ns = self._estimator.state.received_ns
                self._jog_feedback_seq = self._estimator.state.seq
                self._jog_last_desired = 0.0
                self._jog_feedback_retarget = False
            self._input_deadline = input_deadline
            self._jog = command
            self._jog_deadline = min(
                now + int(command.ttl_s * 1e9),
                self._lease_deadline,
                command.admission_deadline_ns or self._lease_deadline,
                input_deadline,
            )
            self._permit = True
            self._state = replace(self._state, state="EXECUTING", command_id=context.command_id)
            return OperationResult(True)

    def _plan_jog_pulse(self, now, command_id):
        if self._pending_segment is not None or not self._pulse_idle:
            return
        expiry = min(
            self._lease_deadline,
            self._jog_deadline,
            self._input_deadline,
            now + round(self.config.output_ttl_s * 1e9),
        )
        if now + round(self.config.pulse_segment_s * 1e9) > expiry:
            return
        profile = self._jog_rotation.plan(
            now, self._confirmed_pulses, angle_offset=self._rotation_offset
        )
        if profile is None:
            return
        segment = PulseSegment(
            str(uuid4()),
            profile.signed_pulses,
            profile.duration_s,
            self._output_generation,
            self._pulse_epoch,
            command_id,
            expiry,
            profile.edge_offsets_us,
            self.stop_serial,
        )
        self._pending_segment = segment.segment_id
        self._pulse_segments[segment.segment_id] = segment
        self._pulse_demand.put((segment, expiry))
        self._diagnostics.update(
            rotation_profile_start_deg_s=profile.start_rate_deg_s,
            rotation_profile_end_deg_s=profile.end_rate_deg_s,
            rotation_phase_start_pulses=profile.phase_start_pulses,
            rotation_phase_end_pulses=profile.phase_end_pulses,
        )

    def _inject(self, name, value):
        targets = {
            "encoder_disconnect": (self.encoder, "disconnected"),
            "encoder_freeze": (self.encoder, "frozen"),
            "encoder_invalid": (self.encoder, "invalid"),
            "encoder_jump": (self.encoder, "jump_mm"),
            "servo_write_error": (self.servo, "fail_write"),
            "servo_read_error": (self.servo, "fail_read"),
            "pulse_submit_error": (self.pulse, "fail_submit"),
            "pulse_unknown_abort": (self.pulse, "unknown_abort"),
        }
        if self.plant is not None:
            targets.update(
                {
                    "plant_stuck": (self.plant, "stuck"),
                    "plant_ignore_zero": (self.plant, "ignore_zero"),
                    "missed_steps": (self.plant, "missed_step_fraction"),
                    "slip": (self.plant, "slip"),
                }
            )
        if name not in targets:
            return OperationResult(False, "UNKNOWN_FAULT")
        target, attribute = targets[name]
        setattr(target, attribute, value)
        return OperationResult(True)

    def _emit(self, event):
        self._terminal_pending.add(event.command_id)
        with self._lock:
            if len(self._results) >= self.config.queue_capacity:
                self._emergency = "RESULT_QUEUE_OVERFLOW"
                self._permit = False
                self._pulse_stop.set()
                self._state = replace(self._state, state="FAULT", error_code=self._emergency)
                return
            self._results.append(event)
        if self.notify:
            try:
                self.notify()
            except Exception:
                self._emergency = "RESULT_NOTIFICATION_FAILED"
                self._permit = False
