"""Pure single-owner teleoperation gates and immutable input provenance.

No ROS, device writes, session claim or implicit ARM occurs here. The caller
serializes process/tick/config changes and delivers every STOP immediately.
"""

from __future__ import annotations

from gca.domain.manual import ManualInputQuality

from .config import read_evidence, validate_mapping_evidence, validate_qualification
from .mapping import map_input
from .model import InputError, InputIdentity, Intent
from .policy import DUAL_AXIS_POLICY


class TeleopPipeline:
    def __init__(self, config, *, policy=DUAL_AXIS_POLICY):
        self.config = config
        self.policy = policy
        self._validate_policy_profile(config)
        self._context = None
        self._epoch = ""
        self._evidence_epoch = ""
        self._evidence_error = ""
        self._qualification = ""
        self._frame = None
        self._mapped = None
        self._servo_stop_reason = None
        self._stop_epoch = 0
        self._load_records()
        self.reset("STARTUP")

    def _validate_policy_profile(self, profile):
        if self.policy.manual_hold_enabled and (
            profile.source != "joydev" or profile.mode != "event_age"
        ):
            raise ValueError("JOYDEV_HOLD_REQUIRES_JOYDEV_EVENT_AGE")

    def _hardware_input_allowed(self, frame, health):
        if self.policy.manual_hold_enabled:
            return bool(
                self.config.source == "joydev"
                and self.config.mode == "event_age"
                and not self._record_error
                and not self._evidence_error
                and frame is not None
                and not frame.synthetic
                and not frame.resync
                and frame.source_kind == "JOYDEV"
                and frame.identity is not None
                and health.identity is not None
                and health.source_id == frame.source_id
                and health.input_epoch == frame.input_epoch
                and frame.identity.fingerprint == self._record_identity_fingerprint
                and health.identity.fingerprint == frame.identity.fingerprint
            )
        return bool(
            self.config.source == "joydev"
            and self.config.mode == "qualified_state"
            and self._qualification
            and not self._evidence_error
            and frame is not None
            and not frame.synthetic
        )

    def _load_records(self):
        # Management-time IO AND complete raw replay. A full physical mapping
        # can contain hundreds of frames: doing this in process() would starve
        # the reader and invalidate its independent 100ms watchdog.
        self._records = {}
        self._record_error = ""
        self._record_identity_fingerprint = ""
        self._verified_qualification = ""
        for name in ("mapping_evidence", "qualification_file"):
            path = getattr(self.config, name)
            try:
                self._records[name] = read_evidence(path) if path else {}
            except (OSError, ValueError, TypeError):
                self._records[name] = {}
        if self.config.source == "scripted":
            return
        try:
            if not self.config.mapping.complete:
                raise ValueError("MAPPING_UNVERIFIED")
            claimed = dict(self._records["mapping_evidence"].get("identity", {}))
            for name in ("axis_codes", "button_codes"):
                claimed[name] = tuple(claimed.get(name, ()))
            identity = InputIdentity(**claimed)
            validate_mapping_evidence(
                self.config, identity, evidence=self._records["mapping_evidence"]
            )
            self._record_identity_fingerprint = identity.fingerprint
            if self.config.mode == "qualified_state":
                self._verified_qualification = validate_qualification(
                    self.config,
                    identity,
                    evidence=self._records["qualification_file"],
                    mapping_evidence=self._records["mapping_evidence"],
                )
        except (OSError, ValueError, TypeError, KeyError, AttributeError, IndexError) as exc:
            self._record_error = str(exc) or "INPUT_EVIDENCE_INVALID"

    def hardware_arm_error(self, health, now_ns):
        """Teleop-only ARM precondition; no implicit ARM and no file IO.

        Core hardware HIL gates remain independently required. Recheck at ARM
        ingress; any later input failure must use the session STOP fence.
        """
        frame = self._frame
        if not self._hardware_input_allowed(frame, health):
            return "HARDWARE_INPUT_NOT_QUALIFIED"
        if (
            not health.connected
            or not health.initialized
            or health.reason
            or frame is None
            or not frame.initialized
        ):
            return "INPUT_NOT_READY"
        if (
            health.input_epoch != self._epoch
            or health.source_id != frame.source_id
            or frame.identity is None
            or health.identity is None
            or health.identity.fingerprint != frame.identity.fingerprint
        ):
            return "INPUT_IDENTITY_MISMATCH"
        if (
            type(now_ns) is not int
            or type(health.reader_heartbeat_ns) is not int
            or health.reader_heartbeat_ns > now_ns
            or now_ns - health.reader_heartbeat_ns >= int(self.config.reader_timeout_s * 1e9)
        ):
            return "READER_STALE"
        return ""

    def hardware_arm_quality(self, health, now_ns):
        """Snapshot neutral input without extending its reader deadline.

        This is management proof, not a motion intent or an LT edge. Only a
        later reader observation can supply a newer deadline; calling this
        method again over the same frame does not renew it.
        """
        error = self.hardware_arm_error(health, now_ns)
        if error:
            raise InputError(error)
        frame, mapped = self._frame, self._mapped
        if frame.synthetic or frame.source_kind != "JOYDEV":
            raise InputError("HARDWARE_INPUT_NOT_QUALIFIED")
        if mapped is None or not mapped.centered or mapped.b or mapped.lt > self.config.lt_release:
            raise InputError("ARM_REQUIRES_RELEASE_AND_CENTER")
        held = self.policy.manual_hold_enabled
        # Neutral INIT is enough for a zero-target ARM handshake. It is never
        # an LT edge; moving still requires a later real trigger press.
        if not held and (frame.event_seq < 1 or frame.raw_event_ns < 1):
            raise InputError("ARM_INPUT_EVENT_REQUIRED")
        age_s = self.policy.input_max_age_s(self.config.input_max_age_s)
        deadline = frame.observed_ns + int(age_s * 1e9)
        if frame.observed_ns > now_ns or now_ns >= deadline:
            raise InputError("ARM_INPUT_EXPIRED")
        return ManualInputQuality(
            "JOYDEV_HELD_ARM" if held else "QUALIFIED_STATE_CONTRACT",
            frame.source_id,
            frame.input_epoch,
            frame.event_seq,
            frame.raw_event_ns,
            "JOYDEV_INIT_ANCHORED_U32_MS",
            frame.observation_seq,
            frame.observed_ns,
            deadline,
            frame.clock_domain_id,
            "UNKNOWN" if held else "QUALIFIED_STATE_CONTRACT",
            "" if held else self._qualification,
        )

    def reset(self, reason="RESET"):
        self.gate = "WAIT_NEUTRAL"
        # Reset the gate without inventing a physical LT release. A stopped
        # held trigger must remain held until a later real release is read.
        self._lt_pressed = getattr(self, "_lt_pressed", False)
        if self._mapped is not None:
            if self._mapped.lt >= self.config.lt_press:
                self._lt_pressed = True
            elif self._mapped.lt <= self.config.lt_release:
                self._lt_pressed = False
        self._neutral_observed = False
        self._press_edge = False
        self._pending_lt_epoch = None
        self._active = False
        self._event_ns = 0
        self._event_seq = 0
        self._stop_reason = reason
        self._stop_emitted = False
        if reason != "CONTEXT_CHANGED":
            self._servo_stop_reason = None

    def set_config(self, profile, *, stopped, disarmed):
        if stopped is not True or disarmed is not True:
            raise InputError("CONFIG_REQUIRES_STOPPED_DISARMED")
        self._validate_policy_profile(profile)
        self.config = profile
        self._load_records()
        self._frame = self._mapped = None
        self._context = None
        self._epoch = self._evidence_epoch = ""
        self.reset("PROFILE_CHANGED")

    def _stop(self, reason):
        changed = self._active or self._stop_reason != reason or not self._stop_emitted
        # READY -> STOPPING -> READY is an effect of the requested STOP, not a
        # new physical fault. Keep that fault's emission separate from gates.
        previous_servo_reason = self._servo_stop_reason
        released_neutral = (
            self.policy.is_servo
            and reason == "DEADMAN_RELEASED"
            and self._neutral_observed
            and not self._lt_pressed
            and self._mapped is not None
            and self._mapped.centered
            and not self._mapped.b
        )
        derived_gate = reason in ("CONTEXT_CHANGED", "NOT_ARMED_READY", "BASELINE_PENDING")
        if self.policy.is_servo and not derived_gate:
            changed = self._active or previous_servo_reason != reason
        self.reset(reason)
        if released_neutral:
            self._neutral_observed = True
        if self.policy.is_servo:
            self._servo_stop_reason = previous_servo_reason if derived_gate else reason
        self._stop_emitted = True
        return Intent(
            action="STOP" if changed else "NONE",
            reason=reason,
            gate=self.gate,
            lt_pressed=self._lt_pressed,
            b_pressed=bool(self._mapped and self._mapped.b),
            input_epoch=self._epoch,
            control_policy=self.policy.control_policy,
        )

    def _context_check(self, *, armed, ready, reference_id, session_key, hardware):
        context = (bool(armed), bool(ready), reference_id, session_key, bool(hardware))
        if self.policy.is_servo:
            # STOP progress changes permission, not the operator's identity.
            context = (bool(armed), reference_id, session_key, bool(hardware))
        if self._context != context:
            old = self._context
            self._context = context
            self.reset("CONTEXT_CHANGED")
            if self.policy.is_servo and (
                old is None or old[1:] != context[1:] or (not old[0] and context[0])
            ):
                # A new authority/ARM cycle must independently stop a still
                # held B; only a readiness transition preserves its episode.
                self._servo_stop_reason = None
            return True
        return False

    def bind_stop_epoch(self, epoch):
        """Bind our emitted STOP without losing its simultaneous neutral release."""
        if self.policy.is_servo:
            self._stop_epoch = epoch
            self._pending_lt_epoch = None

    def _sync_stop_epoch(self, epoch):
        if self.policy.is_servo and epoch > self._stop_epoch:
            # A STOP from another path cancels all earlier rearm evidence.
            previous_reason = self._servo_stop_reason
            self.reset("STOP_EPOCH_CHANGED")
            self._servo_stop_reason = previous_reason
            self._stop_epoch = epoch

    def _evidence(self, frame):
        key = (
            frame.input_epoch,
            frame.identity.fingerprint if frame.identity else "",
            self.config.fingerprint,
        )
        if key == self._evidence_epoch:
            return
        self._evidence_epoch = key
        self._qualification, self._evidence_error = "", ""
        try:
            if frame.identity is None:
                raise ValueError("INPUT_IDENTITY_REQUIRED")
            if (self.config.source == "scripted") != frame.synthetic:
                raise ValueError("INPUT_SOURCE_MISMATCH")
            if not self.config.mapping.complete:
                raise ValueError("MAPPING_UNVERIFIED")
            if not frame.synthetic:
                if self._record_error:
                    raise ValueError(self._record_error)
                if frame.identity.fingerprint != self._record_identity_fingerprint:
                    raise ValueError("MAPPING_EVIDENCE_MISMATCH")
                self._qualification = self._verified_qualification
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self._evidence_error = str(exc) or "INPUT_EVIDENCE_INVALID"

    def process(
        self,
        frame,
        health,
        now_ns,
        *,
        armed,
        ready=True,
        reference_id="",
        session_key="",
        hardware=False,
        stop_pending=False,
        stop_epoch=0,
    ):
        context_was_active = self._active
        changed_context = self._context_check(
            armed=armed,
            ready=ready,
            reference_id=reference_id,
            session_key=session_key,
            hardware=hardware,
        )
        if changed_context and self.policy.is_servo:
            self._stop_epoch = stop_epoch
        self._sync_stop_epoch(stop_epoch)
        if self.policy.is_servo and stop_epoch < self._stop_epoch:
            # gate() may have been sampled just before our synchronous STOP.
            ready, stop_pending = False, True
        numeric = (
            now_ns,
            frame.observed_ns,
            frame.raw_event_ns,
            frame.event_seq,
            frame.observation_seq,
            health.reader_heartbeat_ns,
        )
        if (
            any(type(n) is not int or n < 0 or n >= 2**64 for n in numeric)
            or not frame.input_epoch
            or not frame.source_id
            or frame.observation_seq < 1
            or frame.observed_ns > now_ns
            or frame.raw_event_ns > frame.observed_ns
            or frame.source_id != health.source_id
            or frame.input_epoch != health.input_epoch
        ):
            return self._stop("INPUT_FRAME_INVALID")
        if frame.input_epoch != self._epoch or frame.resync:
            was_active = self._active
            self._epoch = frame.input_epoch
            self._frame = self._mapped = None
            self.reset("INPUT_EPOCH_CHANGED")
            self._evidence(frame)
            if was_active or frame.resync:
                # Do not swallow INIT/reopen STOP while establishing baseline.
                self._frame = frame
                return self._stop("INPUT_RESYNC")
        self._evidence(frame)
        old = self._frame
        if old is not None and (
            frame.event_seq < old.event_seq
            or frame.observation_seq < old.observation_seq
            or frame.observed_ns < old.observed_ns
            or frame.raw_event_ns < old.raw_event_ns
        ):
            return self._stop("INPUT_FRAME_REPLAY")
        self._frame = frame
        if not frame.initialized:
            return self._stop("BASELINE_PENDING")
        try:
            mapped = self.policy.mapped(map_input(frame, self.config))
        except InputError as exc:
            return self._stop(exc.code)
        previous = self._mapped
        self._mapped = mapped
        # B's current held state is enough to inhibit, including startup INIT.
        if mapped.b:
            if self.policy.is_servo and previous is not None and not previous.b:
                self._servo_stop_reason = None
            return self._stop("B_STOP")
        before = self._lt_pressed
        if mapped.lt >= self.config.lt_press:
            self._lt_pressed = True
        elif mapped.lt <= self.config.lt_release:
            self._lt_pressed = False
        if self.policy.is_servo and self._pending_lt_epoch is not None and not self._lt_pressed:
            self._pending_lt_epoch = None
            self._neutral_observed = False
            self.gate = "WAIT_NEUTRAL"
        self._press_edge = (
            not before
            and self._lt_pressed
            and frame.event_seq > 0
            and self.config.mapping.lt.index in frame.changed_axes
        )
        if self.policy.manual_hold_enabled:
            self._press_edge = bool(
                self._press_edge
                and old is not None
                and frame.event_seq > old.event_seq
                and not frame.synthetic
                and frame.source_kind == "JOYDEV"
            )
        if not self._lt_pressed and mapped.centered:
            self._neutral_observed = True
        # Left-side event age is independent of right controls and cache polls.
        meaningful = previous is not None and (
            (
                not self.policy.is_servo
                and self.config.mapping.lx.index in frame.changed_axes
                and mapped.lx != previous.lx
            )
            or (self.config.mapping.ly.index in frame.changed_axes and mapped.ly != previous.ly)
            or (self.config.mapping.lt.index in frame.changed_axes and before != self._lt_pressed)
        )
        if meaningful and frame.event_seq > 0:
            self._event_ns, self._event_seq = frame.raw_event_ns, frame.event_seq
        if frame.synthetic and frame.event_seq > 0:
            self._event_ns, self._event_seq = frame.raw_event_ns, frame.event_seq
        if self._active and before and not self._lt_pressed:
            return self._stop("DEADMAN_RELEASED")
        if changed_context and context_was_active:
            return self._stop("CONTEXT_CHANGED")
        if changed_context:
            # Even when a newly armed frame is held, it cannot inherit the old
            # READY gate. The operator must release and center in this context.
            self.gate = "WAIT_NEUTRAL"
        return self._evaluate(
            health, now_ns, armed=armed, ready=ready, hardware=hardware, stop_pending=stop_pending
        )

    def tick(
        self,
        health,
        now_ns,
        *,
        armed,
        ready=True,
        reference_id="",
        session_key="",
        hardware=False,
        stop_pending=False,
        stop_epoch=0,
    ):
        changed = self._context_check(
            armed=armed,
            ready=ready,
            reference_id=reference_id,
            session_key=session_key,
            hardware=hardware,
        )
        if changed and self.policy.is_servo:
            self._stop_epoch = stop_epoch
        self._sync_stop_epoch(stop_epoch)
        if self.policy.is_servo and stop_epoch < self._stop_epoch:
            ready, stop_pending = False, True
        if changed:
            return self._stop("CONTEXT_CHANGED")
        return self._evaluate(
            health, now_ns, armed=armed, ready=ready, hardware=hardware, stop_pending=stop_pending
        )

    def _evaluate(self, health, now, *, armed, ready, hardware, stop_pending=False):
        frame, mapped = self._frame, self._mapped
        if not health.connected or health.reason not in ("", "BASELINE_PENDING"):
            return self._stop(health.reason or "INPUT_DISCONNECTED")
        if (
            not health.initialized
            or frame is None
            or not frame.initialized
            or mapped is None
            or health.input_epoch != self._epoch
        ):
            return self._stop("BASELINE_PENDING")
        if health.reader_heartbeat_ns > now or now - health.reader_heartbeat_ns >= int(
            self.config.reader_timeout_s * 1e9
        ):
            return self._stop("READER_STALE")
        if self._evidence_error:
            return self._stop(self._evidence_error)
        if (hardware or self.policy.manual_hold_enabled) and not self._hardware_input_allowed(
            frame, health
        ):
            return self._stop("HARDWARE_INPUT_NOT_QUALIFIED")
        if self.policy.is_servo and mapped.b:
            return self._stop("B_STOP")
        waiting_stop = self.policy.is_servo and stop_pending
        if not armed or (not ready and not waiting_stop):
            return self._stop("NOT_ARMED_READY")
        if self.gate == "WAIT_NEUTRAL":
            if (
                self.policy.is_servo
                and self._neutral_observed
                and self._lt_pressed
                and self._press_edge
            ):
                # The prior LT-release STOP frame itself may prove neutral;
                # no extra neutral poll is required before this new edge.
                self.gate = "READY"
            else:
                if self._neutral_observed and not self._lt_pressed and mapped.centered:
                    self.gate = "READY"
                    self._stop_reason, self._stop_emitted = "", False
                return Intent(
                    gate=self.gate,
                    reason="RELEASE_AND_CENTER",
                    input_epoch=self._epoch,
                    control_policy=self.policy.control_policy,
                )
        if waiting_stop:
            if self._neutral_observed and self._press_edge and self._lt_pressed:
                self._pending_lt_epoch = self._stop_epoch
            if self._pending_lt_epoch is not None:
                origin = (
                    frame.observed_ns
                    if self.policy.manual_hold_enabled
                    or frame.synthetic
                    or self.config.mode == "qualified_state"
                    else self._event_ns
                )
                age_s = self.policy.input_max_age_s(self.config.input_max_age_s)
                if now >= origin + int(age_s * 1e9):
                    return self._stop("GCA_INPUT_EXPIRED")
                self.gate = "WAIT_STOP"
            return Intent(
                gate=self.gate,
                reason="STOP_PENDING",
                lt_pressed=self._lt_pressed,
                b_pressed=False,
                input_epoch=self._epoch,
                control_policy=self.policy.control_policy,
            )
        if not self._lt_pressed:
            return Intent(
                gate=self.gate, input_epoch=self._epoch, control_policy=self.policy.control_policy
            )
        if not self._active and not self._press_edge and self._pending_lt_epoch is None:
            return Intent(
                gate=self.gate,
                reason="NEW_LT_EDGE_REQUIRED",
                input_epoch=self._epoch,
                control_policy=self.policy.control_policy,
            )
        if self._event_seq < 1:
            return self._stop("NO_REAL_GCA_EVENT")
        if self.policy.manual_hold_enabled:
            source, origin, evidence = "JOYDEV_HELD_STATE", frame.observed_ns, "UNKNOWN"
        elif frame.synthetic:
            source, origin, evidence = "SCRIPT_SAMPLE", frame.observed_ns, "SCRIPTED_ONLY"
        elif self.config.mode == "qualified_state":
            if not self._qualification:
                return self._stop("INPUT_NOT_QUALIFIED")
            source, origin, evidence = (
                "QUALIFIED_STATE_CONTRACT",
                frame.observed_ns,
                "QUALIFIED_STATE_CONTRACT",
            )
        else:
            source, origin, evidence = "EVENT_AGE", self._event_ns, "UNKNOWN"
        age_s = self.policy.input_max_age_s(self.config.input_max_age_s)
        deadline = origin + int(age_s * 1e9)
        if now >= deadline:
            return self._stop("GCA_INPUT_EXPIRED")
        quality = ManualInputQuality(
            source,
            frame.source_id,
            frame.input_epoch,
            self._event_seq,
            self._event_ns,
            "SCRIPT_MONOTONIC" if frame.synthetic else "JOYDEV_INIT_ANCHORED_U32_MS",
            frame.observation_seq,
            frame.observed_ns,
            deadline,
            frame.clock_domain_id,
            evidence,
            "" if self.policy.manual_hold_enabled else self._qualification,
        )
        self._pending_lt_epoch = None
        self.gate, self._active = "ACTIVE", True
        if self.policy.is_servo:
            self._servo_stop_reason = None
        return Intent(
            "JOG",
            0.0 if self.policy.is_servo else mapped.ly * self.config.max_linear_mm_s,
            0.0 if self.policy.is_servo else mapped.lx * self.config.max_rotation_deg_s,
            True,
            quality,
            gate=self.gate,
            lt_pressed=True,
            b_pressed=False,
            input_epoch=self._epoch,
            control_policy=self.policy.control_policy,
            servo_rpm=self.policy.servo_rpm(mapped) if self.policy.is_servo else 0.0,
        )
