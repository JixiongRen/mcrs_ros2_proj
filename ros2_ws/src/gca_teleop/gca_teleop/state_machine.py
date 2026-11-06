"""Pure single-owner teleoperation gates and immutable input provenance.

No ROS, device writes, session claim or implicit ARM occurs here. The caller
serializes process/tick/config changes and delivers every STOP immediately.
"""

from __future__ import annotations

from gca.domain.manual import ManualInputQuality

from .config import read_evidence, validate_mapping_evidence, validate_qualification
from .mapping import map_input
from .model import InputError, InputIdentity, Intent


class TeleopPipeline:
    def __init__(self, config):
        self.config = config
        self._context = None
        self._epoch = ""
        self._evidence_epoch = ""
        self._evidence_error = ""
        self._qualification = ""
        self._frame = None
        self._mapped = None
        self._load_records()
        self.reset("STARTUP")

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
        if self.config.source != "joydev" or self.config.mode != "qualified_state":
            return "HARDWARE_INPUT_NOT_QUALIFIED"
        if self._evidence_error or not self._qualification:
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
        self._active = False
        self._event_ns = 0
        self._event_seq = 0
        self._stop_reason = reason
        self._stop_emitted = False

    def set_config(self, profile, *, stopped, disarmed):
        if stopped is not True or disarmed is not True:
            raise InputError("CONFIG_REQUIRES_STOPPED_DISARMED")
        self.config = profile
        self._load_records()
        self._frame = self._mapped = None
        self._context = None
        self._epoch = self._evidence_epoch = ""
        self.reset("PROFILE_CHANGED")

    def _stop(self, reason):
        changed = self._active or self._stop_reason != reason or not self._stop_emitted
        self.reset(reason)
        self._stop_emitted = True
        return Intent(
            action="STOP" if changed else "NONE",
            reason=reason,
            gate=self.gate,
            lt_pressed=self._lt_pressed,
            b_pressed=bool(self._mapped and self._mapped.b),
            input_epoch=self._epoch,
        )

    def _context_check(self, *, armed, ready, reference_id, session_key, hardware):
        context = (bool(armed), bool(ready), reference_id, session_key, bool(hardware))
        if self._context != context:
            self._context = context
            self.reset("CONTEXT_CHANGED")
            return True
        return False

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
    ):
        context_was_active = self._active
        changed_context = self._context_check(
            armed=armed,
            ready=ready,
            reference_id=reference_id,
            session_key=session_key,
            hardware=hardware,
        )
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
            mapped = map_input(frame, self.config)
        except InputError as exc:
            return self._stop(exc.code)
        previous = self._mapped
        self._mapped = mapped
        # B's current held state is enough to inhibit, including startup INIT.
        if mapped.b:
            return self._stop("B_STOP")
        before = self._lt_pressed
        if mapped.lt >= self.config.lt_press:
            self._lt_pressed = True
        elif mapped.lt <= self.config.lt_release:
            self._lt_pressed = False
        self._press_edge = (
            not before
            and self._lt_pressed
            and frame.event_seq > 0
            and self.config.mapping.lt.index in frame.changed_axes
        )
        if not self._lt_pressed and mapped.centered:
            self._neutral_observed = True
        # Left-side event age is independent of right controls and cache polls.
        meaningful = previous is not None and (
            (self.config.mapping.lx.index in frame.changed_axes and mapped.lx != previous.lx)
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
        return self._evaluate(health, now_ns, armed=armed, ready=ready, hardware=hardware)

    def tick(
        self, health, now_ns, *, armed, ready=True, reference_id="", session_key="", hardware=False
    ):
        changed = self._context_check(
            armed=armed,
            ready=ready,
            reference_id=reference_id,
            session_key=session_key,
            hardware=hardware,
        )
        if changed:
            return self._stop("CONTEXT_CHANGED")
        return self._evaluate(health, now_ns, armed=armed, ready=ready, hardware=hardware)

    def _evaluate(self, health, now, *, armed, ready, hardware):
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
        if hardware and (
            frame.synthetic or self.config.mode != "qualified_state" or not self._qualification
        ):
            return self._stop("HARDWARE_INPUT_NOT_QUALIFIED")
        if not armed or not ready:
            return self._stop("NOT_ARMED_READY")
        if self.gate == "WAIT_NEUTRAL":
            if self._neutral_observed and not self._lt_pressed and mapped.centered:
                self.gate = "READY"
                self._stop_reason, self._stop_emitted = "", False
            return Intent(gate=self.gate, reason="RELEASE_AND_CENTER", input_epoch=self._epoch)
        if not self._lt_pressed:
            return Intent(gate=self.gate, input_epoch=self._epoch)
        if not self._active and not self._press_edge:
            return Intent(gate=self.gate, reason="NEW_LT_EDGE_REQUIRED", input_epoch=self._epoch)
        if self._event_seq < 1:
            return self._stop("NO_REAL_GCA_EVENT")
        if frame.synthetic:
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
        deadline = origin + int(self.config.input_max_age_s * 1e9)
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
            self._qualification,
        )
        self.gate, self._active = "ACTIVE", True
        return Intent(
            "JOG",
            mapped.ly * self.config.max_linear_mm_s,
            mapped.lx * self.config.max_rotation_deg_s,
            True,
            quality,
            gate=self.gate,
            lt_pressed=True,
            b_pressed=False,
            input_epoch=self._epoch,
        )
