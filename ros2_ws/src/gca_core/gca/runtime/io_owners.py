"""Exclusive device I/O ownership and output supervision for ControlRuntime.

This narrow mixin groups the three independent I/O owners and their bounded
mailboxes. It borrows the host runtime's immutable configuration, local clock,
permission flags and transport buffers; it does not create a second motion
state machine or controller. Hardware is accessed only inside the relevant
owner method. Owners publish raw samples/receipts; the control owner remains
the sole writer of domain motion snapshots and terminal candidates.

Output deadlines are checked here even while the control task/ROS executor is
paused. Whole-process/OS failures still require verified device watchdogs and
finite waveform limits. No I/O operation is performed under a state lock.
"""

from queue import Full


class DeviceIOOwners:
    """Private implementation shared with the single ControlRuntime host.

    Runtime initialization supplies ports, config, clock, mailboxes and stop
    events before these methods are started as dedicated owner threads. This
    class has no independent public lifecycle and must not be instantiated.
    """

    def _io_event(self, kind: str, value: object | None = None) -> None:
        if kind == "stop_unknown":
            self._cleanup_failed.set()
        try:
            self._io_events.put_nowait((kind, value))
        except Full:
            self._emergency = "IO_EVENT_OVERFLOW"
            self._permit = False
            self._pulse_stop.set()
            if kind.startswith("pulse_"):
                # Losing a physical execution receipt destroys count evidence.
                # Latch only a safety flag here; domain state remains owned by
                # the control task, and snapshots immediately fail closed.
                self._count_evidence_lost.set()
        self._wake.set()

    def _valid_output(
        self, expires_ns: int, generation: int, feedback_ns: int = 0, stop_serial: int | None = None
    ) -> bool:
        now = self.clock()
        return (
            self._permit
            and self._stop_request is None
            and self._armed_stop_serial == self.stop_serial
            and self._active_lifecycle
            and self._lease is not None
            and now < self._lease_deadline
            and now < expires_ns
            and (not self._input_deadline or now < self._input_deadline)
            and generation == self._output_generation
            and (stop_serial is None or stop_serial == self.stop_serial)
            and (not feedback_ns or now - feedback_ns <= int(self.config.feedback_max_age_s * 1e9))
        )

    def _servo_owner(self) -> None:
        last_seq = -1
        last_motion = False
        zero_reported_generation = None
        try:
            self.servo.open()
            self.servo.configure()
            self.servo.enable(False)
            self._ready[0].set()
            while not self._shutdown.is_set():
                deadline = self.clock() + int(self.config.io_timeout_s * 1e9)
                enabled = bool(
                    self._lease
                    and self._active_lifecycle
                    and self.clock() < self._lease_deadline
                    and self._state.armed
                )
                if self._servo_enabled != enabled:
                    if enabled:
                        # The stop fence linearizes before any not-yet-started
                        # enable transaction. Underlying SDK I/O remains bounded.
                        with self._stop_lock:
                            if self._armed_stop_serial == self.stop_serial:
                                self.servo.enable(True)
                                self._servo_enabled = True
                    else:
                        self.servo.enable(False)
                        self._servo_enabled = False
                demand = self._velocity_demand.get()
                with self._stop_lock:
                    valid = demand and self._valid_output(
                        demand.expires_ns, demand.generation, demand.feedback_ns, demand.stop_serial
                    )
                    if valid and demand.seq != last_seq:
                        accepted = self.servo.write_velocity(demand.velocity, deadline)
                        self._io_event("applied", (demand.seq, demand.generation, accepted))
                        last_seq, last_motion = demand.seq, abs(demand.velocity) > 0
                if not valid:
                    stopping_generation = self._output_generation
                    self.servo.stop(deadline)
                    if last_motion or zero_reported_generation != stopping_generation:
                        # A successful zero write updates accepted-setpoint
                        # telemetry only. Physical stop still needs fresh
                        # velocity/disable evidence in the control owner.
                        self._io_event("zero_applied", stopping_generation)
                        zero_reported_generation = stopping_generation
                    if (
                        last_motion
                        and self._permit
                        and demand
                        and demand.generation == self._output_generation
                    ):
                        self._permit = False
                        self._pulse_stop.set()
                        if self._jog is not None and self.clock() >= self._jog_deadline:
                            self._io_event("jog_expired", "JOG_EXPIRED")
                        else:
                            self._io_event("fault", "OUTPUT_EXPIRED")
                    last_motion = False
                self._servo_sample.put(self.servo.read_state(deadline))
                self._shutdown.wait(self.config.io_period_s)
        except Exception as exc:
            self._emergency = getattr(exc, "code", "SERVO_IO_ERROR")
            self._permit = False
            self._pulse_stop.set()
            self._io_event("fault", self._emergency)
            self._ready[0].set()
            try:
                stopping_generation = self._output_generation
                self.servo.stop(self.clock() + int(self.config.io_timeout_s * 1e9))
                self._io_event("zero_applied", stopping_generation)
            except Exception:
                self._io_event("stop_unknown", "SERVO_STOP_FAILED")
            try:
                self.servo.enable(False)
            except Exception:
                self._io_event("stop_unknown", "SERVO_DISABLE_FAILED")
        finally:
            try:
                self.servo.close()
            except Exception as exc:
                self._io_event("stop_unknown", str(exc))

    def _encoder_owner(self) -> None:
        try:
            self.encoder.open()
            self._ready[1].set()
            while not self._shutdown.is_set():
                try:
                    sample = self.encoder.read(self.clock() + int(self.config.io_timeout_s * 1e9))
                    if sample.valid:
                        self._sample.put(sample)
                except Exception as exc:
                    self._io_event("acquisition_error", getattr(exc, "code", "ENCODER_IO_ERROR"))
                self._shutdown.wait(self.config.encoder_period_s)
        except Exception as exc:
            self._emergency = getattr(exc, "code", "ENCODER_INIT_FAILED")
            self._ready[1].set()
        finally:
            try:
                self.encoder.close()
            except Exception as exc:
                self._io_event("acquisition_error", str(exc))

    def _pulse_owner(self) -> None:
        last_id = None
        active_id = None
        try:
            identity = self.pulse.open()
            self._pulse_epoch = identity.device_epoch
            self._ready[2].set()
            while not self._shutdown.is_set():
                # Poll before abort so a completely emitted segment is settled
                # once instead of being unnecessarily declared unknown.
                for completion in self.pulse.poll():
                    self._io_event("pulse_complete", completion)
                    if active_id == completion.segment_id:
                        active_id = None
                if self._pulse_stop.is_set():
                    queued = self._pulse_demand.get()
                    result = self.pulse.abort()
                    self._pulse_stop.clear()
                    self._pulse_demand.put(None)
                    self._io_event("pulse_aborted", (active_id, result))
                    active_id = None
                    if queued and queued[0].segment_id != last_id:
                        self._io_event("pulse_rejected", queued[0])
                item = self._pulse_demand.get()
                if item:
                    segment, expires_ns = item
                    if segment.segment_id != last_id and self.pulse.is_idle():
                        with self._stop_lock:
                            if self._valid_output(
                                expires_ns, segment.generation, stop_serial=segment.stop_serial
                            ) and self.clock() + int(segment.duration_s * 1e9) <= min(
                                expires_ns, self._lease_deadline
                            ):
                                active_id = segment.segment_id
                                self.pulse.submit(segment)
                                self._io_event("pulse_accepted", segment)
                            else:
                                self._io_event("pulse_rejected", segment)
                        last_id = segment.segment_id
                self._pulse_idle = self.pulse.is_idle()
                self._shutdown.wait(self.config.io_period_s)
        except Exception as exc:
            self._emergency = getattr(exc, "code", "PULSE_IO_ERROR")
            self._permit = False
            self._io_event("fault", self._emergency)
            try:
                receipt = self.pulse.abort()
                self._io_event("pulse_aborted", (active_id, receipt))
                active_id = None
            except Exception as abort_exc:
                self._io_event("stop_unknown", str(abort_exc))
            self._ready[2].set()
        finally:
            try:
                self.pulse.close()
                self._pulse_idle = self.pulse.is_idle()
            except Exception as exc:
                self._io_event("stop_unknown", str(exc))
