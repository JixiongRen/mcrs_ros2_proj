"""Device-port simulations driven by a plant independent from motion targets."""

from bisect import bisect_right
from collections import deque
from random import Random
from time import monotonic_ns
from uuid import uuid4

from gca.algorithms.pulse_timing import validate_edge_offsets
from gca.domain.models import (
    DeviceError,
    DeviceIdentity,
    EncoderSample,
    PulseAbort,
    PulseCompletion,
    ServoSample,
)

from .plant import MechanicalPlant


class SimServo:
    def __init__(self, plant, clock=monotonic_ns):
        self.plant, self.clock = plant, clock
        self.epoch, self.enabled, self.opened = str(uuid4()), False, False
        self.fail_write = self.fail_read = False

    def open(self):
        self.opened = True
        return DeviceIdentity(
            "SIM_VELOCITY_SERVO", self.epoch, capabilities=("velocity", "disable")
        )

    def configure(self):
        self.enabled = False
        self.plant.accept_velocity(0.0, self.clock())

    def enable(self, enabled):
        self.enabled = enabled
        if not enabled:
            self.plant.disable(self.clock())

    def write_velocity(self, velocity_mm_s, deadline_ns):
        if self.fail_write or self.clock() >= deadline_ns:
            raise DeviceError("SERVO_TIMEOUT")
        if velocity_mm_s and not self.enabled:
            raise DeviceError("SERVO_DISABLED")
        return self.plant.accept_velocity(velocity_mm_s, self.clock())

    def read_state(self, deadline_ns):
        if self.fail_read or self.clock() >= deadline_ns:
            raise DeviceError("SERVO_TIMEOUT")
        state = self.plant.advance(self.clock())
        return ServoSample(state.velocity_mm_s, self.enabled, self.clock(), self.epoch)

    def stop(self, deadline_ns):
        self.write_velocity(0.0, deadline_ns)

    def close(self):
        self.enable(False)
        self.opened = False


class SimEncoder:
    def __init__(
        self, plant, mm_per_count=0.01, clock=monotonic_ns, noise_mm=0.0, delay_s=0.0, seed=7
    ):
        self.plant, self.scale, self.clock = plant, mm_per_count, clock
        self.noise, self.delay_ns, self.random = noise_mm, int(delay_s * 1e9), Random(seed)
        self.epoch, self.seq = str(uuid4()), 0
        self.disconnected = self.frozen = self.invalid = False
        self.jump_mm = 0.0
        self._last = None
        self._delay = deque(maxlen=4096)

    def open(self):
        return DeviceIdentity("SIM_EXTERNAL_ENCODER", self.epoch)

    def read(self, deadline_ns):
        now = self.clock()
        if self.disconnected or now >= deadline_ns:
            raise DeviceError("ENCODER_TIMEOUT")
        if self.frozen and self._last is not None:
            return self._last  # Original seq/time deliberately preserved.
        physical = self.plant.advance(now)
        value = physical.position_mm + self.jump_mm + self.random.gauss(0.0, self.noise)
        self.seq += 1
        sample = EncoderSample(
            round(value / self.scale),
            self.seq,
            now,
            self.epoch,
            not self.invalid,
            "BAD_FRAME" if self.invalid else "",
            now,
        )
        self._delay.append((now + self.delay_ns, sample))
        ready = None
        while self._delay and self._delay[0][0] <= now:
            _, ready = self._delay.popleft()
        if ready is not None:
            self._last = ready
        if self._last is None:
            raise DeviceError("ENCODER_WAITING")
        return self._last

    def close(self):
        self._delay.clear()


class SimPulseEngine:
    """One finite active segment; acceptance never increments confirmed count."""

    def __init__(self, plant, clock=monotonic_ns, *, high_us=5, low_us=5, setup_us=10, hold_us=10):
        self.plant, self.clock, self.epoch = plant, clock, str(uuid4())
        self.timing = dict(high_us=high_us, low_us=low_us, setup_us=setup_us, hold_us=hold_us)
        self._active = None
        self._started = self._emitted = 0
        self.unknown_abort = True
        self.fail_submit = False

    def open(self):
        return DeviceIdentity(
            "SIM_FINITE_PULSE_ENGINE", self.epoch, capabilities=("finite_waveform",)
        )

    def submit(self, segment):
        if self.fail_submit:
            raise DeviceError("PULSE_SUBMIT_FAILED")
        if self._active is not None or segment.device_epoch != self.epoch:
            raise DeviceError("PULSE_BUSY_OR_EPOCH")
        if segment.duration_s <= 0 or not segment.signed_pulses:
            raise DeviceError("INVALID_PULSE_SEGMENT")
        try:
            validate_edge_offsets(segment, **self.timing)
        except ValueError as exc:
            raise DeviceError("PULSE_TIMING_INVALID", str(exc)) from exc
        if type(segment.expires_ns) is not int or segment.expires_ns < 0:
            raise DeviceError("INVALID_ARGUMENT")
        started_ns = self.clock()
        if segment.expires_ns and started_ns + int(segment.duration_s * 1e9) > segment.expires_ns:
            raise DeviceError("PULSE_EXPIRED", "finite pulse would exceed output window")
        self._active, self._started, self._emitted = segment, started_ns, 0

    def _advance(self):
        if self._active is None:
            return False
        elapsed = (self.clock() - self._started) / 1e9
        fraction = min(1.0, max(0.0, elapsed / self._active.duration_s))
        pulses = (
            bisect_right(self._active.edge_offsets_us, round(elapsed * 1e6))
            if self._active.edge_offsets_us is not None
            else int(abs(self._active.signed_pulses) * fraction)
        )
        if self._active.signed_pulses < 0:
            pulses = -pulses
        self.plant.emit_pulses(pulses - self._emitted)
        self._emitted = pulses
        return fraction >= 1.0

    def poll(self):
        if not self._advance():
            return []
        segment = self._active
        self._active = None
        return [
            PulseCompletion(
                segment.segment_id,
                segment.signed_pulses,
                segment.generation,
                self.epoch,
                segment.command_id,
            )
        ]

    def abort(self):
        if self._active is None:
            return PulseAbort(0, True)
        self._advance()
        count = None if self.unknown_abort else self._emitted
        self._active = None
        return PulseAbort(count, True)

    def is_idle(self):
        return self._active is None

    def close(self):
        self.abort()


def simulated_devices(config, clock=monotonic_ns):
    """Return (servo, encoder, pulse, plant), all sharing only physical state."""
    scale = 360 * config.catheter_per_motor_ratio / config.pulses_per_motor_rev
    plant = MechanicalPlant(clock(), slip=config.simulation_slip, degrees_per_pulse=scale)
    return (
        SimServo(plant, clock),
        SimEncoder(
            plant,
            config.encoder_mm_per_count,
            clock,
            config.simulation_noise_mm,
            config.simulation_encoder_delay_s,
            config.simulation_seed,
        ),
        SimPulseEngine(
            plant,
            clock,
            high_us=config.pulse_high_us,
            low_us=config.pulse_low_us,
            setup_us=config.direction_setup_us,
            hold_us=config.direction_hold_us,
        ),
        plant,
    )
