"""Encoder count unwrapping, software reference and measured velocity."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LinearEstimate:
    position_mm: float = 0.0
    velocity_mm_s: float = 0.0
    valid: bool = False
    velocity_valid: bool = False
    seq: int = 0
    received_ns: int = 0
    epoch: str = ""
    referenced: bool = False
    error: str = ""


class LinearEstimator:
    """Single-owner estimator; rejected samples never refresh the last good age."""

    def __init__(self, mm_per_count, max_speed, max_dt, modulus=0):
        self.scale, self.max_speed, self.max_dt = mm_per_count, max_speed, max_dt
        self.modulus = modulus
        self.state = LinearEstimate()
        self._last_raw = None
        self._unwrapped = 0
        self._offset = 0.0

    def set_reference(self, position_mm=0.0):
        if not self.state.valid:
            raise ValueError("valid sample required to establish reference")
        self._offset = self._unwrapped * self.scale - position_mm
        self.state = LinearEstimate(
            position_mm,
            self.state.velocity_mm_s,
            True,
            self.state.velocity_valid,
            self.state.seq,
            self.state.received_ns,
            self.state.epoch,
            True,
        )

    def update(self, sample):
        old = self.state
        if not sample.valid:
            return old
        if sample.device_epoch == old.epoch and sample.seq <= old.seq:
            return old
        epoch_change = bool(old.epoch and sample.device_epoch != old.epoch)
        if self._last_raw is None or epoch_change:
            self._last_raw, self._unwrapped = sample.raw_count, sample.raw_count
            self._offset = 0.0
            self.state = LinearEstimate(
                sample.raw_count * self.scale,
                0.0,
                True,
                False,
                sample.seq,
                sample.received_ns,
                sample.device_epoch,
                False,
                "ENCODER_RECONNECTED" if epoch_change else "",
            )
            return self.state
        dt = (sample.received_ns - old.received_ns) / 1e9
        if dt <= 0:
            return old
        delta = sample.raw_count - self._last_raw
        if self.modulus:
            delta = (delta + self.modulus // 2) % self.modulus - self.modulus // 2
        if abs(delta * self.scale) > self.max_speed * dt + 2 * self.scale:
            # Preserve the last timestamp: a bad jump cannot keep output alive.
            self.state = LinearEstimate(
                old.position_mm,
                old.velocity_mm_s,
                False,
                False,
                old.seq,
                old.received_ns,
                old.epoch,
                old.referenced,
                "ENCODER_JUMP",
            )
            return self.state
        self._unwrapped += delta
        self._last_raw = sample.raw_count
        velocity_valid = dt <= self.max_dt
        velocity = delta * self.scale / dt
        if old.velocity_valid and velocity_valid:
            velocity = 0.5 * velocity + 0.5 * old.velocity_mm_s
        self.state = LinearEstimate(
            self._unwrapped * self.scale - self._offset,
            velocity,
            True,
            velocity_valid,
            sample.seq,
            sample.received_ns,
            sample.device_epoch,
            old.referenced,
            "",
        )
        return self.state
