"""One control owner's count/reference state and bounded physical-time velocity window."""

from collections import deque
from dataclasses import replace
from math import ceil, isfinite
from uuid import uuid4

from gca.domain.models import OperationResult
from gca.domain.position import ExternalPositionFeedback
from gca.estimation.linear import LinearEstimator


class EncoderFeedback:
    def __init__(self, config):
        self.config = config
        self._window_ns = round(config.velocity_window_s * 1e9)
        self._window = deque(
            maxlen=min(4096, ceil(config.velocity_window_s / config.sample_period_s) * 2 + 4)
        )
        self._reference_id = ""
        self._sample = None
        self._state = ExternalPositionFeedback(0.0, 0.0, False, False, 0, 0, "", "")
        self._make_estimator()

    def _make_estimator(self):
        self._estimator = LinearEstimator(
            self.config.mm_per_count,
            self.config.max_speed_mm_s,
            self.config.feedback_max_age_s,
            self.config.count_modulus,
        )

    def invalidate(self, reason):
        self._reference_id = ""
        self._window.clear()
        self._make_estimator()
        self._state = replace(
            self._state,
            position_valid=False,
            velocity_valid=False,
            reference_id="",
            error_code=reason,
        )

    def snapshot(self, now_ns):
        state = self._state
        if not state.device_epoch:
            return state
        if not 0 <= now_ns - state.evidence_ns <= round(self.config.feedback_max_age_s * 1e9):
            return replace(
                state,
                position_valid=False,
                velocity_valid=False,
                error_code="ENCODER_FEEDBACK_STALE",
            )
        return state

    def update(self, sample, now_ns):
        if not sample.valid or sample.error_code:
            self.invalidate(sample.error_code or "ENCODER_SAMPLE_INVALID")
            return self.snapshot(now_ns)
        evidence = sample.request_started_ns or sample.received_ns
        if not 0 <= now_ns - evidence <= round(self.config.feedback_max_age_s * 1e9):
            self.invalidate("ENCODER_FEEDBACK_STALE")
            return self.snapshot(now_ns)
        previous = self._sample
        changed_epoch = previous is not None and sample.device_epoch != previous.device_epoch
        if previous is not None and not changed_epoch and sample.seq <= previous.seq:
            return self.snapshot(now_ns)
        reason = ""
        if changed_epoch:
            self.invalidate("ENCODER_EPOCH_CHANGED")
            reason = "ENCODER_EPOCH_CHANGED"
        elif previous is not None:
            dt_ns = sample.received_ns - previous.received_ns
            if dt_ns <= 0:
                self.invalidate("ENCODER_TIME_INVALID")
                return self.snapshot(now_ns)
            if dt_ns > round(self.config.feedback_max_age_s * 1e9):
                self.invalidate("ENCODER_SAMPLE_GAP")
                reason = "ENCODER_SAMPLE_GAP"
        self._sample = sample
        estimate = self._estimator.update(sample)
        if not estimate.valid:
            self.invalidate(estimate.error or "ENCODER_ESTIMATE_INVALID")
            return self.snapshot(now_ns)
        position = self.config.direction_sign * estimate.position_mm
        self._window.append((sample.received_ns, position))
        while len(self._window) > 2 and sample.received_ns - self._window[1][0] >= self._window_ns:
            self._window.popleft()
        span = sample.received_ns - self._window[0][0]
        velocity_valid = span >= self._window_ns
        velocity = ((position - self._window[0][1]) / (span / 1e9)) if velocity_valid else 0.0
        self._state = ExternalPositionFeedback(
            position,
            velocity,
            bool(self._reference_id),
            velocity_valid,
            sample.seq,
            sample.received_ns,
            sample.device_epoch,
            self._reference_id,
            sample.request_started_ns,
            sample.raw_count,
            reason,
        )
        return self.snapshot(now_ns)

    def set_reference(self, position_mm, now_ns):
        if type(position_mm) not in (float, int) or not isfinite(position_mm):
            return OperationResult(False, "INVALID_REFERENCE")
        current = self.snapshot(now_ns)
        if self._sample is None or not self._estimator.state.valid or current.error_code:
            return OperationResult(False, "ENCODER_FEEDBACK_INVALID")
        self._estimator.set_reference(position_mm / self.config.direction_sign)
        self._reference_id = str(uuid4())
        self._window.clear()
        self._window.append((current.sampled_ns, position_mm))
        self._state = replace(
            current,
            position_mm=float(position_mm),
            position_valid=True,
            velocity_mm_s=0.0,
            velocity_valid=False,
            reference_id=self._reference_id,
            error_code="",
        )
        return OperationResult(True, reference_id=self._reference_id)
