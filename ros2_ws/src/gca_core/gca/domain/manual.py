"""Immutable manual-input provenance and receiver-local freshness validation.

Observation proves that a reader progressed, not that a radio packet arrived.
A receiver never replaces the source deadline with its own receipt timestamp.
"""

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from threading import RLock


class QualityError(ValueError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


@lru_cache(maxsize=1)
def host_clock_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()


@dataclass(frozen=True)
class ManualInputQuality:
    source_kind: str
    source_id: str
    input_epoch: str
    event_seq: int
    event_ns: int
    event_time_source: str
    observation_seq: int
    observation_ns: int
    valid_until_ns: int
    clock_domain_id: str
    link_evidence: str
    qualification_id: str = ""
    episode_end_ns: int = 0


class QualityTracker:
    """Bounded anti-rollback history scoped to an authenticated source session.

    Reusing a still-valid unchanged physical sample is allowed. Its deadline is
    immutable; command sequence replay is checked separately by the receiver.
    History is never silently evicted: exhausting the bound fails closed.
    """

    def __init__(
        self,
        backend="simulation",
        max_age_s=0.15,
        clock_domain_id=None,
        capacity=128,
        programmed_max_s=3600.0,
    ):
        import math

        if (
            isinstance(max_age_s, bool)
            or not math.isfinite(max_age_s)
            or max_age_s <= 0
            or type(capacity) is not int
            or capacity < 1
            or not math.isfinite(programmed_max_s)
            or programmed_max_s <= 0
        ):
            raise ValueError("invalid manual quality limits")
        self.backend = backend
        self.max_age_ns = int(max_age_s * 1e9)
        self.programmed_max_ns = int(programmed_max_s * 1e9)
        self.clock_domain_id = clock_domain_id or host_clock_id()
        self.capacity = capacity
        self._history = {}
        self._current = {}
        self._lock = RLock()

    def validate(self, quality, now_ns, *, session_id="", generation=0):
        with self._lock:
            return self._validate(quality, now_ns, session_id, generation)

    def _validate(self, q, now, session, generation):
        if not isinstance(q, ManualInputQuality):
            raise QualityError("MANUAL_QUALITY_REQUIRED")
        for name in (
            "source_kind",
            "source_id",
            "input_epoch",
            "event_time_source",
            "clock_domain_id",
            "link_evidence",
            "qualification_id",
        ):
            value = getattr(q, name)
            if not isinstance(value, str) or len(value) > 128:
                raise QualityError("INVALID_MANUAL_QUALITY")
        if not q.source_id or not q.input_epoch or not q.event_time_source:
            raise QualityError("INVALID_MANUAL_QUALITY")
        for name in (
            "event_seq",
            "event_ns",
            "observation_seq",
            "observation_ns",
            "valid_until_ns",
            "episode_end_ns",
        ):
            value = getattr(q, name)
            if type(value) is not int or value < 0 or value >= 2**64:
                raise QualityError("INVALID_MANUAL_QUALITY")
        if q.clock_domain_id != self.clock_domain_id:
            raise QualityError("INPUT_CLOCK_MISMATCH")
        if (
            q.event_seq < 1
            or q.observation_seq < 1
            or q.event_ns > q.observation_ns
            or q.observation_ns > now
        ):
            raise QualityError("INPUT_TIME_INVALID")
        if q.source_kind == "EVENT_AGE":
            origin = q.event_ns
            if self.backend != "simulation":
                raise QualityError("INPUT_NOT_QUALIFIED")
        elif q.source_kind == "QUALIFIED_STATE_CONTRACT":
            if not q.qualification_id or q.link_evidence != "QUALIFIED_STATE_CONTRACT":
                raise QualityError("INPUT_NOT_QUALIFIED")
            origin = q.observation_ns
        elif q.source_kind == "SCRIPT_SAMPLE":
            if self.backend != "simulation":
                raise QualityError("SCRIPT_REQUIRES_SIMULATION")
            origin = q.observation_ns
        elif q.source_kind == "PROGRAMMED_JOG":
            origin = q.observation_ns
            if (
                q.episode_end_ns <= q.event_ns
                or q.episode_end_ns - q.event_ns > self.programmed_max_ns
            ):
                raise QualityError("INVALID_PROGRAMMED_EPISODE")
        else:
            raise QualityError("INVALID_INPUT_SOURCE")
        ceiling = origin + self.max_age_ns
        if q.source_kind == "PROGRAMMED_JOG":
            ceiling = min(ceiling, q.episode_end_ns)
        if q.valid_until_ns <= origin or q.valid_until_ns > ceiling:
            raise QualityError("INPUT_DEADLINE_INVALID")
        if now >= q.valid_until_ns or now - q.observation_ns >= self.max_age_ns:
            raise QualityError("INPUT_EXPIRED")
        key = (session, generation, q.source_id)
        full = (*key, q.input_epoch)
        current = self._current.get(key)
        previous = self._history.get(full)
        if current is not None and current != q.input_epoch and previous is not None:
            raise QualityError("INPUT_EPOCH_REPLAY")
        if previous is None and len(self._history) >= self.capacity:
            raise QualityError("INPUT_HISTORY_CAPACITY")
        if previous is not None:
            if q.source_kind != previous.source_kind:
                raise QualityError("INPUT_SOURCE_CHANGED")
            if q.event_seq < previous.event_seq or q.observation_seq < previous.observation_seq:
                raise QualityError("INPUT_SEQUENCE_REPLAY")
            if q.event_ns < previous.event_ns or q.observation_ns < previous.observation_ns:
                raise QualityError("INPUT_TIME_REPLAY")
            if q.event_seq == previous.event_seq and q.event_ns != previous.event_ns:
                raise QualityError("INPUT_EVENT_REWRITTEN")
            if q.observation_seq == previous.observation_seq and (
                q.observation_ns != previous.observation_ns
                or q.valid_until_ns != previous.valid_until_ns
            ):
                raise QualityError("INPUT_OBSERVATION_REWRITTEN")
            if (
                q.source_kind == "EVENT_AGE"
                and q.event_seq == previous.event_seq
                and q.valid_until_ns != previous.valid_until_ns
            ):
                raise QualityError("INPUT_DEADLINE_REWRITTEN")
            if q.source_kind == "PROGRAMMED_JOG" and (
                q.episode_end_ns != previous.episode_end_ns or q.event_ns != previous.event_ns
            ):
                raise QualityError("PROGRAMMED_EPISODE_REWRITTEN")
            if q.qualification_id != previous.qualification_id:
                raise QualityError("INPUT_QUALIFICATION_CHANGED")
        self._history[full] = q
        self._current[key] = q.input_epoch
        return q.valid_until_ns

    def clear_session(self, session_id, generation=None):
        with self._lock:
            for mapping in (self._history, self._current):
                for key in list(mapping):
                    if key[0] == session_id and (generation is None or key[1] == generation):
                        del mapping[key]
