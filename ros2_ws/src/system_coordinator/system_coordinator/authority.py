"""Source control sessions; authorization is independent from DDS authentication."""

import copy
import math
import secrets
import time
import uuid
from dataclasses import dataclass


class SessionError(ValueError):
    pass


def bounded_positive_seconds(value, name, minimum=0.0):
    """Validate scheduling/authorization budgets before constructing deadlines."""
    if isinstance(value, bool):
        raise ValueError(name + " must be a positive finite number")
    result = float(value)
    if not math.isfinite(result) or result <= 0 or result < minimum:
        raise ValueError(name + " must be positive, finite and within its minimum")
    return result


@dataclass
class SourceSession:
    session_id: str
    generation: int
    token: str
    owner: str
    mode: str
    deadline: float
    renewal_sequence: int
    executor_token: str = ""
    needs_reauthorization: bool = False


class SourceAuthority:
    def __init__(self, maximum_s=5.0, clock=time.monotonic):
        self.maximum_s = bounded_positive_seconds(maximum_s, "source_lease_max_s", minimum=0.1)
        self.clock = clock
        self.current = None
        self._generation = 0
        self._retired = None
        self._retired_until = 0.0

    def duration(self, value):
        if not math.isfinite(value) or not 0.1 <= value <= self.maximum_s:
            raise SessionError("INVALID_LEASE")
        return float(value)

    def claim(self, owner, mode, duration_s):
        if self.current is not None:
            raise SessionError("CONTROL_BUSY")
        if not owner or len(owner) > 128 or mode not in ("MANUAL", "AUTO", "MAINTENANCE"):
            raise SessionError("INVALID_OWNER_OR_MODE")
        duration_s = self.duration(duration_s)
        self._generation += 1
        self.current = SourceSession(
            str(uuid.uuid4()),
            self._generation,
            secrets.token_urlsafe(32),
            owner,
            mode,
            self.clock() + duration_s,
            1,
        )
        return self.current

    def verify(self, session_id, generation, token, allow_expired=False):
        current = self.current
        if current is None or not token or not secrets.compare_digest(current.token, token):
            raise SessionError("INVALID_TOKEN")
        if session_id != current.session_id or generation != current.generation:
            raise SessionError("STALE_GENERATION")
        if not allow_expired and self.clock() >= current.deadline:
            raise SessionError("SESSION_EXPIRED")
        return current

    def release(self, remember=True):
        if remember and self.current is not None:
            self._retired = copy.copy(self.current)
            self._retired_until = self.clock() + self.maximum_s
        self.current = None

    def verify_retired(self, session_id, generation, token):
        """Bounded stop/simulation-clear authority, never a renewable session."""
        retired = self._retired
        if (
            self.current is not None
            or retired is None
            or self.clock() >= self._retired_until
            or not token
            or not secrets.compare_digest(token, retired.token)
            or retired.session_id != session_id
            or retired.generation != generation
        ):
            raise SessionError("INVALID_RETIRED_CAPABILITY")
        return retired

    def rotate_generation(self, source):
        if self.current is not source:
            raise SessionError("SESSION_EXPIRED")
        self._generation += 1
        source.generation = self._generation
        source.renewal_sequence += 1
        source.executor_token = ""
