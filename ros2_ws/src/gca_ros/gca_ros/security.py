"""Bounded, monotonic capabilities; these supplement deployment access control.

A coordinator key authorizes lease issuance. A source or executor token only
represents an application capability, never the identity of a DDS participant.
Tokens must never be logged or placed in telemetry.
"""

import math
import secrets
import time
from dataclasses import dataclass
from functools import wraps
from threading import RLock
from typing import Callable


def synchronized(method):
    @wraps(method)
    def guarded(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return guarded


class AccessError(ValueError):
    """Stable admission failure suitable for an external error code."""


@dataclass(frozen=True)
class Grant:
    session_id: str
    generation: int
    token: str
    deadline: float
    sequence: int


class LeaseAuthority:
    """One executor capability, with expiry evaluated only on this host's clock."""

    def __init__(
        self,
        coordinator_key: str,
        maximum_s: float = 5.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        if not coordinator_key or not math.isfinite(maximum_s) or maximum_s <= 0:
            raise ValueError("coordinator key and positive finite maximum lease required")
        self._lock = RLock()
        self._key = coordinator_key
        self._maximum = maximum_s
        self._clock = clock
        self.grant: Grant | None = None
        self._retired: Grant | None = None
        self._retired_until = 0.0

    @property
    def transaction_lock(self):
        """Protect authenticate-and-latch STOP against a concurrent new owner."""
        return self._lock

    @synchronized
    def issue(
        self,
        key: str,
        session_id: str,
        generation: int,
        sequence: int,
        duration_s: float,
        token: str = "",
    ) -> Grant:
        if not secrets.compare_digest(key, self._key):
            raise AccessError("UNAUTHORIZED_COORDINATOR")
        if not session_id or len(session_id) > 128 or generation < 1 or sequence < 1:
            raise AccessError("INVALID_CONTEXT")
        if not math.isfinite(duration_s) or duration_s <= 0 or duration_s > self._maximum:
            raise AccessError("INVALID_LEASE")
        now = self._clock()
        old = self.grant
        if old is not None:
            if now >= old.deadline:
                # Renewing an expired capability would resurrect a revoked execution.
                raise AccessError("SESSION_EXPIRED")
            if old.session_id != session_id or old.generation != generation:
                raise AccessError("CONTROL_BUSY")
            if not token or not secrets.compare_digest(old.token, token):
                raise AccessError("INVALID_TOKEN")
            if sequence <= old.sequence:
                raise AccessError("STALE_SEQUENCE")
            issued = old.token
        else:
            if token:
                raise AccessError("STALE_TOKEN")
            issued = secrets.token_urlsafe(32)
        self.grant = Grant(session_id, generation, issued, now + duration_s, sequence)
        return self.grant

    @synchronized
    def verify(
        self, token: str, session_id: str, generation: int, allow_expired: bool = False
    ) -> Grant:
        grant = self.grant
        if grant is None or not token or not secrets.compare_digest(token, grant.token):
            raise AccessError("INVALID_TOKEN")
        if session_id != grant.session_id or generation != grant.generation:
            raise AccessError("STALE_GENERATION")
        if not allow_expired and self._clock() >= grant.deadline:
            raise AccessError("SESSION_EXPIRED")
        return grant

    @synchronized
    def revoke(self) -> None:
        if self.grant is not None:
            self._retired = self.grant
            self._retired_until = self._clock() + self._maximum
        self.grant = None

    @synchronized
    def verify_retired(self, token, session_id, generation):
        """Only callers implementing stop or simulation-clear may use this path."""
        retired = self._retired
        if (
            self.grant is not None
            or retired is None
            or self._clock() >= self._retired_until
            or not token
            or not secrets.compare_digest(token, retired.token)
            or retired.session_id != session_id
            or retired.generation != generation
        ):
            raise AccessError("INVALID_RETIRED_CAPABILITY")
        return retired

    @synchronized
    def coordinator_stop(self, key, session_id):
        """Privileged stop bridge during executor-token handover; never enables."""
        if not key or not secrets.compare_digest(key, self._key):
            raise AccessError("UNAUTHORIZED_COORDINATOR")
        known = self.grant or self._retired
        if known is None or known.session_id != session_id:
            raise AccessError("STOP_SESSION_MISMATCH")
        if self.grant is None and self._clock() >= self._retired_until:
            raise AccessError("INVALID_RETIRED_CAPABILITY")
        return known
