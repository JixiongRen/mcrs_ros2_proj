"""Receiver-issued, single-use motion freshness independent from renewable leases.

The envelope is opaque to clients. The capability authenticates the current
application session; its nonce binds one command to a fixed receiver-local
monotonic deadline. Renewing the capability never moves a nonce deadline.
"""

import copy
import math
import secrets
import time
from collections import OrderedDict
from dataclasses import dataclass
from threading import RLock


class PermitError(ValueError):
    pass


def same_motion_payload(first, second):
    """Compare intent, excluding credentials and receiver-side admission budgets."""
    left, right = copy.deepcopy(first), copy.deepcopy(second)
    for goal in (left, right):
        goal.token = ""
        goal.admission_timeout_s = 0.0
    return left == right


def capability_part(envelope: str) -> str:
    if not isinstance(envelope, str) or len(envelope) > 512 or envelope.count(".") != 1:
        raise PermitError("MOTION_PERMIT_REQUIRED")
    capability, nonce = envelope.split(".", 1)
    if not capability or not nonce:
        raise PermitError("MOTION_PERMIT_REQUIRED")
    return capability


@dataclass(frozen=True)
class _Permit:
    context: tuple
    deadline: float


class MotionPermits:
    """A bounded nonce registry. Only its owning ingress callback mutates it."""

    def __init__(self, capacity=256, maximum_s=1.0, clock=time.monotonic):
        self.capacity, self.maximum_s, self.clock = capacity, maximum_s, clock
        self._pending = OrderedDict()
        self._lock = RLock()

    @staticmethod
    def _binding(context):
        return (
            context.session_id,
            int(context.generation),
            context.command_id,
            int(context.sequence),
            context.task_id,
        )

    def _expire(self):
        now = self.clock()
        for nonce, permit in list(self._pending.items()):
            if now >= permit.deadline:
                del self._pending[nonce]

    def issue(self, capability, context, ttl_s, lease_deadline):
        with self._lock:
            return self._issue(capability, context, ttl_s, lease_deadline)

    def _issue(self, capability, context, ttl_s, lease_deadline):
        self._expire()
        if not math.isfinite(ttl_s) or not 0 < ttl_s <= self.maximum_s:
            raise PermitError("INVALID_PERMIT_TTL")
        binding = self._binding(context)
        if (
            not binding[0]
            or not binding[2]
            or len(binding[0]) > 128
            or len(binding[2]) > 128
            or binding[1] < 1
            or binding[3] < 1
            or not binding[4]
            or len(binding[4]) > 128
        ):
            raise PermitError("INVALID_CONTEXT")
        now = self.clock()
        deadline = min(now + ttl_s, lease_deadline)
        if deadline <= now:
            raise PermitError("SESSION_EXPIRED")
        if len(self._pending) >= self.capacity:
            raise PermitError("PERMIT_CAPACITY")
        nonce = secrets.token_urlsafe(24)
        self._pending[nonce] = _Permit(binding, deadline)
        return capability + "." + nonce, deadline

    def consume(self, envelope, context):
        with self._lock:
            return self._consume(envelope, context)

    def _consume(self, envelope, context):
        capability_part(envelope)
        nonce = envelope.split(".", 1)[1]
        permit = self._pending.pop(nonce, None)
        if permit is None:
            raise PermitError("INVALID_OR_USED_PERMIT")
        if self.clock() >= permit.deadline:
            raise PermitError("MOTION_PERMIT_EXPIRED")
        if permit.context != self._binding(context):
            raise PermitError("MOTION_PERMIT_CONTEXT_MISMATCH")
        return permit.deadline
