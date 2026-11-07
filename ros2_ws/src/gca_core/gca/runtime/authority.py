"""Shared executor lease, context and revocation checks.

Hosts provide immutable config, current lease/deadline, stop fences and bounded
history. This mixin creates no tokens, threads, devices or second authority.
The coordinator/device ROS boundary authenticates the lease before admission.
"""

from math import isfinite

from gca.domain.models import CommandContext, Lease, OperationResult


class RuntimeAuthority:
    allowed_control_policies = ("LEGACY_DUAL_AXIS",)

    def _validate_lease(self, lease, now, renewal=False):
        if not isinstance(lease, Lease) or not lease.session_id or len(lease.session_id) > 128:
            return OperationResult(False, "INVALID_SESSION")
        if (
            type(lease.generation) is not int
            or type(lease.sequence) is not int
            or min(lease.generation, lease.sequence) < 0
            or type(lease.ttl_s) not in (int, float)
            or not isfinite(lease.ttl_s)
            or not 0 < lease.ttl_s <= self.config.lease_max_s
        ):
            return OperationResult(False, "INVALID_LEASE")
        if lease.control_policy not in self.allowed_control_policies:
            return OperationResult(False, "CONTROL_POLICY_UNSUPPORTED")
        if renewal:
            if self._lease is not None and lease.control_policy != self._lease.control_policy:
                return OperationResult(False, "CONTROL_POLICY_CHANGED")
            if (
                self._lease is None
                or now >= self._lease_deadline
                or lease.session_id != self._lease.session_id
                or lease.generation != self._lease.generation
                or lease.sequence <= self._lease.sequence
                or lease.token != self._lease.token
            ):
                return OperationResult(False, "STALE_LEASE")
        elif lease.generation <= self._revoked_generations.get(lease.session_id, -1):
            return OperationResult(False, "REVOKED_GENERATION")
        return OperationResult(True)

    def _validate_manual_permission(self, context, token, now):
        if (
            not isinstance(context, CommandContext)
            or not all(
                isinstance(v, str) and 0 < len(v) <= 128
                for v in (context.session_id, context.task_id, context.command_id)
            )
            or any(type(v) is not int or v < 0 for v in (context.generation, context.sequence))
        ):
            return OperationResult(False, "INVALID_CONTEXT")
        if context.sequence <= self._stop_fences.get((context.session_id, context.generation), -1):
            return OperationResult(False, "STOP_FENCED")
        if (
            not self._state.armed
            or not self._active_lifecycle
            or self._lease is None
            or now >= self._lease_deadline
            or context.session_id != self._lease.session_id
            or context.generation != self._lease.generation
            or token != self._lease.token
        ):
            return OperationResult(False, "INVALID_PERMISSION")
        return OperationResult(True)

    def _revoke_current_lease(self):
        if self._lease:
            session = self._lease.session_id
            self._revoked_generations[session] = max(
                self._lease.generation, self._revoked_generations.get(session, -1)
            )
            self._revoked_generations.move_to_end(session)
            self._quality_tracker.clear_session(session, self._lease.generation)
            while len(self._revoked_generations) > self.config.history_capacity:
                self._revoked_generations.popitem(last=False)
