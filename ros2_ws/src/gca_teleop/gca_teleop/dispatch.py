"""One outstanding permit plus one replaceable latest intent, without ROS imports."""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from time import monotonic_ns


@dataclass(frozen=True)
class DispatchGuard:
    dispatch_epoch: int
    stop_epoch: int
    identity: tuple


@dataclass(frozen=True)
class _Offer:
    intent: object
    guard: DispatchGuard


@dataclass(frozen=True)
class _Flight:
    future: object
    intent: object
    context: object
    stop_epoch: int
    dispatch_epoch: int


class LatestDispatcher:
    def __init__(
        self,
        session,
        publish,
        *,
        max_ttl_s=0.15,
        clock=monotonic_ns,
        notify=lambda event: None,
        failure=lambda reason: None,
    ):
        self.session, self.publish, self.clock = session, publish, clock
        self.max_ttl_s = max_ttl_s
        self.notify, self.failure = notify, failure
        self._lock = RLock()
        self._latest = None
        self._flight = None
        self._publishing = None
        self._epoch = 0
        self.dropped_count = 0
        self.sent_count = 0

    @property
    def pending_count(self):
        with self._lock:
            return int(self._latest is not None)

    @property
    def inflight_count(self):
        with self._lock:
            return int(self._flight is not None or self._publishing is not None)

    def guard(self):
        """Capture before computing an intent, never after a concurrent STOP."""
        with self._lock:
            return DispatchGuard(self._epoch, self.session.stop_epoch, self.session.identity())

    def _current(self, guard):
        return (
            guard.dispatch_epoch == self._epoch
            and guard.stop_epoch == self.session.stop_epoch
            and guard.identity == self.session.identity()
        )

    def offer(self, intent, guard=None):
        with self._lock:
            if intent.action != "JOG" or not intent.deadman or intent.quality is None:
                return
            guard = self.guard() if guard is None else guard
            if not self._current(guard):
                self.dropped_count += 1
                return
            if self._latest is not None:
                self.dropped_count += 1
            self._latest = _Offer(intent, guard)

    def invalidate(self):
        with self._lock:
            self._epoch += 1
            self._latest = None
            flight, self._flight = self._flight, None
            if flight is not None:
                # rclpy removes a canceled pending request; a late remote reply
                # cannot regain this flight's generation or its fixed deadline.
                flight.future.cancel()

    def pump(self):
        error = None
        register = None
        with self._lock:
            now = self.clock()
            if self._publishing is not None:
                return
            if self._flight is not None:
                if now >= self._flight.intent.quality.valid_until_ns:
                    self.invalidate()
                    error = "PERMIT_INPUT_EXPIRED"
                else:
                    return
            if error is None and self._latest is not None:
                offered, self._latest = self._latest, None
                intent, guard = offered.intent, offered.guard
                if not self._current(guard):
                    self.dropped_count += 1
                    return
                if now >= intent.quality.valid_until_ns:
                    self.dropped_count += 1
                    error = "INPUT_EXPIRED_BEFORE_PERMIT"
                else:
                    try:
                        context, token = self.session.context()
                        if not self._current(guard):
                            self.dropped_count += 1
                            return
                        ttl_s = min(self.max_ttl_s, (intent.quality.valid_until_ns - now) / 1e9)
                        future = self.session.permit_async(context, token, ttl_s)
                        flight = _Flight(
                            future, intent, context, guard.stop_epoch, guard.dispatch_epoch
                        )
                        self._flight = flight
                        self.notify(
                            {
                                "kind": "permit_request",
                                "sequence": context.sequence,
                                "valid_until_ns": intent.quality.valid_until_ns,
                                "monotonic_ns": now,
                            }
                        )
                        register = flight
                    except Exception as exc:
                        error = "PERMIT_REQUEST:" + str(exc)[:160]
        # An already completed Future may invoke its callback synchronously.
        # Register after releasing this mutex so even that path publishes unlocked.
        if register is not None:
            register.future.add_done_callback(lambda f: self._complete(register, f))
        if error is not None:
            self.failure(error)

    def _complete(self, flight, future):
        error = None
        response = None
        with self._lock:
            if self._flight is not flight:
                self.dropped_count += 1
                return
            self._flight = None
            now = self.clock()
            valid = (
                not future.cancelled()
                and self._epoch == flight.dispatch_epoch
                and self.session.stop_epoch == flight.stop_epoch
                and self.session.identity()
                == (flight.context.session_id, flight.context.generation)
                and now < flight.intent.quality.valid_until_ns
            )
            if not valid:
                self.dropped_count += 1
                return
            try:
                response = future.result()
                if not response.accepted:
                    raise RuntimeError(response.error_code)
                # Do not compare against the globally current sequence: renewal
                # and newer observations may legitimately have advanced it.
                self._publishing = flight
            except Exception as exc:
                error = "PERMIT_REPLY:" + str(exc)[:160]
        if response is not None and error is None:
            try:
                # DDS may block. No input/STOP/dispatcher mutex is held here.
                # A publishing slot stays occupied until this call returns;
                # invalidation fences any late packet at coordinator/device.
                self.publish(flight.intent, flight.context, response.token)
                with self._lock:
                    self.sent_count += 1
                self.notify(
                    {
                        "kind": "jog_published",
                        "sequence": flight.context.sequence,
                        "input_epoch": flight.intent.quality.input_epoch,
                        "valid_until_ns": flight.intent.quality.valid_until_ns,
                        "linear_mm_s": flight.intent.linear_mm_s,
                        "rotation_deg_s": flight.intent.rotation_deg_s,
                        "control_policy": flight.intent.control_policy,
                        "servo_rpm": flight.intent.servo_rpm,
                        "monotonic_ns": self.clock(),
                    }
                )
            except Exception as exc:
                with self._lock:
                    if self._epoch == flight.dispatch_epoch:
                        error = "PUBLISH:" + str(exc)[:160]
            finally:
                with self._lock:
                    self._publishing = None
        if error is not None:
            self.failure(error)
