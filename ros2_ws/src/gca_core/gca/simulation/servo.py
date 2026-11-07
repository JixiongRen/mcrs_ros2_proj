"""Native shaft plant driven only by accepted output; no encoder/stepper objects."""

from __future__ import annotations

import math
import time
import uuid
from contextlib import nullcontext

from gca.devices.common import DeviceError
from gca.devices.servo_types import ServoShaftSample
from gca.domain.models import DeviceIdentity


class SimulatedVelocityDrive:
    """Finite-response velocity plant; simulation parameters are not hardware data.

    Every operation advances the plant from the previous accepted register value.
    Feedback never copies a control request.  A first-order motor response and
    the configured profile acceleration limit affect the measured speed/angle.
    """

    def __init__(self, config, *, clock=time.monotonic_ns):
        self.settings, self.config, self.clock = config, config.device, clock
        self.identity = None
        self.opened = self.configured = self.enabled = False
        self.fail_read = self.fail_write = False
        self._goal_rpm = self._velocity_rpm = self._position_count = 0.0
        self._last_ns = self._bus_ns = 0
        self._seq = 0
        self._watchdog_raw = 0
        self.hardware_error = 0
        self.configuration_readback = {}

    def _check(self, deadline_ns=None):
        if not self.opened:
            raise DeviceError("DEVICE_CLOSED", "simulated servo is closed")
        if deadline_ns is not None and self.clock() >= deadline_ns:
            raise DeviceError("DEVICE_TIMEOUT", "simulated servo deadline expired")

    def _advance(self):
        now = self.clock()
        if self._last_ns == 0:
            self._last_ns = now
        dt = max(0.0, (now - self._last_ns) / 1e9)
        self._last_ns = now
        expired = (
            self.enabled
            and self._watchdog_raw > 0
            and now - self._bus_ns > self._watchdog_raw * 20_000_000
        )
        if expired:
            self._watchdog_raw = -1
        target = self._goal_rpm if self.enabled and self._watchdog_raw != -1 else 0.0
        previous = self._velocity_rpm
        response = target + (previous - target) * math.exp(-dt / 0.08)
        if self.enabled and self._watchdog_raw != -1:
            # Manufacturer acceleration unit is rev/min^2, hence /60 -> rpm/s.
            budget = self.config.profile_acceleration_raw * 214.577 / 60 * dt
            response = previous + max(-budget, min(budget, response - previous))
        self._velocity_rpm = response
        self._position_count += (previous + response) * 0.5 * dt / 60 * 4096

    def _instruction(self, deadline_ns=None):
        self._check(deadline_ns)
        self._advance()
        self._bus_ns = self.clock()

    def open_identified(self):
        if self.opened:
            raise DeviceError("ALREADY_OPEN")
        self.opened = True
        self._last_ns = self._bus_ns = self.clock()
        self.identity = DeviceIdentity(
            "SIM_NATIVE_VELOCITY_SERVO",
            str(uuid.uuid4()),
            "simulation",
            "1",
            ("servo_velocity", "servo_shaft_feedback", "torque_disable"),
        )
        self._seq = 0
        return self.identity

    def configure_velocity_mode(self):
        self._instruction()
        self.configured = self.enabled = False
        if self._watchdog_raw == -1:
            raise DeviceError("WATCHDOG_LATCHED")
        if self.hardware_error:
            raise DeviceError("HARDWARE_FAULT")
        self._goal_rpm = 0.0
        self._watchdog_raw = self.config.watchdog_raw
        self.configuration_readback = {
            "mode": 1,
            "drive_mode": 0,
            "velocity_limit_raw": self.config.max_velocity_raw,
            "profile_acceleration_raw": self.config.profile_acceleration_raw,
            "watchdog_raw": self.config.watchdog_raw,
            "goal_velocity_raw": 0,
            "torque_enabled": False,
        }
        self.configured = True

    def set_torque(self, enabled, deadline_ns=None, *, enable_guard=None):
        self._instruction(deadline_ns)
        if type(enabled) is not bool:
            raise DeviceError("INVALID_ARGUMENT")
        if enabled:
            if not self.configured:
                raise DeviceError("NOT_CONFIGURED")
            if self._watchdog_raw == -1 or self.hardware_error:
                raise DeviceError(
                    "WATCHDOG_LATCHED" if self._watchdog_raw == -1 else "HARDWARE_FAULT"
                )
            self._goal_rpm = 0.0
            if enable_guard is not None and not enable_guard():
                raise DeviceError("STOP_SUPERSEDED")
        self.enabled = enabled
        if enabled and enable_guard is not None and not enable_guard():
            self.enabled = False
            self._goal_rpm = 0.0
            raise DeviceError("STOP_SUPERSEDED")

    def write_velocity_rpm(self, rpm, deadline_ns, *, write_guard=None):
        self._instruction(deadline_ns)
        if self.fail_write:
            raise DeviceError("SDK_ERROR", "injected simulated write failure")
        if not self.configured or not self.enabled:
            raise DeviceError("NOT_ARMED")
        if self._watchdog_raw == -1:
            raise DeviceError("WATCHDOG_LATCHED")
        if type(rpm) not in (int, float) or not math.isfinite(rpm):
            raise DeviceError("INVALID_ARGUMENT")
        if abs(rpm) > self.config.max_rpm:
            raise DeviceError("LIMIT_EXCEEDED")
        with (write_guard or nullcontext)():
            self._check(deadline_ns)
            self._goal_rpm = (
                round(rpm / self.config.velocity_unit_rpm) * self.config.velocity_unit_rpm
            )
        return self._goal_rpm

    def read_shaft_state(self, deadline_ns):
        started = self.clock()
        self._instruction(deadline_ns)
        if self.fail_read:
            raise DeviceError("SDK_ERROR", "injected simulated read failure")
        self._seq += 1
        error = (
            "HARDWARE_FAULT"
            if self.hardware_error
            else "WATCHDOG_LATCHED"
            if self._watchdog_raw == -1
            else ""
        )
        return ServoShaftSample(
            round(self._position_count),
            self._velocity_rpm,
            self.enabled,
            self._seq,
            self.clock(),
            self.identity.device_epoch,
            True,
            error,
            started,
            self.hardware_error,
            self._watchdog_raw,
        )

    def stop(self, deadline_ns):
        self._instruction(deadline_ns)
        if self.fail_write:
            raise DeviceError("SDK_ERROR", "injected simulated zero failure")
        if self._watchdog_raw == -1:
            raise DeviceError(
                "WATCHDOG_LATCHED", "goal register is read-only until explicit recovery"
            )
        self._goal_rpm = 0.0

    def recover_watchdog(self):
        self._instruction()
        if self.enabled:
            raise DeviceError("NOT_DISARMED")
        if self.hardware_error:
            raise DeviceError("HARDWARE_FAULT")
        self.configured = False
        self._goal_rpm = 0.0
        self._watchdog_raw = self.config.watchdog_raw

    def close(self):
        if not self.opened:
            return
        failure = None
        try:
            self.stop(self.clock() + int(self.config.transaction_timeout_s * 1e9))
        except DeviceError as exc:
            failure = exc
        finally:
            self.set_torque(False)
            self.opened = self.configured = False
        if failure:
            raise failure
