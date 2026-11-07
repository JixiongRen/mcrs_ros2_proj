"""Servo-only lifecycle host sharing the established management and STOP machinery."""

import math
import time

from gca.domain.manual import QualityTracker
from gca.domain.models import Lease
from gca.domain.servo import MANUAL_SERVO_VELOCITY, FeedRuntimeState
from gca.runtime.feed_axis import FeedAxisRuntime
from gca.servo_config import load_servo_settings
from gca_system_interfaces.msg import ServoJogCommand
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .conversion import quality_from_ros, servo_jog_from_ros
from .node import GcaControlNode, run_node
from .security import AccessError


class ServoControlNode(GcaControlNode):
    def __init__(self):
        super().__init__()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ServoJogCommand,
            "/gca/servo_jog",
            self._servo_jog,
            qos,
            callback_group=self.action_group,
        )

    def _declare_runtime_parameters(self):
        self.declare_parameter("servo_config_file", "")
        return set()

    def _read_config(self):
        path = self.get_parameter("servo_config_file").value
        if not path:
            raise ValueError("servo_config_file must name an exported JSON snapshot")
        return load_servo_settings(path)

    def _create_runtime(self):
        return FeedAxisRuntime(self._config, notify=self._guard.trigger)

    def _create_quality_tracker(self):
        return QualityTracker(
            backend=self._config.backend,
            max_age_s=self._config.manual_input_max_age_s,
            capacity=self._config.history_capacity,
            allow_joydev_hold=self._config.manual_hold_enabled,
        )

    def _snapshot(self):
        return self.runtime.snapshot() if self.runtime is not None else FeedRuntimeState()

    def _validate_execution_policy(self, policy):
        if policy == "POSITION_EXTERNAL_ENCODER":
            raise AccessError("EXTERNAL_ENCODER_POSITION_UNSUPPORTED")
        if policy != MANUAL_SERVO_VELOCITY:
            raise AccessError("CONTROL_POLICY_UNSUPPORTED")

    def _jog(self, request):
        # Old double-axis messages are never interpreted as servo shaft velocity.
        return

    def _servo_jog(self, request):
        if request.deadman:
            source = request.quality.source_kind
            if source == "JOYDEV_HELD_ARM":
                return
            if self._config.manual_hold_enabled and source != "JOYDEV_HELD_STATE":
                return
            if self._config.backend == "hardware" and not self._config.manual_hold_enabled:
                if source != "QUALIFIED_STATE_CONTRACT":
                    return
        self._dispatch_jog(request, servo_jog_from_ros)

    def _arm_runtime(self, grant, request, expected_serial):
        input_deadline_ns = 0
        if self._config.backend == "hardware":
            if not request.has_input_quality:
                raise AccessError("HARDWARE_ARM_INPUT_QUALITY_REQUIRED")
            quality = quality_from_ros(request.input_quality)
            expected_kind = (
                "JOYDEV_HELD_ARM"
                if self._config.manual_hold_enabled
                else "QUALIFIED_STATE_CONTRACT"
            )
            if quality.source_kind != expected_kind:
                raise AccessError("HARDWARE_ARM_INPUT_NOT_QUALIFIED")
            input_deadline_ns = self._quality.validate(
                quality,
                time.monotonic_ns(),
                session_id=grant.session_id,
                generation=grant.generation,
            )
        return self.runtime.arm(
            Lease(
                grant.session_id,
                grant.generation,
                grant.sequence,
                max(0.0, grant.deadline - time.monotonic()),
                grant.token,
                control_policy=grant.control_policy,
            ),
            expected_stop_serial=expected_serial,
            input_deadline_ns=input_deadline_ns,
        )

    def _goal(self, request):
        return self._deny(request.context.command_id, "EXTERNAL_ENCODER_POSITION_UNSUPPORTED")

    def _capabilities(self, request, response):
        response.backend = self._config.backend
        response.simulation = self._config.backend == "simulation"
        response.control_profile = "servo"
        response.servo_config_sha256 = self._config.sha256
        ready = self._snapshot().configured
        response.can_servo_velocity = ready
        response.can_measure_servo = ready
        response.can_stop = ready
        response.max_servo_velocity_rad_s = self._config.max_rpm * 2.0 * math.pi / 60.0
        response.unverified_capabilities = ["EXTERNAL_ENCODER_POSITION_UNSUPPORTED"]
        if not response.simulation:
            response.unverified_capabilities += ["WIRELESS_INPUT_QUALIFICATION", "SERVO_STOP_HIL"]
        return response

    def _apply_config(self, request, response):
        response.accepted = False
        response.error_code = "SERVO_SNAPSHOT_RECONFIGURE_REQUIRED"
        response.message = "STOP, DISARM and cleanup before configuring an exported servo snapshot"
        return response

    def _validate_parameters(self, parameters):
        result = super()._validate_parameters(parameters)
        if any(p.name == "servo_config_file" for p in parameters):
            return SetParametersResult(successful=False, reason="Restart required")
        return result


def main(args=None):
    run_node(ServoControlNode, args)
