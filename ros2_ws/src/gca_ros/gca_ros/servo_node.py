"""Servo-only lifecycle host sharing the established management and STOP machinery."""

import math
import time

from gca.domain.manual import QualityTracker
from gca.domain.models import Lease
from gca.domain.servo import FeedRuntimeState
from gca.runtime.feed_axis import FeedAxisRuntime
from gca.servo_config import load_servo_settings
from gca_system_interfaces.msg import ServoJogCommand
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
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
        self.declare_parameter("control_mode", "", ParameterDescriptor(read_only=True))
        self.declare_parameter("simulation_slip_ratio", 1.0, ParameterDescriptor(read_only=True))
        return set()

    def _read_config(self):
        path = self.get_parameter("servo_config_file").value
        if not path:
            raise ValueError("servo_config_file must name an exported JSON snapshot")
        config = load_servo_settings(path)
        requested_mode = self.get_parameter("control_mode").value
        if requested_mode and requested_mode != config.control_mode:
            raise ValueError("control_mode must match the validated servo snapshot")
        self._control_mode = config.control_mode
        slip = self.get_parameter("simulation_slip_ratio").value
        if not math.isfinite(slip) or not 0.0 < slip <= 1.0:
            raise ValueError("simulation_slip_ratio must be finite in (0, 1]")
        if (config.backend != "simulation" or config.control_mode != "position") and slip != 1.0:
            raise ValueError("simulation_slip_ratio requires position simulation")
        return config

    def _create_runtime(self):
        if self._control_mode == "position":
            from gca_position.assembly import create_position_components

            drive, position = create_position_components(
                self._config,
                slip_ratio=self.get_parameter("simulation_slip_ratio").value,
            )
            return FeedAxisRuntime(
                self._config, notify=self._guard.trigger, drive=drive, position=position
            )
        return FeedAxisRuntime(self._config, notify=self._guard.trigger)

    def _create_quality_tracker(self):
        return QualityTracker(
            backend=self._config.backend,
            max_age_s=self._config.manual_input_max_age_s,
            capacity=self._config.history_capacity,
            allow_joydev_hold=self._config.manual_hold_enabled,
        )

    def _snapshot(self):
        if self.runtime is not None:
            return self.runtime.snapshot()
        return FeedRuntimeState(control_policy=self._config.control_policy)

    def _validate_execution_policy(self, policy):
        if policy != self._config.control_policy:
            raise AccessError("CONTROL_MODE_MISMATCH")

    def _jog(self, request):
        # Old double-axis messages are never interpreted as servo shaft velocity.
        return

    def _servo_jog(self, request):
        if self._control_mode != "manual":
            return
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
        if self._control_mode == "manual" and self._config.backend == "hardware":
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
        if self._control_mode != "position":
            return self._deny(request.context.command_id, "CONTROL_MODE_MISMATCH")
        if (
            request.target.axis_mask != 1
            or request.target.mode not in (1, 2)
            or request.target.rotation_rad != 0.0
            or request.end_policy != 1
        ):
            return self._deny(request.context.command_id, "POSITION_LINEAR_STOP_REQUIRED")
        return super()._goal(request)

    def _capabilities(self, request, response):
        response.backend = self._config.backend
        response.simulation = self._config.backend == "simulation"
        response.control_profile = "servo"
        response.servo_config_sha256 = self._config.sha256
        ready = self._snapshot().configured
        response.can_servo_velocity = ready and self._control_mode == "manual"
        response.can_measure_servo = ready
        response.can_stop = ready
        response.max_servo_velocity_rad_s = self._config.max_rpm * 2.0 * math.pi / 60.0
        response.unverified_capabilities = []
        if self._control_mode == "position":
            response.can_linear_motion = ready
            response.can_measure_linear = ready
            response.limits.max_linear_m_s = self._config.position.motion.max_linear_mm_s / 1000.0
            response.limits.max_linear_m_s2 = self._config.position.motion.max_linear_mm_s2 / 1000.0
        else:
            response.unverified_capabilities.append("EXTERNAL_ENCODER_POSITION_UNSUPPORTED")
        if not response.simulation:
            response.unverified_capabilities.append("SERVO_STOP_HIL")
            response.unverified_capabilities.append(
                "POSITION_ENCODER_HIL"
                if self._control_mode == "position"
                else "WIRELESS_INPUT_QUALIFICATION"
            )
        return response

    def _apply_config(self, request, response):
        response.accepted = False
        response.error_code = "SERVO_SNAPSHOT_RECONFIGURE_REQUIRED"
        response.message = "STOP, DISARM and cleanup before configuring an exported servo snapshot"
        return response

    def _validate_parameters(self, parameters):
        result = super()._validate_parameters(parameters)
        if any(
            p.name in ("servo_config_file", "control_mode", "simulation_slip_ratio")
            for p in parameters
        ):
            return SetParametersResult(successful=False, reason="Restart required")
        return result


def main(args=None):
    run_node(ServoControlNode, args)
