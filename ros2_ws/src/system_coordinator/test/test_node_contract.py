"""Use the actual Jazzy Node to catch collisions with its instance attributes."""

import rclpy
from gca_system_interfaces.msg import GcaState
from system_coordinator.node import SystemCoordinator


def test_coordinator_builds_typed_lease_request_without_hardware():
    rclpy.init(args=["--ros-args", "-p", "coordinator_key:=test-coordinator-capability"])
    node = None
    try:
        node = SystemCoordinator()
        source = node._authority.claim("contract_test", "AUTO", 1.0)
        request = node._lease_request(source, 0.5)
        assert request.context.session_id == source.session_id
        assert request.context.generation == source.generation
        assert request.lease_s == 0.5
        assert request.coordinator_key == "test-coordinator-capability"
        # A terminal response must not subsequently be overwritten by older
        # DDS telemetry from the same process, or by a retired process boot.
        assert node._update_state(GcaState(boot_id="first", state_seq=20, state="READY"))
        assert not node._update_state(GcaState(boot_id="first", state_seq=19, state="EXECUTING"))
        assert node._state.state == "READY"
        assert not node._update_state(GcaState(boot_id="first", state_seq=20, state="EXECUTING"))
        assert node._update_state(
            GcaState(
                boot_id="first", state_seq=20, state="READY", feedback_age_s=0.3, linear_valid=False
            )
        )
        assert node._state.feedback_age_s == 0.3
        assert not node._state.linear_valid
        assert node._update_state(GcaState(boot_id="second", state_seq=1, state="DISARMED"))
        assert not node._update_state(GcaState(boot_id="first", state_seq=50, state="READY"))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
