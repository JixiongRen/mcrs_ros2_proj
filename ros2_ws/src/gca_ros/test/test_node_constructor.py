"""Exercise installed Jazzy constructors without configuring any device resources."""

import pytest
import rclpy
from gca.domain.models import DeviceError
from gca_ros.node import GcaControlNode
from rclpy.parameter import Parameter


def test_lifecycle_host_constructs_without_device_io():
    rclpy.init(args=["--ros-args", "-p", "coordinator_key:=test-constructor-capability"])
    node = None
    try:
        node = GcaControlNode()
        assert node.runtime is None
        assert node.management_group is node.default_callback_group
        assert not node._active
        with pytest.raises(DeviceError) as busy:
            GcaControlNode()
        assert busy.value.code == "DEVICE_BUSY"
        node.on_cleanup(None)
        # Cleanup retains public ROS endpoints, therefore retains exclusivity.
        with pytest.raises(DeviceError) as busy:
            GcaControlNode()
        assert busy.value.code == "DEVICE_BUSY"
        mixed = node._validate_parameters(
            [Parameter("kp", value=2.0), Parameter("coordinator_key", value="replacement")]
        )
        assert not mixed.successful and "Restart" in mixed.reason
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
