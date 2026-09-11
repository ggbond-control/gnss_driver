"""Test BaseGnssNode and BaseSerialGnssNode architecture and class hierarchy."""
import pytest

pytest.importorskip('rclpy')
from gnss_driver.nodes.base_gnss_node import BaseGnssNode
from gnss_driver.nodes.base_serial_node import BaseSerialGnssNode
from gnss_driver.nodes.g60_node import G60DriverNode
from gnss_driver.nodes.g90_node import G90DriverNode
from gnss_driver.nodes.d1m_bridge_node import D1MBridgeNode


def test_class_hierarchy():
    """Verify clean inheritance: BaseGnssNode -> BaseSerialGnssNode -> G60/G90, and BaseGnssNode -> D1M."""
    assert issubclass(BaseSerialGnssNode, BaseGnssNode)
    assert issubclass(G60DriverNode, BaseSerialGnssNode)
    assert issubclass(G90DriverNode, BaseSerialGnssNode)
    assert issubclass(D1MBridgeNode, BaseGnssNode)
    # D1M is a pure ROS topic bridge, NOT a serial node
    assert not issubclass(D1MBridgeNode, BaseSerialGnssNode)


def test_heading_gnss_conversion():
    """Verify Geographic Heading (0 North, clockwise) to ROS ENU Yaw (0 East, CCW) quaternion."""
    import math
    import rclpy
    rclpy.init()
    try:
        node = BaseGnssNode(node_name='test_heading_node', default_is_rtk=True)
        assert node.heading_gnss_topic == '/heading_gnss'
        # 0 deg Geographic North -> ROS ENU Yaw = +pi/2 (90 deg)
        imu = node.publish_heading_gnss(heading_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
        assert imu is not None
        assert abs(imu.orientation.x) < 1e-6
        assert abs(imu.orientation.y) < 1e-6
        assert abs(imu.orientation.z - math.sin(math.pi / 4)) < 1e-6
        assert abs(imu.orientation.w - math.cos(math.pi / 4)) < 1e-6
        assert imu.angular_velocity_covariance[0] == -1.0
        assert imu.linear_acceleration_covariance[0] == -1.0

        # 90 deg Geographic East -> ROS ENU Yaw = 0 deg
        imu_east = node.publish_heading_gnss(heading_deg=90.0, pitch_deg=0.0, roll_deg=0.0)
        assert imu_east is not None
        assert abs(imu_east.orientation.z) < 1e-6
        assert abs(imu_east.orientation.w - 1.0) < 1e-6
        node.destroy_node()
    finally:
        rclpy.shutdown()
