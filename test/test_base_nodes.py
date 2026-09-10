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
