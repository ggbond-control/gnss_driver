"""Test BaseGnssNode and BaseSerialGnssNode architecture and class hierarchy."""
import math
import sys
import types
from unittest.mock import MagicMock
import pytest


def setup_ros_mocks():
    """Inject lightweight ROS2 mocks into sys.modules if real rclpy is not installed."""
    try:
        import rclpy
        import sensor_msgs.msg
        import robots_dog_msgs.msg
        return
    except ImportError:
        pass

    class MockParameter:
        def __init__(self, value):
            self.value = value

    class MockTimeMsg:
        def __init__(self, sec=1789114102, nanosec=517736867):
            self.sec = sec
            self.nanosec = nanosec

    class MockClock:
        def now(self):
            class _Now:
                nanoseconds = 1789114102517736867
                def to_msg(self):
                    return MockTimeMsg()
            return _Now()

    class MockNode:
        def __init__(self, node_name='mock_node', **kwargs):
            self._node_name = node_name
            self._logger = MagicMock()
            self._clock = MockClock()
        def declare_parameter(self, name, default_value):
            return MockParameter(default_value)
        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            pub.topic = topic
            pub.msg_type = msg_type
            pub.publish = MagicMock()
            return pub
        def create_timer(self, period, callback):
            return MagicMock()
        def create_subscription(self, msg_type, topic, callback, qos):
            return MagicMock()
        def get_logger(self):
            return self._logger
        def get_clock(self):
            return self._clock
        def destroy_node(self):
            pass

    rclpy_mock = types.ModuleType('rclpy')
    rclpy_mock.__path__ = []
    rclpy_mock.init = MagicMock()
    rclpy_mock.shutdown = MagicMock()
    rclpy_mock.ok = MagicMock(return_value=True)

    node_module = types.ModuleType('rclpy.node')
    node_module.Node = MockNode
    rclpy_mock.node = node_module

    executors_mock = types.ModuleType('rclpy.executors')
    class ExternalShutdownException(Exception): pass
    executors_mock.ExternalShutdownException = ExternalShutdownException
    rclpy_mock.executors = executors_mock

    qos_mock = types.ModuleType('rclpy.qos')
    qos_mock.qos_profile_sensor_data = MagicMock()
    rclpy_mock.qos = qos_mock

    # Mock sensor_msgs
    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    
    class NavSatStatus:
        STATUS_NO_FIX = -1
        STATUS_FIX = 0
        STATUS_SBAS_FIX = 1
        STATUS_GBAS_FIX = 2
        SERVICE_GPS = 1
        SERVICE_GLONASS = 2
        SERVICE_COMPASS = 4
        SERVICE_GALILEO = 8

    class NavSatFix:
        COVARIANCE_TYPE_UNKNOWN = 0
        COVARIANCE_TYPE_APPROXIMATED = 1
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        COVARIANCE_TYPE_KNOWN = 3
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.status = types.SimpleNamespace(status=0, service=1)
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0
            self.position_covariance = [0.0] * 9
            self.position_covariance_type = 0

    class Imu:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            self.orientation_covariance = [0.0] * 9
            self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular_velocity_covariance = [0.0] * 9
            self.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.linear_acceleration_covariance = [0.0] * 9

    class TimeReference:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.time_ref = MockTimeMsg()
            self.source = 'gps'

    sensor_msgs_msg.NavSatStatus = NavSatStatus
    sensor_msgs_msg.NavSatFix = NavSatFix
    sensor_msgs_msg.Imu = Imu
    sensor_msgs_msg.TimeReference = TimeReference
    sensor_msgs.msg = sensor_msgs_msg

    # Mock robots_dog_msgs
    robots_dog_msgs = types.ModuleType('robots_dog_msgs')
    robots_dog_msgs_msg = types.ModuleType('robots_dog_msgs.msg')
    class UniRtkPvh:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.heading = types.SimpleNamespace(
                header=self.header, utc_time_s=0.0, sol_status=0, heading_type=0,
                base_line=0.0, heading_deg=0.0, pitch_deg=0.0, heading_std=0.0, pitch_std=0.0,
                svs_num=0, soln_svs_num=0
            )
            self.bestnav = types.SimpleNamespace(
                header=self.header, utc_time_s=0.0, p_sol_status=0, pos_type=0,
                latitude_deg=0.0, longitude_deg=0.0, altitude_m=0.0, undulation=0.0,
                lat_std=0.0, lon_std=0.0, hgt_std=0.0, svs_num=0, soln_svs_num=0,
                diff_age_s=0.0, sol_age_s=0.0, hor_spd=0.0, trk_gnd=0.0, ver_spd=0.0,
                ver_spd_std=0.0, hor_spd_std=0.0, v_sol_status=0, vel_type=0
            )
    robots_dog_msgs_msg.UniRtkPvh = UniRtkPvh
    robots_dog_msgs.msg = robots_dog_msgs_msg

    # Mock nav_msgs, geometry_msgs, std_msgs, tf_transformations
    nav_msgs = types.ModuleType('nav_msgs')
    nav_msgs_msg = types.ModuleType('nav_msgs.msg')
    class Odometry:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.child_frame_id = 'base_link'
            self.pose = types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            ))
            self.twist = types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            ))
    nav_msgs_msg.Odometry = Odometry
    nav_msgs.msg = nav_msgs_msg

    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')
    class QuaternionStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.quaternion = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    class TwistStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=MockTimeMsg(), frame_id='gps')
            self.twist = types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            )
    geometry_msgs_msg.QuaternionStamped = QuaternionStamped
    geometry_msgs_msg.TwistStamped = TwistStamped
    geometry_msgs.msg = geometry_msgs_msg

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    class String:
        def __init__(self):
            self.data = ''
    std_msgs_msg.String = String
    std_msgs.msg = std_msgs_msg

    tf_trans = types.ModuleType('tf_transformations')
    def quaternion_from_euler(ai, aj, ak):
        cy = math.cos(ak * 0.5)
        sy = math.sin(ak * 0.5)
        cp = math.cos(aj * 0.5)
        sp = math.sin(aj * 0.5)
        cr = math.cos(ai * 0.5)
        sr = math.sin(ai * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * cp * sy,
        )
    tf_trans.quaternion_from_euler = quaternion_from_euler

    sys.modules['rclpy'] = rclpy_mock
    sys.modules['rclpy.node'] = node_module
    sys.modules['rclpy.executors'] = executors_mock
    sys.modules['rclpy.qos'] = qos_mock
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg
    sys.modules['robots_dog_msgs'] = robots_dog_msgs
    sys.modules['robots_dog_msgs.msg'] = robots_dog_msgs_msg
    sys.modules['nav_msgs'] = nav_msgs
    sys.modules['nav_msgs.msg'] = nav_msgs_msg
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg
    sys.modules['tf_transformations'] = tf_trans


# Ensure mocks are installed before importing nodes
setup_ros_mocks()

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
    assert not issubclass(D1MBridgeNode, BaseSerialGnssNode)


def test_heading_gnss_conversion():
    """Verify Geographic Heading (0 North, clockwise) to ROS ENU Yaw (0 East, CCW) quaternion."""
    import rclpy
    rclpy.init()
    try:
        node = BaseGnssNode(node_name='test_heading_node', default_is_rtk=True)
        assert node.heading_gnss_topic == '/heading_gnss'
        assert callable(node.publish_heading_gnss)
        assert callable(node.publish_fix)
        assert callable(node.publish_rtk)

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


def test_node_instantiation_and_publish_callability():
    """Verify nodes instantiate without AttributeError and publish methods are callable (no bool shadowing)."""
    import rclpy
    rclpy.init()
    try:
        # 1. BaseGnssNode
        base = BaseGnssNode(node_name='test_base', default_is_rtk=True)
        assert hasattr(base, 'enable_heading_gnss')
        assert base.enable_heading_gnss is True
        assert callable(base.publish_heading_gnss)
        assert callable(base.publish_fix)
        assert callable(base.publish_rtk)
        base.destroy_node()

        # 2. G90DriverNode
        g90 = G90DriverNode()
        assert hasattr(g90, 'rtk_topic')
        assert hasattr(g90, 'fix_topic')
        assert hasattr(g90, 'heading_gnss_topic')
        assert hasattr(g90, 'enable_heading_gnss')
        assert g90.rtk_topic == '/rtk_pvh'
        assert g90.heading_gnss_topic == '/heading_gnss'
        assert callable(g90.publish_heading_gnss)  # Crucial regression test for 'bool' object is not callable
        assert callable(g90.publish_fix)
        assert callable(g90.publish_rtk)

        # Simulate receiving PVTSLNA and GNHPR sentences and calling _publish()
        # Ensure publish_heading_gnss executes cleanly without TypeError
        g90.latest = {
            'sol_status': 0, 'pos_type': 50,
            'lat': 30.28, 'lon': 120.0, 'hgt': 10.0,
            'undulation': 0.0, 'lat_std': 0.01, 'lon_std': 0.01, 'hgt_std': 0.02,
            'diff_age_s': 1.0, 'sol_age_s': 1.0, 'svs_num': 20, 'soln_svs_num': 18,
            'utc_time_s': 1000.0,
        }
        g90.heading = types.SimpleNamespace(heading_deg=180.0, pitch_deg=-2.0, roll_deg=1.0)
        # This call would trigger TypeError: 'bool' object is not callable if publish_heading_gnss was shadowed
        g90._publish()

        g90.destroy_node()

        # 3. D1MBridgeNode
        d1m = D1MBridgeNode()
        assert hasattr(d1m, 'rtk_topic')
        assert hasattr(d1m, 'heading_gnss_topic')
        assert hasattr(d1m, 'enable_heading_gnss')
        assert callable(d1m.publish_heading_gnss)
        assert callable(d1m.publish_fix)
        d1m.destroy_node()

        # 4. G60DriverNode
        g60 = G60DriverNode()
        assert callable(g60.publish_fix)
        g60.destroy_node()

    finally:
        rclpy.shutdown()


def _make_nmea(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"${body}*{checksum:02X}"


def _make_extended(body):
    from gnss_driver.adapters import g90_unicore
    crc = 0
    for byte in body.encode('ascii'):
        crc = ((crc >> 8) ^ g90_unicore._table[(crc ^ byte) & 0xff]) & 0xffffffff
    return f"#{body}*{crc:08X}"


def test_g90_handle_line_end_to_end():
    """Verify G90DriverNode.handle_line correctly parses lines and publishes without TypeError."""
    import rclpy
    rclpy.init()
    try:
        g90 = G90DriverNode()
        assert g90.heading_gnss_pub is not None

        # 1. Feed Heading sentence ($GNHPR) with valid checksum
        hpr_line = _make_nmea('GNHPR,173000.00,170.393,-4.086,0.000,1,0')
        g90.handle_line(hpr_line)
        assert g90.heading is not None
        assert abs(g90.heading.heading_deg - 170.393) < 1e-3

        # 2. Feed PVTSLNA sentence (Triggers _publish -> publish_heading_gnss)
        pvtslna_body = (
            'PVTSLNA,COM1,0,0,FINESTEERING,2300,345600.5,SOL_COMPUTED,NARROW_INT,0,'
            '7.5,30.2,120.3,0.3,0.1,0.2,18,12'
        )
        g90.handle_line(_make_extended(pvtslna_body))

        # Verify rtk_pub received a message
        assert g90.rtk_pub.publish.called
        published_rtk = g90.rtk_pub.publish.call_args[0][0]
        assert abs(published_rtk.heading.heading_deg - 170.393) < 1e-3
        assert published_rtk.bestnav.pos_type == 50

        # Verify heading_gnss_pub received standard Imu message
        assert g90.heading_gnss_pub.publish.called
        published_imu = g90.heading_gnss_pub.publish.call_args[0][0]
        assert abs(published_imu.orientation.w) > 0.0 or abs(published_imu.orientation.z) > 0.0

        # 3. Feed GGA sentence
        gga_line = _make_nmea('GNGGA,173000.00,3017.3356,N,11958.8585,E,4,18,1.0,6.67,M,7.65,M,1.0,0000')
        g90.handle_line(gga_line)
        assert g90.gga is not None

        g90.destroy_node()
    finally:
        rclpy.shutdown()


def test_d1m_bridge_on_message_end_to_end():
    """Verify D1MBridgeNode._on_rtk correctly handles dual-antenna RTK messages."""
    import rclpy
    from robots_dog_msgs.msg import UniRtkPvh
    rclpy.init()
    try:
        d1m = D1MBridgeNode()
        assert d1m.heading_gnss_pub is not None

        msg = UniRtkPvh()
        msg.heading.sol_status = 0
        msg.heading.heading_deg = 45.0
        msg.heading.pitch_deg = 1.5
        msg.heading.heading_std = 0.2
        msg.bestnav.latitude_deg = 30.28
        msg.bestnav.longitude_deg = 120.0
        msg.bestnav.altitude_m = 10.0
        msg.bestnav.lat_std = 0.02
        msg.bestnav.lon_std = 0.02
        msg.bestnav.hgt_std = 0.05

        d1m._on_rtk(msg)

        assert d1m.fix_pub.publish.called
        assert d1m.heading_gnss_pub.publish.called
        imu = d1m.heading_gnss_pub.publish.call_args[0][0]
        assert math.isfinite(imu.orientation.w)

        d1m.destroy_node()
    finally:
        rclpy.shutdown()

