from types import SimpleNamespace
import numpy as np
import pytest

# Provide lightweight mock for standalone pytest environment
try:
    import builtin_interfaces.msg
    import sensor_msgs.msg
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import NavSatFix, NavSatStatus
except ImportError:
    import types
    import sys
    
    bi_mock = types.ModuleType('builtin_interfaces')
    bi_msg_mock = types.ModuleType('builtin_interfaces.msg')
    class Time:
        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec
    bi_msg_mock.Time = Time
    bi_mock.msg = bi_msg_mock
    sys.modules['builtin_interfaces'] = bi_mock
    sys.modules['builtin_interfaces.msg'] = bi_msg_mock

    sm_mock = types.ModuleType('sensor_msgs')
    sm_msg_mock = types.ModuleType('sensor_msgs.msg')
    class NavSatStatus:
        STATUS_NO_FIX = -1
        STATUS_FIX = 0
        SERVICE_GPS = 1
    class NavSatFix:
        COVARIANCE_TYPE_UNKNOWN = 0
        COVARIANCE_TYPE_APPROXIMATED = 1
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        COVARIANCE_TYPE_KNOWN = 3
        def __init__(self):
            self.header = SimpleNamespace(stamp=Time(), frame_id='')
            self.status = SimpleNamespace(status=0, service=1)
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0
            self.position_covariance = [0.0] * 9
            self.position_covariance_type = 0
    sm_msg_mock.NavSatStatus = NavSatStatus
    sm_msg_mock.NavSatFix = NavSatFix
    sm_mock.msg = sm_msg_mock
    sys.modules['sensor_msgs'] = sm_mock
    sys.modules['sensor_msgs.msg'] = sm_msg_mock

    # Mock other ROS interfaces needed by transform_node
    for mod in [
        'action_msgs', 'action_msgs.msg',
        'inspection_interfaces', 'inspection_interfaces.action',
        'nav2_msgs', 'nav2_msgs.action',
        'rcl_interfaces', 'rcl_interfaces.msg', 'rcl_interfaces.srv',
        'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg',
        'nav_msgs', 'nav_msgs.msg',
        'robots_dog_msgs', 'robots_dog_msgs.msg',
        'rclpy.action', 'rclpy.action.server', 'rclpy.callback_groups'
    ]:
        if mod not in sys.modules:
            m = types.ModuleType(mod)
            sys.modules[mod] = m
    sys.modules['action_msgs.msg'].GoalStatus = types.SimpleNamespace()
    sys.modules['inspection_interfaces.action'].SetGPSGoal = types.SimpleNamespace()
    sys.modules['nav2_msgs.action'].NavigateToPose = types.SimpleNamespace()
    sys.modules['geometry_msgs.msg'].TransformStamped = types.SimpleNamespace
    sys.modules['tf2_ros'].TransformBroadcaster = lambda *a, **kw: None
    sys.modules['tf2_ros'].StaticTransformBroadcaster = lambda *a, **kw: None
    sys.modules['rclpy.action'].ActionClient = lambda *a, **kw: None
    sys.modules['rclpy.action'].ActionServer = lambda *a, **kw: None
    sys.modules['rclpy.action.server'].CancelResponse = types.SimpleNamespace()
    sys.modules['rclpy.callback_groups'].ReentrantCallbackGroup = lambda *a, **kw: None
    sys.modules['rclpy.executors'].MultiThreadedExecutor = types.SimpleNamespace
    sys.modules['rcl_interfaces.msg'].Parameter = types.SimpleNamespace
    sys.modules['rcl_interfaces.msg'].ParameterType = types.SimpleNamespace
    sys.modules['rcl_interfaces.msg'].ParameterValue = types.SimpleNamespace
    sys.modules['rcl_interfaces.srv'].SetParameters = types.SimpleNamespace

    class MockUniRtkPvh:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=Time(), frame_id='gps')
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
    sys.modules['robots_dog_msgs.msg'].UniRtkPvh = MockUniRtkPvh

    ament_mock = types.ModuleType('ament_index_python')
    ament_pkg_mock = types.ModuleType('ament_index_python.packages')
    ament_pkg_mock.get_package_share_directory = lambda pkg: '/tmp'
    ament_mock.packages = ament_pkg_mock
    sys.modules['ament_index_python'] = ament_mock
    sys.modules['ament_index_python.packages'] = ament_pkg_mock

from gnss_driver.rtk import rtk_to_navsat_fix


def _message(status=0, lat_std=0.3, lon_std=0.4, hgt_std=0.8):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=Time(sec=1, nanosec=2)),
        bestnav=SimpleNamespace(
            p_sol_status=status,
            latitude_deg=30.0,
            longitude_deg=120.0,
            altitude_m=10.0,
            lat_std=lat_std,
            lon_std=lon_std,
            hgt_std=hgt_std,
        ),
    )


def test_rtk_solution_becomes_known_navsat_covariance():
    fix = rtk_to_navsat_fix(_message())
    assert fix is not None
    assert fix.header.frame_id == 'gps'
    assert fix.position_covariance[0] == pytest.approx(0.16)
    assert fix.position_covariance[4] == pytest.approx(0.09)
    assert fix.position_covariance[8] == pytest.approx(0.64)
    assert fix.position_covariance_type == fix.COVARIANCE_TYPE_DIAGONAL_KNOWN


def test_rtk_unsolved_position_is_rejected():
    assert rtk_to_navsat_fix(_message(status=1)) is None


def test_rtk_odometry_covariance_positive_semidefinite():
    """Verify that RTK odometry covariance matrix has no negative eigenvalues in RViz."""
    from gnss_driver.transform_node import TransformNode
    
    # Mock node and transform
    class DummyTransform:
        rotation = np.array([
            [0.996733243190237, -0.080764112819201],
            [0.080764112819201, 0.996733243190237],
        ])

    class DummyOdom:
        def __init__(self):
            self.pose = SimpleNamespace(
                covariance=[0.0] * 36
            )

    node = SimpleNamespace(transform=DummyTransform())
    
    # Test known covariance
    fix = rtk_to_navsat_fix(_message(status=0, lat_std=0.0163, lon_std=0.0252, hgt_std=0.0384))
    odom = DummyOdom()
    TransformNode._set_rtk_position_covariance(node, odom, fix)
    
    # Check no negative numbers on diagonal
    for i in [0, 7, 14, 21, 28, 35]:
        assert odom.pose.covariance[i] >= 0.0, f"Negative diagonal entry at {i}: {odom.pose.covariance[i]}"
    
    # Check exact symmetry of position XY
    assert odom.pose.covariance[1] == odom.pose.covariance[6]
    
    # 3D position covariance eigenvalues
    cov_mat = np.array(odom.pose.covariance).reshape(6, 6)
    pos_3d_eigs = np.linalg.eigvalsh(cov_mat[:3, :3])
    assert np.all(pos_3d_eigs >= -1e-12), f"Negative eigenvalue in 3D position: {pos_3d_eigs}"

    # Orientation sub-matrices checked by RViz
    pitch_cov = np.array([
        [cov_mat[3, 3], cov_mat[3, 5]],
        [cov_mat[5, 3], cov_mat[5, 5]],
    ])
    assert np.all(np.linalg.eigvalsh(pitch_cov) >= -1e-12)

    yaw_cov = cov_mat[3:5, 3:5]
    assert np.all(np.linalg.eigvalsh(yaw_cov) >= -1e-12)

    # Test unknown covariance case
    fix_unknown = SimpleNamespace(position_covariance_type=0)
    odom_unknown = DummyOdom()
    TransformNode._set_rtk_position_covariance(node, odom_unknown, fix_unknown)
    assert not any(v < 0.0 for v in odom_unknown.pose.covariance)

