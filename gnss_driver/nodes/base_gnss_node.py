"""Base GNSS Node providing common GNSS data models, parameters and ROS publishing."""
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from robots_dog_msgs.msg import UniRtkPvh


class BaseGnssNode(Node):
    """Abstract base ROS node for all GNSS devices and bridges.
    
    Encapsulates core GNSS properties:
    - Coordinate frame_id (default: 'gps')
    - Output fix_topic (default: '/fix')
    - RTK capability flag (is_rtk) and rtk_topic (default: '/rtk_pvh')
    - Standardized NavSatFix and UniRtkPvh publishing methods
    """

    def __init__(self, node_name: str = 'gnss_device', default_is_rtk: bool = False):
        super().__init__(node_name)
        self.frame_id = self.declare_parameter('frame_id', 'gps').value
        self.fix_topic = self.declare_parameter('fix_topic', '/fix').value
        self.is_rtk = self.declare_parameter('is_rtk', default_is_rtk).value
        self.rtk_topic = self.declare_parameter('rtk_topic', '/rtk_pvh').value

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.rtk_pub = (
            self.create_publisher(UniRtkPvh, self.rtk_topic, 10)
            if self.is_rtk and self.rtk_topic else None
        )

    def publish_fix(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        status: int = NavSatStatus.STATUS_FIX,
        service: int = NavSatStatus.SERVICE_GPS,
        std_devs: Optional[Tuple[float, float, float]] = None,
        stamp=None,
    ) -> Optional[NavSatFix]:
        """Assemble and publish a standardized sensor_msgs/NavSatFix message.
        
        Args:
            latitude: WGS-84 latitude in degrees [-90, 90]
            longitude: WGS-84 longitude in degrees [-180, 180]
            altitude: Ellipsoidal or orthometric altitude in meters
            status: NavSatStatus status code (STATUS_NO_FIX, STATUS_FIX, etc.)
            service: NavSatStatus service flag (SERVICE_GPS, etc.)
            std_devs: Optional tuple of (lon_std, lat_std, alt_std) in meters for ENU diagonal covariance
            stamp: Optional ROS Time stamp; if None, current clock is used.
        """
        if not (math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(altitude)):
            return None

        fix = NavSatFix()
        fix.header.stamp = stamp or self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.status.status = status
        fix.status.service = service
        fix.latitude = float(latitude)
        fix.longitude = float(longitude)
        fix.altitude = float(altitude)

        # Standard ROS ENU convention:
        # index 0: East (Lon variance), index 4: North (Lat variance), index 8: Up (Alt variance)
        if (
            std_devs is not None
            and status != NavSatStatus.STATUS_NO_FIX
            and all(math.isfinite(s) and s >= 0.0 for s in std_devs)
        ):
            lon_std, lat_std, alt_std = std_devs
            fix.position_covariance[0] = float(lon_std) ** 2
            fix.position_covariance[4] = float(lat_std) ** 2
            fix.position_covariance[8] = float(alt_std) ** 2
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        if self.fix_pub is not None:
            self.fix_pub.publish(fix)

        return fix

    def publish_rtk(self, rtk_msg: UniRtkPvh) -> None:
        """Publish a robots_dog_msgs/UniRtkPvh message to rtk_topic if RTK is enabled."""
        if self.rtk_pub is not None:
            self.rtk_pub.publish(rtk_msg)
