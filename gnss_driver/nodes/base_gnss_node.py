"""Base GNSS Node providing common GNSS data models, parameters and ROS publishing."""
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from robots_dog_msgs.msg import UniRtkPvh


class BaseGnssNode(Node):
    """Abstract base ROS node for all GNSS devices and bridges.
    
    Encapsulates core GNSS properties:
    - Coordinate frame_id (default: 'gps')
    - Output fix_topic (default: '/fix')
    - RTK capability flag (is_rtk) and rtk_topic (default: '/rtk_pvh')
    - Standardized NavSatFix, UniRtkPvh, and sensor_msgs/Imu heading publishing
    """

    def __init__(self, node_name: str = 'gnss_device', default_is_rtk: bool = False):
        super().__init__(node_name)
        self.frame_id = self.declare_parameter('frame_id', 'gps').value
        self.fix_topic = self.declare_parameter('fix_topic', '/fix').value
        self.is_rtk = self.declare_parameter('is_rtk', default_is_rtk).value
        self.rtk_topic = self.declare_parameter('rtk_topic', '/rtk_pvh').value
        self.enable_heading_gnss = self.declare_parameter('publish_heading_gnss', True).value
        self.heading_gnss_topic = self.declare_parameter('heading_gnss_topic', '/heading_gnss').value

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.rtk_pub = (
            self.create_publisher(UniRtkPvh, self.rtk_topic, 10)
            if self.is_rtk and self.rtk_topic else None
        )
        self.heading_gnss_pub = (
            self.create_publisher(Imu, self.heading_gnss_topic, 10)
            if self.enable_heading_gnss and self.heading_gnss_topic else None
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
        if latitude is None or longitude is None or altitude is None:
            return None
        try:
            lat = float(latitude)
            lon = float(longitude)
            alt = float(altitude)
        except (TypeError, ValueError):
            return None

        if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt)):
            return None

        fix = NavSatFix()
        fix.header.stamp = stamp or self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.status.status = status
        fix.status.service = service
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt

        # Standard ROS ENU convention:
        # index 0: East (Lon variance), index 4: North (Lat variance), index 8: Up (Alt variance)
        if (
            std_devs is not None
            and status != NavSatStatus.STATUS_NO_FIX
            and all(s is not None and math.isfinite(s) and s >= 0.0 for s in std_devs)
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

    def publish_heading_gnss(
        self,
        heading_deg: float,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
        heading_std_deg: Optional[float] = None,
        stamp=None,
    ) -> Optional[Imu]:
        """Publish dual-antenna heading as standard sensor_msgs/Imu orientation (REP-103 ENU)."""
        if self.heading_gnss_pub is None or heading_deg is None:
            return None
        try:
            h_deg = float(heading_deg)
            p_deg = float(pitch_deg) if pitch_deg is not None else 0.0
            r_deg = float(roll_deg) if roll_deg is not None else 0.0
        except (TypeError, ValueError):
            return None

        if not (math.isfinite(h_deg) and math.isfinite(p_deg) and math.isfinite(r_deg)):
            return None

        # Convert Geographic Heading (0 North, clockwise) to ROS REP-103 ENU Yaw (0 East, counter-clockwise):
        # yaw_enu = pi/2 - heading_rad
        heading_rad = math.radians(h_deg % 360.0)
        yaw_enu = math.pi * 0.5 - heading_rad
        pitch_rad = math.radians(p_deg)
        roll_rad = math.radians(r_deg)

        cy = math.cos(yaw_enu * 0.5)
        sy = math.sin(yaw_enu * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        imu = Imu()
        imu.header.stamp = stamp or self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        imu.orientation.w = cr * cp * cy + sr * sp * sy

        # Orientation covariance (index 8 is Yaw)
        std_rad = math.radians(heading_std_deg) if (heading_std_deg is not None and math.isfinite(heading_std_deg)) else math.radians(0.2)
        imu.orientation_covariance[0] = math.radians(1.0) ** 2
        imu.orientation_covariance[4] = math.radians(1.0) ** 2
        imu.orientation_covariance[8] = float(std_rad) ** 2

        # Angular velocity & linear acceleration not available (-1 covariance per REP-145)
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.heading_gnss_pub.publish(imu)
        return imu

    # Backward compatibility alias
    publish_heading_imu = publish_heading_gnss
