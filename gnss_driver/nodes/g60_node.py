"""G60 Serial Driver Node (Single-antenna NMEA-0183 GNSS receiver)."""
import math
from datetime import datetime, timezone
from typing import Optional

from geometry_msgs.msg import QuaternionStamped, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

from .. import nmea
from .base_serial_node import BaseSerialGnssNode


class G60DriverNode(BaseSerialGnssNode):
    """Driver node for G60 single-antenna NMEA GNSS receivers.
    
    Inherits common serial lifecycle from BaseSerialGnssNode and LLA publishing
    from BaseGnssNode. Implements NMEA-0183 parsing and publishing of standard
    /fix, /vel, /heading, and /time_reference topics.
    """

    def __init__(self):
        super().__init__(node_name='gnss_device', default_baud=9600, default_is_rtk=False)

    def setup_subclass(self) -> None:
        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.publish_raw_nmea = self.declare_parameter('publish_raw_nmea', False).value
        self.raw_nmea_topic = self.declare_parameter('raw_nmea_topic', 'nmea_sentence').value

        self.epe_by_quality = {
            0: self.declare_parameter('epe_no_fix', 1000000.0).value,
            1: self.declare_parameter('epe_sps', 4.0).value,
            2: self.declare_parameter('epe_dgps', 0.1).value,
            4: self.declare_parameter('epe_rtk_fixed', 0.02).value,
            5: self.declare_parameter('epe_rtk_float', 4.0).value,
            9: self.declare_parameter('epe_waas', 3.0).value,
        }

        self.velocity_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_pub = self.create_publisher(TimeReference, 'time_reference', 10)
        self.raw_nmea_pub = (
            self.create_publisher(String, self.raw_nmea_topic, 50)
            if self.publish_raw_nmea and self.raw_nmea_topic else None
        )

        self.valid_fix = False
        self.receiver_std_dev = None
        self.utc_date = None

    def handle_line(self, line: str) -> None:
        if self.raw_nmea_pub is not None and line:
            raw_msg = String()
            raw_msg.data = line
            self.raw_nmea_pub.publish(raw_msg)

        try:
            data = nmea.parse(line)
        except ValueError as err:
            self.get_logger().warning(f'nmea parse error: {err}: {line}')
            return

        if data is None:
            return

        stamp = self.get_clock().now().to_msg()

        if isinstance(data, nmea.Gga):
            self._handle_gga(data, stamp)
        elif isinstance(data, nmea.Rmc):
            self.utc_date = data.utc_date or self.utc_date
            if data.valid:
                self._publish_velocity(data.speed_mps, data.course_rad, stamp)
            self._publish_time(stamp, data.utc_seconds)
        elif isinstance(data, nmea.Vtg):
            self._publish_velocity(data.speed_mps, data.course_rad, stamp)
        elif isinstance(data, nmea.Gst):
            self.receiver_std_dev = (data.lon_std_dev, data.lat_std_dev, data.alt_std_dev)
        elif isinstance(data, nmea.Hdt) and not math.isnan(data.heading_deg):
            msg = QuaternionStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            # NMEA heading is clockwise from north; ROS ENU yaw is counter-clockwise from east
            quaternion = quaternion_from_euler(0.0, 0.0, math.radians(90.0 - data.heading_deg))
            msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w = quaternion
            self.heading_pub.publish(msg)

    def _handle_gga(self, data: nmea.Gga, stamp):
        quality_to_status = {
            0: NavSatStatus.STATUS_NO_FIX,
            1: NavSatStatus.STATUS_FIX,
            2: NavSatStatus.STATUS_SBAS_FIX,
            4: NavSatStatus.STATUS_GBAS_FIX,
            5: NavSatStatus.STATUS_GBAS_FIX,
            9: NavSatStatus.STATUS_GBAS_FIX,
        }
        status = quality_to_status.get(data.fix_quality, NavSatStatus.STATUS_NO_FIX)
        self.valid_fix = (status != NavSatStatus.STATUS_NO_FIX)

        altitude = data.altitude_msl + data.geoid_separation
        epe = self.epe_by_quality.get(data.fix_quality, self.epe_by_quality[0])
        lon, lat, alt = self.receiver_std_dev or (epe, epe, epe * 2.0)

        std_devs = None
        if self.valid_fix and not any(math.isnan(v) for v in (lon, lat, alt, data.hdop)):
            std_devs = (data.hdop * lon, data.hdop * lat, 2.0 * data.hdop * alt)

        self.publish_fix(
            latitude=data.latitude,
            longitude=data.longitude,
            altitude=altitude,
            status=status,
            std_devs=std_devs,
            stamp=stamp,
        )
        self._publish_time(stamp, data.utc_seconds)

    def _publish_time(self, stamp, seconds):
        epoch = nmea.epoch_seconds(self.utc_date, seconds)
        if epoch is None:
            return
        msg = TimeReference()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.source = self.time_ref_source or self.frame_id
        msg.time_ref.sec = int(epoch)
        msg.time_ref.nanosec = int((epoch % 1) * 1e9)
        self.time_pub.publish(msg)

    def _publish_velocity(self, speed, course, stamp):
        if not self.valid_fix or math.isnan(speed) or math.isnan(course):
            return
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = speed * math.sin(course)
        msg.twist.linear.y = speed * math.cos(course)
        self.velocity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = G60DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
