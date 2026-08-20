"""Conversion from parsed NMEA data to standard ROS messages."""

from datetime import datetime, timezone
import math

from geometry_msgs.msg import QuaternionStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler

from . import nmea


class GnssDriver(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.frame_id = self.declare_parameter('frame_id', 'gps').value
        self.use_rmc_fix = self.declare_parameter('use_rmc_fix', False).value
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
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_pub = self.create_publisher(TimeReference, 'time_reference', 10)
        self.raw_nmea_pub = (
            self.create_publisher(String, self.raw_nmea_topic, 50)
            if self.publish_raw_nmea and self.raw_nmea_topic else None)
        self.valid_fix = False
        self.receiver_std_dev = None
        self.utc_date = None

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_time(self, stamp, seconds):
        epoch = nmea.epoch_seconds(self.utc_date, seconds)
        if epoch is None:
            return
        message = TimeReference()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.source = self.time_ref_source or self.frame_id
        message.time_ref.sec = int(epoch)
        message.time_ref.nanosec = int((epoch % 1) * 1e9)
        self.time_pub.publish(message)

    def _publish_velocity(self, speed, course, stamp):
        if not self.valid_fix or math.isnan(speed) or math.isnan(course):
            return
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.twist.linear.x = speed * math.sin(course)
        message.twist.linear.y = speed * math.cos(course)
        self.velocity_pub.publish(message)

    def _publish_gga(self, data, stamp):
        quality_to_status = {
            0: NavSatStatus.STATUS_NO_FIX, 1: NavSatStatus.STATUS_FIX,
            2: NavSatStatus.STATUS_SBAS_FIX, 4: NavSatStatus.STATUS_GBAS_FIX,
            5: NavSatStatus.STATUS_GBAS_FIX, 9: NavSatStatus.STATUS_GBAS_FIX,
        }
        message = NavSatFix()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.status.service = NavSatStatus.SERVICE_GPS
        message.status.status = quality_to_status.get(data.fix_quality, NavSatStatus.STATUS_NO_FIX)
        self.valid_fix = message.status.status != NavSatStatus.STATUS_NO_FIX
        message.latitude, message.longitude = data.latitude, data.longitude
        message.altitude = data.altitude_msl + data.geoid_separation
        epe = self.epe_by_quality.get(data.fix_quality, self.epe_by_quality[0])
        lon, lat, alt = self.receiver_std_dev or (epe, epe, epe * 2.0)
        if self.valid_fix and not any(math.isnan(value) for value in (lon, lat, alt, data.hdop)):
            message.position_covariance[0] = (data.hdop * lon) ** 2
            message.position_covariance[4] = (data.hdop * lat) ** 2
            message.position_covariance[8] = (2.0 * data.hdop * alt) ** 2
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(message)
        self._publish_time(stamp, data.utc_seconds)

    def add_sentence(self, sentence: str, stamp=None):
        sentence = sentence.strip()
        if self.raw_nmea_pub is not None and sentence:
            raw_message = String()
            raw_message.data = sentence
            self.raw_nmea_pub.publish(raw_message)
        try:
            data = nmea.parse(sentence)
        except ValueError as error:
            self.get_logger().warning('{}: {}'.format(error, sentence.strip()))
            return False
        if data is None:
            return False
        stamp = stamp or self._stamp()
        if isinstance(data, nmea.Gga) and not self.use_rmc_fix:
            self._publish_gga(data, stamp)
        elif isinstance(data, nmea.Rmc):
            self.utc_date = data.utc_date or self.utc_date
            if self.use_rmc_fix:
                message = NavSatFix()
                message.header.stamp, message.header.frame_id = stamp, self.frame_id
                message.status.service = NavSatStatus.SERVICE_GPS
                message.status.status = NavSatStatus.STATUS_FIX if data.valid else NavSatStatus.STATUS_NO_FIX
                message.latitude, message.longitude, message.altitude = data.latitude, data.longitude, math.nan
                message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                self.valid_fix = data.valid
                self.fix_pub.publish(message)
            if data.valid:
                self._publish_velocity(data.speed_mps, data.course_rad, stamp)
            self._publish_time(stamp, data.utc_seconds)
        elif isinstance(data, nmea.Vtg):
            self._publish_velocity(data.speed_mps, data.course_rad, stamp)
        elif isinstance(data, nmea.Gst):
            self.receiver_std_dev = (data.lon_std_dev, data.lat_std_dev, data.alt_std_dev)
        elif isinstance(data, nmea.Hdt) and not math.isnan(data.heading_deg):
            message = QuaternionStamped()
            message.header.stamp, message.header.frame_id = stamp, self.frame_id
            # NMEA heading is clockwise from north; ROS ENU yaw is counter-clockwise from east.
            quaternion = quaternion_from_euler(0.0, 0.0, math.radians(90.0 - data.heading_deg))
            message.quaternion.x, message.quaternion.y, message.quaternion.z, message.quaternion.w = quaternion
            self.heading_pub.publish(message)
        return True
