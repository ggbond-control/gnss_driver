"""Replay odometry poses as NavSatFix using a saved GPS/odometry transform."""

import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .transform_io import GpsOdomTransform, default_data_path


class TransformNode(Node):
    def __init__(self):
        super().__init__('g60_transform')
        self.input_topic = self.declare_parameter('input_topic', '/odometry_horizon').value
        self.output_topic = self.declare_parameter('output_topic', '/fix_from_odom').value
        path = self.declare_parameter('transform_path', '').value
        self.transform_path = os.path.expanduser(path) if path else default_data_path('gps_odom_transform.txt')
        try:
            self.transform = GpsOdomTransform.load(self.transform_path)
        except (OSError, ValueError) as error:
            self.get_logger().fatal('cannot load transform {}: {}'.format(self.transform_path, error))
            raise
        self.publisher = self.create_publisher(NavSatFix, self.output_topic, 10)
        # Odometry sources from sensor/SLAM stacks are often BEST_EFFORT.
        self.create_subscription(Odometry, self.input_topic, self._on_odom, qos_profile_sensor_data)
        self.get_logger().info('converting {} odometry to {} using {}'.format(
            self.input_topic, self.output_topic, self.transform_path))

    def _on_odom(self, message):
        position = message.pose.pose.position
        xyz = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in xyz):
            self.get_logger().warning('ignoring non-finite odometry position')
            return
        latitude, longitude, altitude = self.transform.world_to_gps_lla(*xyz)
        fix = NavSatFix()
        fix.header.stamp = message.header.stamp
        fix.header.frame_id = self.transform.gps_frame
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(latitude)
        fix.longitude = float(longitude)
        fix.altitude = float(altitude)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
