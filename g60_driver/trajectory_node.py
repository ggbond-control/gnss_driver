import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger


class LocalTrajectory(Node):
    """Converts WGS84 fixes to a bounded local ENU path around the first fix."""

    def __init__(self):
        super().__init__('g60_trajectory')
        self.fix_topic = self.declare_parameter('fix_topic', 'fix').value
        self.path_topic = self.declare_parameter('path_topic', 'gps_path').value
        self.frame_id = self.declare_parameter('frame_id', 'gps_path').value
        self.max_points = self.declare_parameter('max_points', 10000).value
        self.origin = None
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.publisher = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, 10)
        self.create_service(Trigger, 'reset_trajectory', self._reset)

    def _reset(self, _, response):
        self.origin = None
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        response.success = True
        response.message = 'trajectory origin and points reset'
        return response

    def _on_fix(self, fix):
        values = (fix.latitude, fix.longitude, fix.altitude)
        if fix.status.status < 0 or not all(math.isfinite(value) for value in values):
            return
        if self.origin is None:
            self.origin = values
            self.get_logger().info('trajectory origin: lat={:.8f}, lon={:.8f}, alt={:.3f}'.format(*values))
            return
        latitude0, longitude0, altitude0 = self.origin
        radius_m = 6378137.0
        north = radius_m * math.radians(fix.latitude - latitude0)
        east = radius_m * math.cos(math.radians(latitude0)) * math.radians(fix.longitude - longitude0)
        pose = PoseStamped()
        pose.header.stamp = fix.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = fix.altitude - altitude0
        pose.pose.orientation.w = 1.0
        self.path.header.stamp = fix.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[:len(self.path.poses) - self.max_points]
        self.publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = LocalTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
