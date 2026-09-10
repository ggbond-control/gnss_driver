"""Align GNSS positions to a live odometry frame with a 2D rigid fit."""

from collections import deque
import math
import os
import tempfile

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from robots_dog_msgs.msg import UniRtkPvh
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
from .transform_io import GpsOdomTransform, default_data_path
from .rtk import rtk_to_navsat_fix
from .estimators.se2 import fit_rigid_2d


class GpsOdomAlignment(Node):
    def __init__(self):
        super().__init__('gnss_alignment')
        self.fix_topic = self.declare_parameter('fix_topic', '/fix').value
        self.rtk_topic = self.declare_parameter('rtk_topic', '').value
        self.rtk_frame_id = self.declare_parameter('rtk_frame_id', 'gps').value
        self.rtk_fix_topic = self.declare_parameter('rtk_fix_topic', '/fix_from_rtk').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry_horizon').value
        self.output_frame = self.declare_parameter('output_frame', 'world').value
        self.fix_odom_topic = self.declare_parameter('fix_odom_topic', '/fix_odom').value
        self.fix_odom_frame_id = self.declare_parameter('fix_odom_frame_id', 'gps').value
        transform_path = self.declare_parameter('transform_path', '').value
        if transform_path and not os.path.isabs(os.path.expanduser(transform_path)):
            self.transform_path = default_data_path(transform_path)
        elif transform_path:
            self.transform_path = os.path.expanduser(transform_path)
        else:
            self.transform_path = default_data_path('gps_odom_transform.txt')
        self.get_logger().info('transform output path: {}'.format(self.transform_path))
        self.max_time_delta = self.declare_parameter('max_time_delta', 0.15).value
        self.calibration_pairs = self.declare_parameter('calibration_pairs', 30).value
        self.min_calibration_displacement = self.declare_parameter(
            'min_calibration_displacement', 1.0).value
        self.max_points = self.declare_parameter('max_points', 10000).value
        self.use_weighted_fit = self.declare_parameter('use_weighted_fit', False).value
        self.fit_iterations = self.declare_parameter('fit_iterations', 15).value
        self.huber_delta = self.declare_parameter('huber_delta', 2.5).value
        self.gps_buffer = deque(maxlen=200)
        self.odom_buffer = deque(maxlen=500)
        self.calibration_samples = []
        self.paired_gps_stamps = deque(maxlen=200)
        self.gps_origin = None
        self.transform = None
        self.gps_path = self._new_path()
        self.odom_path = self._new_path()

        self.gps_pub = self.create_publisher(Path, 'gps_trajectory_aligned', 10)
        self.odom_pub = self.create_publisher(Path, 'odometry_trajectory', 10)
        self.aligned_pose_pub = self.create_publisher(PoseStamped, 'gps_pose_aligned', 10)
        self.fix_odom_pub = self.create_publisher(NavSatFix, self.fix_odom_topic, 10)
        self.rtk_fix_pub = (self.create_publisher(NavSatFix, self.rtk_fix_topic, 10)
                            if self.rtk_topic and self.rtk_fix_topic else None)
        # Sensor publishers often use BEST_EFFORT; this profile also accepts RELIABLE inputs.
        if self.fix_topic:
            self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, qos_profile_sensor_data)
        if self.rtk_topic:
            self.create_subscription(UniRtkPvh, self.rtk_topic, self._on_rtk, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_service(Trigger, 'reset_alignment', self._reset)

    def _on_rtk(self, message):
        fix = rtk_to_navsat_fix(message, self.rtk_frame_id)
        if fix is None:
            self.get_logger().warning(
                'ignoring UniRtkPvh without a solved finite position (p_sol_status={})'.format(
                    message.bestnav.p_sol_status), throttle_duration_sec=5.0)
            return
        if self.rtk_fix_pub is not None:
            self.rtk_fix_pub.publish(fix)
        self._on_fix(fix)

    def _new_path(self):
        path = Path()
        path.header.frame_id = self.output_frame
        return path

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    @staticmethod
    def _valid_fix(message):
        return message.status.status >= 0 and all(
            math.isfinite(value) for value in (message.latitude, message.longitude, message.altitude))

    def _reset(self, _, response):
        self.gps_buffer.clear()
        self.odom_buffer.clear()
        self.calibration_samples.clear()
        self.paired_gps_stamps.clear()
        self.gps_origin = None
        self.transform = None
        self.gps_path = self._new_path()
        self.odom_path = self._new_path()
        self._write_transform(False)
        response.success = True
        response.message = 'GPS/odometry alignment reset'
        return response

    def _on_fix(self, message):
        if not self._valid_fix(message):
            return
        if self.gps_origin is None:
            self.gps_origin = (message.latitude, message.longitude, message.altitude)
            self.get_logger().info('GPS local origin set')
        point = self._gps_to_local(message)
        self.gps_buffer.append((self._stamp(message), point, message.header.stamp,
                                self._fix_covariance(message)))
        if self.transform is None:
            self._try_pair()
        if self.transform is not None:
            self._publish_aligned_point(point, message.header.stamp)

    def _on_odom(self, message):
        point = np.array((message.pose.pose.position.x, message.pose.pose.position.y), dtype=float)
        if not np.isfinite(point).all():
            return
        self.odom_buffer.append((self._stamp(message), point))
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose = message.pose.pose
        self.odom_path.header.stamp = message.header.stamp
        self.odom_path.poses.append(pose)
        self._trim(self.odom_path)
        self.odom_pub.publish(self.odom_path)
        if self.transform is None:
            self._try_pair()
        if self.transform is not None:
            self._publish_fix_odom(message)

    def _gps_to_local(self, message):
        latitude0, longitude0, _ = self.gps_origin
        radius_m = 6378137.0
        east = radius_m * math.cos(math.radians(latitude0)) * math.radians(message.longitude - longitude0)
        north = radius_m * math.radians(message.latitude - latitude0)
        return np.array((east, north), dtype=float)

    def _publish_fix_odom(self, message):
        rotation, translation = self.transform
        odom_point = np.array((message.pose.pose.position.x, message.pose.pose.position.y), dtype=float)
        gps_local = rotation.T @ (odom_point - translation)
        latitude0, longitude0, altitude0 = self.gps_origin
        radius_m = 6378137.0
        latitude = latitude0 + math.degrees(gps_local[1] / radius_m)
        longitude = longitude0 + math.degrees(
            gps_local[0] / (radius_m * math.cos(math.radians(latitude0))))
        fix = NavSatFix()
        fix.header.stamp = message.header.stamp
        fix.header.frame_id = self.fix_odom_frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude0 + message.pose.pose.position.z
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_odom_pub.publish(fix)

    def _try_pair(self):
        if self.transform is not None or not self.gps_buffer or not self.odom_buffer:
            return
        for gps_stamp, gps_point, ros_stamp, gps_covariance in self.gps_buffer:
            if gps_stamp in self.paired_gps_stamps:
                continue
            odom_stamp, odom_point = min(self.odom_buffer, key=lambda item: abs(item[0] - gps_stamp))
            if abs(odom_stamp - gps_stamp) > self.max_time_delta:
                continue
            if not self._is_calibration_motion(gps_point, odom_point):
                self.paired_gps_stamps.append(gps_stamp)
                continue
            self.calibration_samples.append((gps_point, odom_point, gps_covariance))
            self.paired_gps_stamps.append(gps_stamp)
            if len(self.calibration_samples) == 1 or len(self.calibration_samples) % 10 == 0:
                self.get_logger().info(
                    'calibration progress: {}/{} moving pairs'.format(
                        len(self.calibration_samples), self.calibration_pairs))
            if len(self.calibration_samples) == self.calibration_pairs:
                self._lock_transform(ros_stamp)
                return

    def _is_calibration_motion(self, gps_point, odom_point):
        if not self.calibration_samples:
            return True
        previous_gps, previous_odom = self.calibration_samples[-1][:2]
        gps_distance = np.linalg.norm(gps_point - previous_gps)
        odom_distance = np.linalg.norm(odom_point - previous_odom)
        return (gps_distance >= self.min_calibration_displacement and
                odom_distance >= self.min_calibration_displacement)

    def _fit_transform(self):
        gps = np.array([pair[0] for pair in self.calibration_samples])
        odom = np.array([pair[1] for pair in self.calibration_samples])
        if self.use_weighted_fit:
            return self._fit_weighted(gps, odom)
        return self._fit_unweighted(gps, odom)

    @staticmethod
    def _fit_unweighted(gps, odom):
        return fit_rigid_2d(gps, odom)

    @staticmethod
    def _fix_covariance(message):
        if message.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        covariance = np.array([
            [message.position_covariance[0], message.position_covariance[1]],
            [message.position_covariance[3], message.position_covariance[4]],
        ], dtype=float)
        if not np.isfinite(covariance).all() or np.any(np.diag(covariance) <= 0.0):
            return None
        return covariance

    def _fit_weighted(self, gps, odom):
        """Robust Mahalanobis weighted least-squares fit of yaw and translation."""
        rotation, translation = self._fit_unweighted(gps, odom)
        theta = math.atan2(rotation[1, 0], rotation[0, 0])
        covariances = [pair[2] for pair in self.calibration_samples]
        for _ in range(max(1, int(self.fit_iterations))):
            cos_theta, sin_theta = math.cos(theta), math.sin(theta)
            current_rotation = np.array([[cos_theta, -sin_theta],
                                         [sin_theta, cos_theta]])
            normal = np.zeros((3, 3), dtype=float)
            rhs = np.zeros(3, dtype=float)
            for gps_point, odom_point, covariance in zip(gps, odom, covariances):
                predicted = current_rotation @ gps_point + translation
                residual = odom_point - predicted
                derivative = np.array([
                    -sin_theta * gps_point[0] - cos_theta * gps_point[1],
                     cos_theta * gps_point[0] - sin_theta * gps_point[1]], dtype=float)
                jacobian = np.column_stack((derivative, -np.eye(2)))
                if covariance is None:
                    weight = np.eye(2)
                else:
                    world_covariance = current_rotation @ covariance @ current_rotation.T
                    weight = np.linalg.pinv(world_covariance)
                mahalanobis = math.sqrt(max(0.0, float(residual.T @ weight @ residual)))
                robust_weight = (1.0 if mahalanobis <= self.huber_delta
                                 else self.huber_delta / mahalanobis)
                weight *= robust_weight
                normal += jacobian.T @ weight @ jacobian
                rhs += jacobian.T @ weight @ residual
            try:
                delta = np.linalg.solve(normal, rhs)
            except np.linalg.LinAlgError:
                self.get_logger().warning('weighted fit is rank deficient; using unweighted fit')
                return self._fit_unweighted(gps, odom)
            theta += float(delta[0])
            translation += delta[1:]
            if np.linalg.norm(delta) < 1e-8:
                break
        self.get_logger().info('weighted robust least-squares fit completed')
        return np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]]), translation

    def _lock_transform(self, stamp):
        self.transform = self._fit_transform()
        self._write_transform(True)
        self.get_logger().info(
            'alignment locked with {} moving GPS/odometry pairs'.format(self.calibration_pairs))
        self.gps_path = self._new_path()
        for gps_point, _, _ in self.calibration_samples:
            self._append_aligned_point(gps_point, stamp)
        self._publish_paths()

    def _write_transform(self, locked):
        if locked and self.transform is not None and self.gps_origin is not None:
            transform = GpsOdomTransform(
                self.gps_origin[0], self.gps_origin[1], self.gps_origin[2],
                self.transform[0], self.transform[1], self.output_frame,
                self.fix_odom_frame_id, locked=True)
        else:
            transform = GpsOdomTransform(
                0.0, 0.0, 0.0, np.eye(2), np.zeros(2), self.output_frame,
                self.fix_odom_frame_id, locked=False)
        transform.save(self.transform_path)

    def _publish_aligned_point(self, point, stamp):
        self._append_aligned_point(point, stamp)
        self._publish_paths()

    def _append_aligned_point(self, point, stamp):
        rotation, translation = self.transform
        aligned = rotation @ point + translation
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = float(aligned[0])
        pose.pose.position.y = float(aligned[1])
        pose.pose.orientation.w = 1.0
        self.gps_path.header.stamp = stamp
        self.gps_path.poses.append(pose)
        self._trim(self.gps_path)

    def _publish_paths(self):
        self.gps_pub.publish(self.gps_path)
        self.aligned_pose_pub.publish(self.gps_path.poses[-1])

    def _trim(self, path):
        if len(path.poses) > self.max_points:
            del path.poses[:len(path.poses) - self.max_points]


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomAlignment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
