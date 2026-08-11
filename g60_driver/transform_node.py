"""Replay odometry poses as NavSatFix using a saved GPS/odometry transform."""

import math
import os
import threading

from action_msgs.msg import GoalStatus
import rclpy
from inspection_interfaces.action import SetGPSGoal
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .transform_io import GpsOdomTransform, default_data_path


class TransformNode(Node):
    def __init__(self):
        super().__init__('g60_transform')
        self.input_topic = self.declare_parameter('input_topic', '/odometry_horizon').value
        self.output_topic = self.declare_parameter('output_topic', '/fix_from_odom').value
        self.enable_gps_goal_action = self.declare_parameter('enable_gps_goal_action', True).value
        self.gps_goal_action = self.declare_parameter('gps_goal_action', '/set_gps_goal').value
        self.navigation_action = self.declare_parameter(
            'navigation_action', '/multi_map_navigate_to_pose').value
        self.action_server_wait_sec = self.declare_parameter('action_server_wait_sec', 5.0).value
        self.action_result_timeout_sec = self.declare_parameter(
            'action_result_timeout_sec', 0.0).value
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
        self.navigation_client = None
        self.gps_goal_server = None
        self.navigation_group = ReentrantCallbackGroup()
        self.navigation_goal_lock = threading.Lock()
        self.active_navigation_goal_handle = None
        self.active_navigation_goal_lock = threading.Lock()
        if self.enable_gps_goal_action:
            self.navigation_client = ActionClient(
                self, NavigateToPose, self.navigation_action,
                callback_group=self.navigation_group)
            self.gps_goal_server = ActionServer(
                self, SetGPSGoal, self.gps_goal_action,
                execute_callback=self._execute_gps_goal,
                cancel_callback=self._on_cancel_gps_goal,
                callback_group=self.navigation_group)
            self.get_logger().info('bridging {} to {}'.format(
                self.gps_goal_action, self.navigation_action))
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

    def _execute_gps_goal(self, goal_handle):
        # Serialize goals and return only after Nav2 reports a terminal action result.
        with self.navigation_goal_lock:
            return self._send_and_wait_for_navigation(goal_handle)

    def _on_cancel_gps_goal(self, _):
        self.get_logger().info('received SetGPSGoal cancel request')
        return CancelResponse.ACCEPT

    @staticmethod
    def _wait_future(future, timeout_sec):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if timeout_sec and timeout_sec > 0.0:
            if not completed.wait(timeout_sec):
                return None, True
        else:
            completed.wait()
        try:
            return future.result(), False
        except Exception as error:
            return error, False

    def _result(self, success, message):
        result = SetGPSGoal.Result()
        result.success = success
        result.message = message
        return result

    def _abort(self, goal_handle, message):
        goal_handle.abort()
        return self._result(False, message)

    def _cancel(self, goal_handle, message):
        goal_handle.canceled()
        return self._result(False, message)

    def _cancel_navigation_goal(self):
        with self.active_navigation_goal_lock:
            navigation_goal_handle = self.active_navigation_goal_handle
        if navigation_goal_handle is None:
            return 'no active navigation goal to cancel'
        cancel_future = navigation_goal_handle.cancel_goal_async()
        cancel_response, timed_out = self._wait_future(cancel_future, self.action_server_wait_sec)
        if timed_out:
            return 'timed out while forwarding cancel to {}'.format(self.navigation_action)
        if isinstance(cancel_response, Exception):
            return 'failed to forward cancel to {}: {}'.format(self.navigation_action, cancel_response)
        goals_canceling = getattr(cancel_response, 'goals_canceling', [])
        if goals_canceling:
            return 'forwarded cancel to {}'.format(self.navigation_action)
        return 'cancel request sent to {}, but no goals were reported canceling'.format(
            self.navigation_action)

    def _wait_navigation_result(self, result_future, goal_handle):
        completed = threading.Event()
        result_future.add_done_callback(lambda _: completed.set())
        deadline = None
        if self.action_result_timeout_sec and self.action_result_timeout_sec > 0.0:
            deadline = self.get_clock().now().nanoseconds / 1e9 + self.action_result_timeout_sec
        while True:
            if goal_handle.is_cancel_requested:
                cancel_message = self._cancel_navigation_goal()
                return None, False, cancel_message
            if completed.wait(0.1):
                try:
                    return result_future.result(), False, None
                except Exception as error:
                    return error, False, None
            if deadline is not None and self.get_clock().now().nanoseconds / 1e9 >= deadline:
                return None, True, None

    def _send_and_wait_for_navigation(self, goal_handle):
        request = goal_handle.request
        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle, 'SetGPSGoal was canceled before processing')
        if not request.header.frame_id:
            return self._abort(goal_handle, 'header.frame_id must be provided for the navigation goal')
        values = (request.latitude, request.longitude, request.altitude)
        if not all(math.isfinite(value) for value in values):
            return self._abort(goal_handle, 'latitude, longitude, and altitude must be finite')
        if not -90.0 <= request.latitude <= 90.0 or not -180.0 <= request.longitude <= 180.0:
            return self._abort(goal_handle, 'latitude or longitude is outside the valid range')
        orientation = request.orientation
        norm = math.sqrt(orientation.x ** 2 + orientation.y ** 2 +
                         orientation.z ** 2 + orientation.w ** 2)
        if not math.isfinite(norm) or norm == 0.0:
            return self._abort(goal_handle, 'orientation must be a non-zero finite quaternion')
        if not self.navigation_client.wait_for_server(timeout_sec=self.action_server_wait_sec):
            return self._abort(
                goal_handle,
                'navigation action server is not ready after {:.1f}s: {}'.format(
                    self.action_server_wait_sec, self.navigation_action))
        if goal_handle.is_cancel_requested:
            return self._cancel(goal_handle, 'SetGPSGoal was canceled before sending navigation goal')

        x, y, z = self.transform.gps_lla_to_world(*values)
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = request.header.frame_id
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = float(z)
        goal.pose.pose.orientation.x = orientation.x / norm
        goal.pose.pose.orientation.y = orientation.y / norm
        goal.pose.pose.orientation.z = orientation.z / norm
        goal.pose.pose.orientation.w = orientation.w / norm
        goal_future = self.navigation_client.send_goal_async(
            goal, feedback_callback=lambda feedback: self._forward_navigation_feedback(
                goal_handle, feedback))
        navigation_goal_handle, timed_out = self._wait_future(
            goal_future, self.action_server_wait_sec)
        if timed_out:
            return self._abort(goal_handle, 'timed out waiting for navigation goal acceptance')
        if isinstance(navigation_goal_handle, Exception):
            return self._abort(
                goal_handle, 'navigation goal request failed: {}'.format(navigation_goal_handle))
        if navigation_goal_handle is None or not navigation_goal_handle.accepted:
            return self._abort(goal_handle, 'navigation goal was rejected')

        with self.active_navigation_goal_lock:
            self.active_navigation_goal_handle = navigation_goal_handle
        result_future = navigation_goal_handle.get_result_async()
        action_result, timed_out, cancel_message = self._wait_navigation_result(
            result_future, goal_handle)
        try:
            if cancel_message is not None:
                return self._cancel(goal_handle, cancel_message)
            if timed_out:
                cancel_message = self._cancel_navigation_goal()
                return self._abort(
                    goal_handle,
                    'timed out waiting for navigation action result; {}'.format(cancel_message))
            if isinstance(action_result, Exception):
                return self._abort(goal_handle, 'navigation action result failed: {}'.format(action_result))
            if action_result is None:
                return self._abort(goal_handle, 'navigation action returned no result')
        finally:
            with self.active_navigation_goal_lock:
                if self.active_navigation_goal_handle is navigation_goal_handle:
                    self.active_navigation_goal_handle = None

        result = action_result.result
        status = action_result.status
        error_code = getattr(result, 'error_code', 0)
        error_message = getattr(result, 'error_msg', '')
        success = (status == GoalStatus.STATUS_SUCCEEDED and error_code == 0)
        status_name = self._status_name(status)
        message = 'navigation {} (status={}, error_code={}{}), world=({:.3f}, {:.3f}, {:.3f})'.format(
            'succeeded' if success else 'failed', status_name, error_code,
            ': ' + error_message if error_message else '', x, y, z)
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self._result(success, message)

    @staticmethod
    def _forward_navigation_feedback(goal_handle, navigation_feedback):
        distance = getattr(navigation_feedback.feedback, 'distance_remaining', math.nan)
        feedback = SetGPSGoal.Feedback()
        feedback.distance_remaining = float(distance) if math.isfinite(distance) else math.nan
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _status_name(status):
        return {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(status, str(status))


def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
