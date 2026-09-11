"""Replay odometry poses as NavSatFix using a saved GPS/odometry transform."""

import math
import os
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
import rclpy
from inspection_interfaces.action import SetGPSGoal
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import numpy as np
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from robots_dog_msgs.msg import UniRtkPvh
from .rtk import rtk_to_navsat_fix
from .transform_io import GpsOdomTransform, default_data_path


class TransformNode(Node):
    def __init__(self):
        super().__init__('gnss_transform')
        self.input_topic = self.declare_parameter('input_topic', '/odometry_horizon').value
        self.output_topic = self.declare_parameter('output_topic', '/fix_from_odom').value
        self.rtk_topic = self.declare_parameter('rtk_topic', '').value
        self.rtk_output_topic = self.declare_parameter(
            'rtk_output_topic', '/odometry_from_rtk').value
        self.rtk_fix_topic = self.declare_parameter('rtk_fix_topic', '/fix_from_rtk').value
        self.rtk_frame_id = self.declare_parameter('rtk_frame_id', 'gps').value
        self.rtk_child_frame_id = self.declare_parameter('rtk_child_frame_id', 'rtk_gps').value
        self.enable_gps_goal_action = self.declare_parameter('enable_gps_goal_action', True).value
        self.gps_goal_action = self.declare_parameter('gps_goal_action', '/set_gps_goal').value
        self.navigation_action = self.declare_parameter(
            'navigation_action', '/multi_map_navigate_to_pose').value
        self.next_goal_policy_service = self.declare_parameter(
            'next_goal_policy_service', '/next_goal_policy').value
        self.action_server_wait_sec = self.declare_parameter('action_server_wait_sec', 5.0).value
        self.next_goal_policy_wait_sec = self.declare_parameter(
            'next_goal_policy_wait_sec', 5.0).value
        self.action_result_timeout_sec = self.declare_parameter(
            'action_result_timeout_sec', 0.0).value
        path = self.declare_parameter('transform_path', '').value
        if path and not os.path.isabs(os.path.expanduser(path)):
            self.transform_path = default_data_path(path)
        elif path:
            self.transform_path = os.path.expanduser(path)
        else:
            self.transform_path = default_data_path('gps_odom_transform.txt')
        try:
            self.transform = GpsOdomTransform.load(self.transform_path)
        except (OSError, ValueError) as error:
            self.get_logger().fatal('cannot load transform {}: {}'.format(self.transform_path, error))
            raise
        if self.transform.output_frame == self.transform.gps_frame:
            raise ValueError('output_frame and gps_frame must be different in {}'.format(
                self.transform_path))
        if self.transform.output_frame == 'map':
            raise ValueError('output_frame must differ from map for rviz_satellite')
        self.publisher = self.create_publisher(NavSatFix, self.output_topic, 10)
        self.rtk_odom_pub = (
            self.create_publisher(Odometry, self.rtk_output_topic, 10) if self.rtk_topic else None)
        self.rtk_fix_pub = (self.create_publisher(NavSatFix, self.rtk_fix_topic, 10)
                            if self.rtk_topic and self.rtk_fix_topic else None)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_map_tf()
        # Odometry sources from sensor/SLAM stacks are often BEST_EFFORT.
        self.create_subscription(Odometry, self.input_topic, self._on_odom, qos_profile_sensor_data)
        if self.rtk_topic:
            self.create_subscription(UniRtkPvh, self.rtk_topic, self._on_rtk, qos_profile_sensor_data)
        self.navigation_client = None
        self.next_goal_policy_client = None
        self.gps_goal_server = None
        self.navigation_group = ReentrantCallbackGroup()
        self.navigation_goal_lock = threading.Lock()
        self.active_navigation_goal_handle = None
        self.active_navigation_goal_lock = threading.Lock()
        if self.enable_gps_goal_action:
            self.navigation_client = ActionClient(
                self, NavigateToPose, self.navigation_action,
                callback_group=self.navigation_group)
            self.next_goal_policy_client = self.create_client(
                SetParameters, self.next_goal_policy_service,
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
        if self.rtk_topic:
            self.get_logger().info('converting {} UniRtkPvh positions to {}'.format(
                self.rtk_topic, self.rtk_output_topic))

    def _publish_map_tf(self):
        """Publish rviz_satellite's hard-coded ENU map frame from the TXT rotation."""
        yaw = math.atan2(self.transform.rotation[1, 0], self.transform.rotation[0, 0])
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.transform.output_frame
        transform.child_frame_id = 'map'
        transform.transform.translation.x = float(self.transform.translation[0])
        transform.transform.translation.y = float(self.transform.translation[1])
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.static_tf_broadcaster.sendTransform(transform)
        self.get_logger().info('published rviz_satellite ENU TF {} -> map (yaw={:.3f} deg)'.format(
            self.transform.output_frame, math.degrees(yaw)))

    def _on_odom(self, message):
        position = message.pose.pose.position
        xyz = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in xyz):
            self.get_logger().warning('ignoring non-finite odometry position')
            return
        if not hasattr(self, '_first_odom_logged'):
            self._first_odom_logged = True
            self.get_logger().info('active conversion: odometry -> /fix_from_odom running normally')
        self._publish_gps_tf(message)
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

    def _on_rtk(self, message):
        fix = rtk_to_navsat_fix(message, self.rtk_frame_id)
        if fix is None:
            status = message.bestnav.p_sol_status
            status_desc = {
                0: 'SOL_COMPUTED',
                1: 'INSUFFICIENT_OBS (未搜到足够卫星/无有效定位解)',
                2: 'NO_CONVERGENCE (解算未收敛)',
                3: 'SINGULARITY (矩阵奇异)',
                6: 'COLD_START (冷启动搜星中)',
            }.get(status, f'STATUS_{status}')
            self.get_logger().warning(
                f'ignoring UniRtkPvh without a solved position (p_sol_status={status}: {status_desc})',
                throttle_duration_sec=5.0)
            return
        if not hasattr(self, '_first_rtk_logged'):
            self._first_rtk_logged = True
            self.get_logger().info('valid RTK position received (p_sol_status=0), publishing to /odometry_from_rtk')
        if self.rtk_fix_pub is not None:
            self.rtk_fix_pub.publish(fix)
        x, y, z = self.transform.gps_lla_to_world(
            fix.latitude, fix.longitude, fix.altitude)
        odom = Odometry()
        odom.header.stamp = fix.header.stamp
        odom.header.frame_id = self.transform.output_frame
        odom.child_frame_id = self.rtk_child_frame_id
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation.w = 1.0
        self._set_rtk_position_covariance(odom, fix)
        self.rtk_odom_pub.publish(odom)

    def _set_rtk_position_covariance(self, odom, fix):
        """Rotate UniBestNav east/north covariance into the TXT world axes."""
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            odom.pose.covariance[21] = -1.0
            return
        enu_covariance = np.diag((fix.position_covariance[0], fix.position_covariance[4]))
        world_covariance = self.transform.rotation @ enu_covariance @ self.transform.rotation.T
        odom.pose.covariance[0] = float(world_covariance[0, 0])
        odom.pose.covariance[1] = float(world_covariance[0, 1])
        odom.pose.covariance[6] = float(world_covariance[1, 0])
        odom.pose.covariance[7] = float(world_covariance[1, 1])
        odom.pose.covariance[14] = fix.position_covariance[8]
        # Heading is intentionally not mapped in this first RTK integration.
        odom.pose.covariance[21] = -1.0

    def _publish_gps_tf(self, message):
        """Place the NavSatFix sensor frame at the same world pose as the odometry."""
        pose = message.pose.pose
        orientation = pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        norm = math.sqrt(sum(value * value for value in quaternion))

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.transform.output_frame
        transform.child_frame_id = self.transform.gps_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        if math.isfinite(norm) and norm > 0.0:
            transform.transform.rotation.x = orientation.x / norm
            transform.transform.rotation.y = orientation.y / norm
            transform.transform.rotation.z = orientation.z / norm
            transform.transform.rotation.w = orientation.w / norm
        else:
            # A valid TF rotation is required even when the odometry source omits attitude.
            transform.transform.rotation.w = 1.0
            self.get_logger().warning('odometry orientation is invalid; publishing identity rotation for {} -> {}'.format(
                self.transform.output_frame, self.transform.gps_frame))
        self.tf_broadcaster.sendTransform(transform)

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

    def _wait_for_policy_service(self, goal_handle):
        deadline = self.get_clock().now().nanoseconds / 1e9 + self.next_goal_policy_wait_sec
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            if goal_handle.is_cancel_requested:
                return False, True
            if self.next_goal_policy_client.wait_for_service(timeout_sec=0.1):
                return True, False
        return False, False

    def _set_next_goal_yaw_policy(self, goal_handle, skip_yaw_alignment):
        service_ready, canceled = self._wait_for_policy_service(goal_handle)
        if canceled:
            return False, 'SetGPSGoal was canceled before setting next goal policy', True
        if not service_ready:
            return False, 'next goal policy service is not ready after {:.1f}s: {}'.format(
                self.next_goal_policy_wait_sec, self.next_goal_policy_service), False

        parameter = Parameter()
        parameter.name = 'align_final_yaw'
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = not skip_yaw_alignment
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.next_goal_policy_client.call_async(request)

        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        deadline = self.get_clock().now().nanoseconds / 1e9 + self.next_goal_policy_wait_sec
        while not completed.wait(0.1):
            if goal_handle.is_cancel_requested:
                return False, 'SetGPSGoal was canceled while setting next goal policy', True
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                return False, 'timed out waiting for next goal policy response', False
        try:
            response = future.result()
        except Exception as error:
            return False, 'next goal policy request failed: {}'.format(error), False
        if response is None or not response.results:
            return False, 'next goal policy returned no parameter result', False
        failed = [result.reason or 'unspecified error'
                  for result in response.results if not result.successful]
        if failed:
            return False, 'next goal policy was rejected: {}'.format('; '.join(failed)), False
        return True, 'align_final_yaw={}'.format(not skip_yaw_alignment), False

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
        policy_set, policy_message, canceled = self._set_next_goal_yaw_policy(
            goal_handle, request.skip_yaw_alignment)
        if canceled:
            return self._cancel(goal_handle, policy_message)
        if not policy_set:
            return self._abort(goal_handle, policy_message)

        # The next-goal policy is consumed by the next Nav2 goal. Once it is
        # accepted by multi_map_nav, always send that goal even if cancellation
        # arrived in the meantime; cancellation is forwarded immediately below.
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
        if goal_handle.is_cancel_requested:
            cancel_message = self._cancel_navigation_goal()
            with self.active_navigation_goal_lock:
                if self.active_navigation_goal_handle is navigation_goal_handle:
                    self.active_navigation_goal_handle = None
            return self._cancel(goal_handle, cancel_message)
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
