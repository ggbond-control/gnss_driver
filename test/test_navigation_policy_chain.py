"""Real TaskHub and GNSS bridge, with local navigation/policy/gait mocks."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import numpy as np
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters, SetParametersAtomically
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Int32, String
from inspection_interfaces.srv import StartRoute, GetStatus, ValidateRouteWaypoints
from gnss_driver.transform_io import GpsOdomTransform


class NavigationPolicyChainTest(unittest.TestCase):
    def test_same_explicit_policy_through_slam_and_gps_and_rejection(self):
        if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 80:
            self.skipTest('Use an isolated ROS_DOMAIN_ID >= 80')
        rclpy.init()
        node = rclpy.create_node('navigation_policy_chain')
        group = ReentrantCallbackGroup()
        trace, pending = [], []
        reject = False
        gait_code = 6
        def policy(request, response):
            fields = {p.name: p.value for p in request.parameters}
            value = (fields['arrival_mode'].string_value, fields['speed_limit_mps'].double_value)
            trace.append(('policy', value))
            if not reject:
                pending.append(value)
            response.results = [SetParametersResult(successful=not reject, reason='mock policy')
                                for _ in request.parameters]
            return response
        def navigate(handle):
            self.assertEqual(len(pending), 1, 'each action must follow exactly one accepted policy')
            trace.append(('nav', pending.pop()))
            handle.succeed()
            return NavigateToPose.Result()
        def gait(request, response):
            nonlocal gait_code
            gait_code = 36
            trace.append(('gait', 36))
            response.results = [SetParametersResult(successful=True)]
            return response
        def valid(request, response):
            response.success = True
            return response
        speed_state = node.create_publisher(String, '/navigation/speed_limit_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        def speed_values(request):
            fields = {p.name: p.value for p in request.parameters}
            value = 0. if fields['reset'].bool_value else fields['speed_limit_mps'].double_value
            return value, fields['command_id'].string_value
        def confirm_speed(value, command):
            speed_state.publish(String(data=json.dumps(dict(schema_version=1, command_id=command,
                speed_limit_mps=value, effective_limit_mps=min(2.2, value) if value else 2.2))))
        def speed(request, response):
            value, command = speed_values(request)
            response.result.successful = True
            confirm_speed(value, command)
            trace.append(('speed', value))
            return response
        node.create_service(SetParametersAtomically, '/navigation/set_speed_limit', speed, callback_group=group)
        node.create_service(SetParameters, '/next_goal_policy', policy, callback_group=group)
        node.create_service(SetParameters, '/nav_bridge_node/set_gait', gait, callback_group=group)
        node.create_service(ValidateRouteWaypoints, '/validate_route_waypoints', valid, callback_group=group)
        nav = ActionServer(node, NavigateToPose, '/multi_map_navigate_to_pose', navigate,
                           callback_group=group)
        odom_pub = node.create_publisher(Odometry, '/odometry_multi_maps', 10)
        gait_pub = node.create_publisher(Int32, '/robot_gait_state', 10)
        def publish():
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'world'
            message.pose.pose.orientation.w = 1.
            odom_pub.publish(message)
            gait_pub.publish(Int32(data=gait_code))
        node.create_timer(.04, publish, callback_group=group)
        start = node.create_client(StartRoute, '/start_route', callback_group=group)
        status = node.create_client(GetStatus, '/get_status', callback_group=group)
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        processes, logs = [], []
        def call(client, request):
            future = client.call_async(request)
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(future.done())
            return future.result()
        try:
            with tempfile.TemporaryDirectory(prefix='navigation-policy-chain-') as directory:
                directory = Path(directory)
                transform = directory / 'transform.txt'
                GpsOdomTransform(30., 120., 0., np.eye(2), np.zeros(2)).save(transform)
                root = Path(__file__).resolve().parents[4]
                hub = os.environ.get('TASK_HUB_BINARY', str(root / 'task_ws/install/inspection_task_hub/lib/inspection_task_hub/task_hub_node'))
                commands = [
                    [sys.executable, '-c', 'from gnss_driver.transform_node import main; main()',
                     '--ros-args', '-p', 'transform_path:=' + str(transform)],
                    [hub, '--ros-args', '-p', 'energy_usage_recording_enabled:=false',
                     '-p', 'inspection_report_enabled:=false', '-p', 'runtime_log_directory:=' + str(directory)],
                ]
                for index, command in enumerate(commands):
                    log = (directory / f'node-{index}.log').open('w+')
                    logs.append(log)
                    processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
                self.assertTrue(start.wait_for_service(timeout_sec=15))
                time.sleep(1.)
                for coordinate_type in ('slam', 'gps'):
                    trace.clear()
                    points = []
                    for i, mode in enumerate(('coarse', 'position_only', 'precise'), 1):
                        p = dict(id=i, frame_id='world', yaw=0., tasks=[],
                                 position=[120.+i*.00001, 30., 0.] if coordinate_type == 'gps' else [float(i), 0., 0.])
                        if i < 3:
                            p['navigation'] = dict(arrival_mode=mode)
                        if i == 1:
                            p['tasks'] = [dict(id=101, type='set_gait', gait='L_STAIR', timeout=5),
                                          dict(id=102, type='set_speed_limit', speed_limit_mps=.5, timeout=5)]
                        if i == 2:
                            p['tasks'] = [dict(id=201, type='set_speed_limit', reset=True, timeout=5)]
                        points.append(p)
                    route = dict(route=dict(name='policy', coordinate_type=coordinate_type,
                        navigation_defaults=dict(arrival_mode='precise', speed_limit_mps=1.), waypoints=points))
                    route_path = directory / 'route.yaml'
                    route_path.write_text(json.dumps(route))
                    response = call(start, StartRoute.Request(route_config_path=str(route_path)))
                    self.assertTrue(response.success, response.message)
                    deadline = time.monotonic() + 12
                    while time.monotonic() < deadline:
                        if call(status, GetStatus.Request()).route_status.state == 3:
                            break
                        time.sleep(.05)
                    self.assertEqual(call(status, GetStatus.Request()).route_status.state, 3)
                    expected = [('coarse', 1.), ('position_only', .5), ('precise', 1.)]
                    self.assertEqual([value for kind, value in trace if kind == 'policy'], expected)
                    self.assertEqual([value for kind, value in trace if kind == 'nav'], expected)
                    self.assertLess(trace.index(('nav', expected[0])), trace.index(('gait', 36)))
                    deadline = time.monotonic() + 5
                    while ('speed', 0.) not in trace and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertEqual([v for kind, v in trace if kind == 'speed'], [.5, 1., 0.])
                    time.sleep(.15)
                reject = True
                trace.clear()
                response = call(start, StartRoute.Request(route_config_path=str(route_path)))
                self.assertTrue(response.success, response.message)
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and call(status, GetStatus.Request()).route_status.state != 3:
                    time.sleep(.05)
                self.assertEqual([kind for kind, _ in trace if kind != 'speed'], ['policy']*3)
        except BaseException:
            for log in logs:
                log.flush()
                log.seek(0)
                print(log.read()[-10000:])
            raise
        finally:
            for process in processes:
                process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for log in logs:
                log.close()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            nav.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
