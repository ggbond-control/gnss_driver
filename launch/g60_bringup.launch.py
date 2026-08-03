from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('g60_driver')
    return LaunchDescription([
        Node(package='g60_driver', executable='g60_serial', name='g60_serial', output='screen',
             parameters=[os.path.join(share, 'config', 'g60_serial.yaml')]),
        Node(package='g60_driver', executable='g60_trajectory', name='g60_trajectory', output='screen',
             parameters=[os.path.join(share, 'config', 'g60_trajectory.yaml')]),
    ])
