from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('g60_driver'), 'config', 'g60_serial.yaml')
    return LaunchDescription([
        Node(package='g60_driver', executable='g60_serial', name='g60_serial',
             output='screen', parameters=[config]),
    ])
