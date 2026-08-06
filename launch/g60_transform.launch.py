from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('g60_driver')
    config = os.path.join(share, 'config', 'g60_transform.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('export_polyline', default_value='true',
                              description='Write /fix_from_odom to data/fix_from_odom_trajectory.ovjsn.'),
        Node(package='g60_driver', executable='g60_transform', name='g60_transform',
             output='screen', parameters=[config]),
        Node(package='g60_driver', executable='g60_fix_to_polyline',
             name='g60_fix_from_odom_to_polyline', output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('export_polyline'))),
    ])
