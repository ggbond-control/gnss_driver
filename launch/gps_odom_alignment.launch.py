from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('g60_driver')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(package='g60_driver', executable='g60_gps_odom_alignment',
             name='g60_gps_odom_alignment', output='screen',
             parameters=[os.path.join(share, 'config', 'gps_odom_alignment.yaml')]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'gps_odom_alignment.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
