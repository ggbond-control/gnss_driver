from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('g60_driver')
    driver_config = os.path.join(share, 'config', 'g60_driver.yaml')
    alignment_config = os.path.join(share, 'config', 'g60_alignment.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('start_g60_fix', default_value='false',
                              description='Start G60 serial input and publish /fix.'),
        DeclareLaunchArgument('export_polyline', default_value='true',
                              description='Write the raw and aligned OVJSN trajectories.'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start RViz with the alignment display.'),
        Node(package='g60_driver', executable='g60_serial', name='g60_serial', output='screen',
             parameters=[driver_config], condition=IfCondition(LaunchConfiguration('start_g60_fix'))),
        Node(package='g60_driver', executable='g60_gps_odom_alignment',
             name='g60_gps_odom_alignment', output='screen', parameters=[alignment_config]),
        Node(package='g60_driver', executable='g60_fix_to_polyline',
             name='g60_fix_to_polyline', output='screen', parameters=[alignment_config],
             condition=IfCondition(LaunchConfiguration('export_polyline'))),
        Node(package='g60_driver', executable='g60_fix_to_polyline',
             name='g60_fix_odom_to_polyline', output='screen', parameters=[alignment_config],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('export_polyline'), "' == 'true' "]))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'gps_odom_alignment.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
