import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('gnss_driver')
    device = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备配置：g60、g90 或 d1m'),
        DeclareLaunchArgument('export_polyline', default_value='true', description='是否启动轨迹导出节点'),
        Node(package='gnss_driver', executable='gnss_transform', name='gnss_transform', output='screen',
             parameters=[os.path.join(share, 'config', 'transform.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' != 'd1m'"]))),
        Node(package='gnss_driver', executable='gnss_transform', name='gnss_transform', output='screen',
             parameters=[os.path.join(share, 'config', 'd1m_transform.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'd1m'"]))),
        Node(package='gnss_driver', executable='gnss_trajectory', name='gnss_trajectory', output='screen',
             parameters=[os.path.join(share, 'config', 'trajectory.yaml')],
             condition=IfCondition(LaunchConfiguration('export_polyline'))),
    ])
