import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('gnss_driver')
    device = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备配置：g60、g90 或 d1m'),
        Node(package='gnss_driver', executable='g60_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g60.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'g60'"]))),
        Node(package='gnss_driver', executable='d1m_bridge', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'd1m.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'd1m'"]))),
        Node(package='gnss_driver', executable='g90_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g90.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'g90'"]))),
    ])
