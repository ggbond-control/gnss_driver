import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('gnss_driver')
    dev = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备配置：g60、g90 或 d1m'),
        DeclareLaunchArgument('start_driver', default_value='false', description='是否随同启动对应设备的驱动节点'),
        DeclareLaunchArgument('export_polyline', default_value='true', description='是否启动轨迹导出节点'),
        Node(package='gnss_driver', executable='g60_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g60.yaml')],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_driver'), "' == 'true' and '", dev, "' == 'g60'"]))),
        Node(package='gnss_driver', executable='g90_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g90.yaml')],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_driver'), "' == 'true' and '", dev, "' == 'g90'"]))),
        Node(package='gnss_driver', executable='d1m_bridge', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'd1m.yaml')],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_driver'), "' == 'true' and '", dev, "' == 'd1m'"]))),
        Node(package='gnss_driver', executable='gnss_alignment', name='gnss_alignment', output='screen',
             parameters=[os.path.join(share, 'config', 'alignment.yaml')],
             condition=IfCondition(PythonExpression(["'", dev, "' != 'd1m'"]))),
        Node(package='gnss_driver', executable='gnss_alignment', name='gnss_alignment', output='screen',
             parameters=[os.path.join(share, 'config', 'd1m_alignment.yaml')],
             condition=IfCondition(PythonExpression(["'", dev, "' == 'd1m'"]))),
        Node(package='gnss_driver', executable='gnss_trajectory', name='gnss_trajectory', output='screen',
             parameters=[os.path.join(share, 'config', 'trajectory.yaml')],
             condition=IfCondition(LaunchConfiguration('export_polyline'))),
    ])
