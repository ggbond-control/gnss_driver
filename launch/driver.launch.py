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
    start_ntrip = LaunchConfiguration('ntrip')
    ntrip_config = LaunchConfiguration('ntrip_config')

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备配置：g60、g90 或 d1m'),
        DeclareLaunchArgument('ntrip', default_value='false', description='是否随同启动 NTRIP 差分客户端 (用于 G90 RTK)'),
        DeclareLaunchArgument('ntrip_config', default_value=os.path.join(share, 'config', 'ntrip.yaml'), description='NTRIP 配置文件路径'),

        # 设备驱动节点
        Node(package='gnss_driver', executable='g60_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g60.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'g60'"]))),
        Node(package='gnss_driver', executable='d1m_bridge', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'd1m.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'd1m'"]))),
        Node(package='gnss_driver', executable='g90_driver', name='gnss_device', output='screen',
             parameters=[os.path.join(share, 'config', 'devices', 'g90.yaml')],
             condition=IfCondition(PythonExpression(["'", device, "' == 'g90'"]))),

        # 可选：伴随启动 NTRIP 差分注入节点 (将 RTCM3 注入到 /dev/wheeltec_rtk)
        Node(package='gnss_driver', executable='ntrip_client', name='ntrip_client', output='screen',
             parameters=[ntrip_config],
             condition=IfCondition(PythonExpression(["'", start_ntrip, "' == 'true' and '", device, "' == 'g90'"]))),
    ])
