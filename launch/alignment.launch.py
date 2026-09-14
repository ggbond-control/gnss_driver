import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('gnss_driver')
    dev = LaunchConfiguration('device')
    start_driver = LaunchConfiguration('start_driver')
    export_polyline = LaunchConfiguration('export_polyline')

    # 根据 device 参数直接加载对应配置文件：g60_alignment.yaml / g90_alignment.yaml / d1m_alignment.yaml
    alignment_config = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression(["'", dev, "_alignment.yaml'"])
    ])

    # 对应设备的驱动参数文件
    driver_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'devices',
        PythonExpression(["'", dev, ".yaml'"])
    ])

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备型号：g60 (单点GNSS)、g90 (UM982 RTK) 或 d1m (RTK桥接)'),
        DeclareLaunchArgument('start_driver', default_value='false', description='是否随同启动对应设备的驱动节点'),
        DeclareLaunchArgument('export_polyline', default_value='true', description='是否启动轨迹导出节点'),

        # 可选启动设备驱动节点
        Node(
            package='gnss_driver',
            executable='g60_driver',
            name='gnss_device',
            output='screen',
            parameters=[driver_config],
            condition=IfCondition(PythonExpression(["'", start_driver, "' == 'true' and '", dev, "' == 'g60'"]))
        ),
        Node(
            package='gnss_driver',
            executable='g90_driver',
            name='gnss_device',
            output='screen',
            parameters=[driver_config],
            condition=IfCondition(PythonExpression(["'", start_driver, "' == 'true' and '", dev, "' == 'g90'"]))
        ),
        Node(
            package='gnss_driver',
            executable='d1m_bridge',
            name='gnss_device',
            output='screen',
            parameters=[driver_config],
            condition=IfCondition(PythonExpression(["'", start_driver, "' == 'true' and '", dev, "' == 'd1m'"]))
        ),

        # 核心：直接加载设备对应的对齐参数文件
        Node(
            package='gnss_driver',
            executable='gnss_alignment',
            name='gnss_alignment',
            output='screen',
            parameters=[alignment_config]
        ),

        # 轨迹记录节点
        Node(
            package='gnss_driver',
            executable='gnss_trajectory',
            name='gnss_trajectory',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'trajectory.yaml']),
                {'fix_topic': '/fix', 'output_filename': 'fix_trajectory.ovjsn',
                 'polyline_name': 'fix'},
            ],
            condition=IfCondition(export_polyline)
        ),
    ])
