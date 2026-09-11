import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('gnss_driver')
    device = LaunchConfiguration('device')
    export_polyline = LaunchConfiguration('export_polyline')

    # 根据 device 参数直接加载对应配置文件：g60_transform.yaml / g90_transform.yaml / d1m_transform.yaml
    transform_config = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression(["'", device, "_transform.yaml'"])
    ])

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='g60', description='设备型号：g60 (单点GNSS)、g90 (UM982 RTK) 或 d1m (RTK桥接)'),
        DeclareLaunchArgument('export_polyline', default_value='true', description='是否启动轨迹导出节点'),

        # 核心：直接加载设备对应的变换参数文件
        Node(
            package='gnss_driver',
            executable='gnss_transform',
            name='gnss_transform',
            output='screen',
            parameters=[transform_config]
        ),

        # 轨迹记录节点
        Node(
            package='gnss_driver',
            executable='gnss_trajectory',
            name='gnss_trajectory',
            output='screen',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'trajectory.yaml'])],
            condition=IfCondition(export_polyline)
        ),
    ])
