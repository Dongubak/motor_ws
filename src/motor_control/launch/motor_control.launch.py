"""
motor_control.launch.py - 전체 모터 제어 시스템 런치파일

사용법:
  ros2 launch motor_control motor_control.launch.py
  ros2 launch motor_control motor_control.launch.py ifname:=enp3s0
  ros2 launch motor_control motor_control.launch.py auto_home:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('motor_control')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'motor_params.yaml'])

    ifname_arg = DeclareLaunchArgument(
        'ifname',
        default_value='enP8p1s0',
        description='EtherCAT 네트워크 인터페이스 이름 (예: enP8p1s0, enp3s0)',
    )
    auto_home_arg = DeclareLaunchArgument(
        'auto_home',
        default_value='false',
        description='시작 시 자동 호밍 여부 (미구현 — 별도 클라이언트 사용)',
    )

    motor_driver_node = Node(
        package='motor_control',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[
            config_file,
            {'ifname': LaunchConfiguration('ifname')},
        ],
    )

    return LaunchDescription([
        ifname_arg,
        auto_home_arg,
        motor_driver_node,
    ])
