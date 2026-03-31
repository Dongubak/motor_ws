"""
homing_only.launch.py - 호밍만 수행하는 런치파일

motor_driver_node를 기동한 뒤 /motor/homing 액션 Goal을 자동 전송하는
간단한 런치파일. 호밍 완료 후 노드는 계속 실행된다.

사용법:
  ros2 launch motor_control homing_only.launch.py
  ros2 launch motor_control homing_only.launch.py ifname:=enp3s0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share  = FindPackageShare('motor_control')
    config_file = PathJoinSubstitution([pkg_share, 'config', 'motor_params.yaml'])

    ifname_arg = DeclareLaunchArgument(
        'ifname',
        default_value='eth0',
        description='EtherCAT 네트워크 인터페이스',
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

    # 드라이버 기동 후 3초 뒤 호밍 액션 클라이언트 실행
    homing_client_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='motor_control',
                executable='homing_client',
                name='homing_client',
                output='screen',
                parameters=[{'force_rehome': False}],
            )
        ],
    )

    return LaunchDescription([
        ifname_arg,
        motor_driver_node,
        # homing_client_node,  # 필요 시 주석 해제
    ])
