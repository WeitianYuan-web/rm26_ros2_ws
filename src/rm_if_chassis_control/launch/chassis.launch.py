from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    control_mode = LaunchConfiguration('control_mode')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='remote',
        description='底盘发射控制输入源: remote/mouse/hybrid'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='底盘串口设备'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='460800',
        description='底盘串口波特率'
    )

    # 底盘串口通讯节点
    chassis_serial_node = Node(
        package='rm_if_chassis_control',
        executable='chassis_serial_node',
        name='chassis_serial_node',
        output='screen',
        parameters=[
            {'serial_port': serial_port},
            {'baud_rate': baud_rate},
        ]
    )

    # 底盘控制节点
    chassis_control_node = Node(
        package='rm_if_chassis_control',
        executable='chassis_control_node',
        name='chassis_control_node',
        output='screen',
        parameters=[
            {'max_linear_accel': 4.0},
            {'max_angular_accel': 8.0},
            {'max_linear_vel': 1.5},
            {'max_angular_vel': 4.0},
            {'min_linear_vel': 1.0},
            {'fire_control_source': control_mode},
            {'input_priority_timeout': 0.3},
            {'deadzone': 2},
            {'gimbal_yaw_zero_offset': -1.5758},
        ]
    )

    return LaunchDescription([
        control_mode_arg,
        serial_port_arg,
        baud_rate_arg,
        chassis_serial_node,
        chassis_control_node,
    ])
