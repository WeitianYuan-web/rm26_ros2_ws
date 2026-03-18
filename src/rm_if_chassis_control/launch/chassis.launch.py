from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 底盘串口通讯节点
    chassis_serial_node = Node(
        package='rm_if_chassis_control',
        executable='chassis_serial_node',
        name='chassis_serial_node',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'baud_rate': 460800},
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
            {'fire_control_source': 'hybrid'},
            {'input_priority_timeout': 0.3},
            {'deadzone': 2},
            {'gimbal_yaw_zero_offset': 0.0},
        ]
    )

    return LaunchDescription([
        chassis_serial_node,
        chassis_control_node,
    ])
