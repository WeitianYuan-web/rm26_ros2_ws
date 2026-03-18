from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    engine_path = LaunchConfiguration('engine_path')
    gimbal_control_source = LaunchConfiguration('gimbal_control_source')

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value='/home/linkerhand/RMUL2026/rm26_ros2_ws/src/rm26_auto_aim/model/yolo26n_rm_500.engine',
        description='TensorRT engine 文件路径'
    )
    gimbal_control_source_arg = DeclareLaunchArgument(
        'gimbal_control_source',
        default_value='remote',
        description='自瞄输入源: remote/mouse/hybrid'
    )

    # 1. 海康威视相机节点
    hk_camera_node = Node(
        package='rm26_auto_aim',
        executable='hk_camera_node',
        name='hk_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # 在此可覆盖相机标定参数 (若有需要)
            # 'camera_info.fx': 877.4866,
            # 'camera_info.fy': 875.7676,
        }]
    )

    # 2. 装甲板检测节点 (TensorRT)
    armor_detector_node = Node(
        package='rm26_auto_aim',
        executable='armor_detector_node',
        name='armor_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'engine_path': engine_path,
            'conf_threshold': 0.5,
            'show_image': True,        # 是否发布 /detector/result_img 以便 rqt 调试
            'armor_real_width': 0.135, # 装甲板真实宽度 (米)
        }]
    )

    # 3. 自动瞄准与解算节点
    auto_aim_node = Node(
        package='rm26_auto_aim',
        executable='auto_aim_node',
        name='auto_aim_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gimbal_control_source': gimbal_control_source, # hybrid/remote/mouse
            # 'track_yaw_kp': 1.5,
            # 其他需要调参的值都可以在此覆盖...
        }]
    )

    return LaunchDescription([
        engine_path_arg,
        gimbal_control_source_arg,
        hk_camera_node,
        armor_detector_node,
        auto_aim_node
    ])
