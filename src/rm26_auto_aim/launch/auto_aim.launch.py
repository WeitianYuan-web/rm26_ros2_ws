from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    engine_path = LaunchConfiguration('engine_path')
    gimbal_control_source = LaunchConfiguration('gimbal_control_source')
    rc_min_value = LaunchConfiguration('rc_min_value')
    rc_max_value = LaunchConfiguration('rc_max_value')
    rc_mid_value = LaunchConfiguration('rc_mid_value')
    deadzone = LaunchConfiguration('deadzone')
    motor1_channel_index = LaunchConfiguration('motor1_channel_index')
    motor2_channel_index = LaunchConfiguration('motor2_channel_index')
    motor1_max_velocity = LaunchConfiguration('motor1_max_velocity')
    motor2_min_position = LaunchConfiguration('motor2_min_position')
    motor2_max_position = LaunchConfiguration('motor2_max_position')
    motor2_control_speed = LaunchConfiguration('motor2_control_speed')
    mouse_yaw_sensitivity = LaunchConfiguration('mouse_yaw_sensitivity')
    mouse_pitch_sensitivity = LaunchConfiguration('mouse_pitch_sensitivity')
    mouse_max_speed = LaunchConfiguration('mouse_max_speed')
    input_priority_timeout = LaunchConfiguration('input_priority_timeout')
    track_yaw_kp = LaunchConfiguration('track_yaw_kp')
    track_yaw_ki = LaunchConfiguration('track_yaw_ki')
    track_pitch_kp = LaunchConfiguration('track_pitch_kp')
    track_pitch_ki = LaunchConfiguration('track_pitch_ki')
    track_exit_timeout = LaunchConfiguration('track_exit_timeout')
    kf_q_angle = LaunchConfiguration('kf_q_angle')
    kf_q_velocity = LaunchConfiguration('kf_q_velocity')
    kf_r_yaw = LaunchConfiguration('kf_r_yaw')
    kf_r_pitch = LaunchConfiguration('kf_r_pitch')
    target_match_threshold = LaunchConfiguration('target_match_threshold')
    target_lost_timeout = LaunchConfiguration('target_lost_timeout')
    track_yaw_offset = LaunchConfiguration('track_yaw_offset')
    track_pitch_offset = LaunchConfiguration('track_pitch_offset')
    bullet_velocity = LaunchConfiguration('bullet_velocity')
    gravity = LaunchConfiguration('gravity')
    aim_offset_x_px = LaunchConfiguration('aim_offset_x_px')
    aim_offset_y_px = LaunchConfiguration('aim_offset_y_px')

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value='/home/linkerhand/RMUL2026/rm26_ros2_ws/src/rm26_auto_aim/model/yolo26n_rm_500_n.engine',
        description='TensorRT engine 文件路径'
    )
    gimbal_control_source_arg = DeclareLaunchArgument(
        'gimbal_control_source',
        default_value='hybrid',
        description='自瞄输入源兜底值: remote/mouse/hybrid（优先由 /vt_remote/switches mode 决定）'
    )
    rc_min_value_arg = DeclareLaunchArgument('rc_min_value', default_value='364')
    rc_max_value_arg = DeclareLaunchArgument('rc_max_value', default_value='1684')
    rc_mid_value_arg = DeclareLaunchArgument('rc_mid_value', default_value='1024')
    deadzone_arg = DeclareLaunchArgument('deadzone', default_value='2')
    motor1_channel_index_arg = DeclareLaunchArgument('motor1_channel_index', default_value='0')
    motor2_channel_index_arg = DeclareLaunchArgument('motor2_channel_index', default_value='1')
    motor1_max_velocity_arg = DeclareLaunchArgument('motor1_max_velocity', default_value='8.0')
    motor2_min_position_arg = DeclareLaunchArgument('motor2_min_position', default_value='-0.21')
    motor2_max_position_arg = DeclareLaunchArgument('motor2_max_position', default_value='0.31')
    motor2_control_speed_arg = DeclareLaunchArgument('motor2_control_speed', default_value='4.0')
    mouse_yaw_sensitivity_arg = DeclareLaunchArgument('mouse_yaw_sensitivity', default_value='0.02')
    mouse_pitch_sensitivity_arg = DeclareLaunchArgument('mouse_pitch_sensitivity', default_value='0.02')
    mouse_max_speed_arg = DeclareLaunchArgument('mouse_max_speed', default_value='1500')
    input_priority_timeout_arg = DeclareLaunchArgument('input_priority_timeout', default_value='0.3')
    track_yaw_kp_arg = DeclareLaunchArgument('track_yaw_kp', default_value='0.3')
    track_yaw_ki_arg = DeclareLaunchArgument('track_yaw_ki', default_value='0.05')
    track_pitch_kp_arg = DeclareLaunchArgument('track_pitch_kp', default_value='-0.3')
    track_pitch_ki_arg = DeclareLaunchArgument('track_pitch_ki', default_value='-0.05')
    track_exit_timeout_arg = DeclareLaunchArgument('track_exit_timeout', default_value='0.5')
    kf_q_angle_arg = DeclareLaunchArgument('kf_q_angle', default_value='0.01',
                                           description='KF角度过程噪声')
    kf_q_velocity_arg = DeclareLaunchArgument('kf_q_velocity', default_value='0.1',
                                              description='KF角速度过程噪声')
    kf_r_yaw_arg = DeclareLaunchArgument('kf_r_yaw', default_value='0.005',
                                         description='KF Yaw观测噪声')
    kf_r_pitch_arg = DeclareLaunchArgument('kf_r_pitch', default_value='0.005',
                                           description='KF Pitch观测噪声')
    target_match_threshold_arg = DeclareLaunchArgument('target_match_threshold', default_value='0.3',
                                                       description='目标匹配最大角度误差 rad')
    target_lost_timeout_arg = DeclareLaunchArgument('target_lost_timeout', default_value='0.3',
                                                    description='目标丢失后继续预测时长 s')
    track_yaw_offset_arg = DeclareLaunchArgument('track_yaw_offset', default_value='0.0')
    track_pitch_offset_arg = DeclareLaunchArgument('track_pitch_offset', default_value='0.05')
    bullet_velocity_arg = DeclareLaunchArgument('bullet_velocity', default_value='10.0')
    gravity_arg = DeclareLaunchArgument('gravity', default_value='9.81')
    aim_offset_x_px_arg = DeclareLaunchArgument('aim_offset_x_px', default_value='-30.0')
    aim_offset_y_px_arg = DeclareLaunchArgument('aim_offset_y_px', default_value='0.0')

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
            'conf_threshold': 0.2,
            'show_image': False,        # 是否发布 /detector/result_img 以便 rqt 调试
            'armor_real_width': 0.135, # 装甲板真实宽度 (米)
            'aim_offset_x_px': aim_offset_x_px,
            'aim_offset_y_px': aim_offset_y_px,
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
            'gimbal_control_source': gimbal_control_source,  # 兜底参数，主切换由 /vt_remote/switches mode 决定
            'rc_min_value': rc_min_value,
            'rc_max_value': rc_max_value,
            'rc_mid_value': rc_mid_value,
            'deadzone': deadzone,
            'motor1_channel_index': motor1_channel_index,
            'motor2_channel_index': motor2_channel_index,
            'motor1_max_velocity': motor1_max_velocity,
            'motor2_min_position': motor2_min_position,
            'motor2_max_position': motor2_max_position,
            'motor2_control_speed': motor2_control_speed,
            'mouse_yaw_sensitivity': mouse_yaw_sensitivity,
            'mouse_pitch_sensitivity': mouse_pitch_sensitivity,
            'mouse_max_speed': mouse_max_speed,
            'input_priority_timeout': input_priority_timeout,
            'track_yaw_kp': track_yaw_kp,
            'track_yaw_ki': track_yaw_ki,
            'track_pitch_kp': track_pitch_kp,
            'track_pitch_ki': track_pitch_ki,
            'track_exit_timeout': track_exit_timeout,
            'kf_q_angle': kf_q_angle,
            'kf_q_velocity': kf_q_velocity,
            'kf_r_yaw': kf_r_yaw,
            'kf_r_pitch': kf_r_pitch,
            'target_match_threshold': target_match_threshold,
            'target_lost_timeout': target_lost_timeout,
            'track_yaw_offset': track_yaw_offset,
            'track_pitch_offset': track_pitch_offset,
            'bullet_velocity': bullet_velocity,
            'gravity': gravity,
        }]
    )

    return LaunchDescription([
        engine_path_arg,
        gimbal_control_source_arg,
        rc_min_value_arg,
        rc_max_value_arg,
        rc_mid_value_arg,
        deadzone_arg,
        motor1_channel_index_arg,
        motor2_channel_index_arg,
        motor1_max_velocity_arg,
        motor2_min_position_arg,
        motor2_max_position_arg,
        motor2_control_speed_arg,
        mouse_yaw_sensitivity_arg,
        mouse_pitch_sensitivity_arg,
        mouse_max_speed_arg,
        input_priority_timeout_arg,
        track_yaw_kp_arg,
        track_yaw_ki_arg,
        track_pitch_kp_arg,
        track_pitch_ki_arg,
        track_exit_timeout_arg,
        kf_q_angle_arg,
        kf_q_velocity_arg,
        kf_r_yaw_arg,
        kf_r_pitch_arg,
        target_match_threshold_arg,
        target_lost_timeout_arg,
        track_yaw_offset_arg,
        track_pitch_offset_arg,
        bullet_velocity_arg,
        gravity_arg,
        aim_offset_x_px_arg,
        aim_offset_y_px_arg,
        hk_camera_node,
        armor_detector_node,
        auto_aim_node
    ])
