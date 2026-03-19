/**
 * @file auto_aim_node.cpp
 * @brief 自动瞄准控制节点
 *
 * 负责遥控器/鼠标手动控制与自动瞄准追踪逻辑，将计算结果以统一格式
 * 通过 /gimbal/cmd 话题下发给底层的 gimbal_control_node。
 *
 * /gimbal/cmd 消息格式 (std_msgs/Float64MultiArray):
 *   data[0]: mode       — 0.0=普通模式, 1.0=追踪模式
 *   data[1]: yaw_cmd    — 普通模式: Yaw 速度 (rad/s); 追踪模式: Yaw 位置增量 (rad)
 *   data[2]: pitch_cmd  — 普通模式: Pitch 绝对目标位置 (rad); 追踪模式: Pitch 位置增量 (rad)
 *
 * 输入话题:
 *   /vt_remote/channels  — 遥控器通道值
 *   /vt_remote/mouse     — 图传遥控器鼠标输入
 *   /vt_remote/switches  — 遥控器开关（含 trigger 按键）
 *   /detector/armors     — 视觉检测到的装甲板位姿
 */

#include <auto_aim_interfaces/msg/armors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief 单轴匀速运动卡尔曼滤波器
 *
 * 状态量: x = [θ, ω]
 *   θ — 角度 (rad)
 *   ω — 角速度 (rad/s)
 * 观测量: z = θ
 * 状态转移: θ(k+1) = θ(k) + ω(k)*dt, ω(k+1) = ω(k)
 * 过程噪声 Q = diag(q_angle, q_vel)
 * 观测噪声 R = r
 */
struct KalmanAngle {
  double angle{0.0};    ///< 角度估计
  double velocity{0.0}; ///< 角速度估计
  double P00{1.0}, P01{0.0};
  double P10{0.0}, P11{1.0};

  /** @brief 用初始角度重置状态（速度归零，协方差复位） */
  void reset(double init_angle) {
    angle    = init_angle;
    velocity = 0.0;
    P00 = 1.0; P01 = 0.0;
    P10 = 0.0; P11 = 1.0;
  }

  /** @brief 预测步骤 */
  void predict(double dt, double q_angle, double q_vel) {
    angle += velocity * dt;
    double p00 = P00 + dt * (P10 + P01 + dt * P11) + q_angle;
    double p01 = P01 + dt * P11;
    double p10 = P10 + dt * P11;
    double p11 = P11 + q_vel;
    P00 = p00; P01 = p01;
    P10 = p10; P11 = p11;
  }

  /** @brief 更新步骤（观测为角度值） */
  void update(double z, double r) {
    double S  = P00 + r;
    double K0 = P00 / S;
    double K1 = P10 / S;
    double innov = z - angle;
    angle    += K0 * innov;
    velocity += K1 * innov;
    double p00 = (1.0 - K0) * P00;
    double p01 = (1.0 - K0) * P01;
    double p10 = P10 - K1 * P00;
    double p11 = P11 - K1 * P01;
    P00 = p00; P01 = p01;
    P10 = p10; P11 = p11;
  }
};

/**
 * @brief 云台输入源模式
 */
enum class GimbalControlSource {
  MOUSE = 0,  ///< 仅鼠标
  REMOTE = 1, ///< 仅遥控器
  HYBRID = 2  ///< 混合模式（鼠标优先窗口）
};

/**
 * @brief 自动瞄准控制节点
 *
 * 整合遥控/鼠标/视觉自瞄三种输入源，计算 Yaw/Pitch 控制量，
 * 通过 /gimbal/cmd 话题向底层 gimbal_control_node 发送指令。
 */
class AutoAimNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，声明并读取参数，初始化 ROS 通信与控制定时器
   */
  AutoAimNode() : rclcpp::Node("auto_aim_node") {

    // 遥控器参数
    this->declare_parameter<int>("rc_min_value", 364);
    this->declare_parameter<int>("rc_max_value", 1684);
    this->declare_parameter<int>("rc_mid_value", 1024);
    this->declare_parameter<int>("deadzone", 2);
    this->declare_parameter<int>("motor1_channel_index", 0);
    this->declare_parameter<int>("motor2_channel_index", 1);
    this->declare_parameter<double>("motor1_max_velocity", 8.0);
    this->declare_parameter<bool>("motor1_position_limit_enabled", true);
    this->declare_parameter<double>("motor1_min_position", -4.0 * M_PI);
    this->declare_parameter<double>("motor1_max_position", 4.0 * M_PI);
    this->declare_parameter<bool>("chassis_gyro_compensation_enabled", true);
    this->declare_parameter<double>("chassis_gyro_compensation_gain", 1.0);
    this->declare_parameter<double>("chassis_gyro_compensation_sign", 1.0);
    this->declare_parameter<double>("chassis_gyro_filter_alpha", 0.9);
    this->declare_parameter<bool>("chassis_gyro_apply_in_tracking", false);
    this->declare_parameter<double>("chassis_rotation_activation_threshold", 0.08);
    this->declare_parameter<bool>("gimbal_gyro_stabilization_enabled", true);
    this->declare_parameter<double>("gimbal_gyro_stabilization_gain", 1.0);
    this->declare_parameter<double>("gimbal_gyro_filter_alpha", 0.9);
    this->declare_parameter<bool>("gimbal_gyro_apply_in_tracking", false);

    // 电机2 (Pitch) 限位（用于遥控/鼠标模式下的位置映射与限幅）
    this->declare_parameter<double>("motor2_min_position", -0.21);
    this->declare_parameter<double>("motor2_max_position", 0.31);
    this->declare_parameter<double>("motor2_control_speed", 4.0);

    // 鼠标控制参数
    this->declare_parameter<double>("mouse_yaw_sensitivity", 0.02);
    this->declare_parameter<double>("mouse_pitch_sensitivity", 0.02);
    this->declare_parameter<int>("mouse_max_speed", 1500);
    this->declare_parameter<std::string>("gimbal_control_source", "hybrid");
    this->declare_parameter<double>("input_priority_timeout", 0.3);

    // 追踪模式参数（增量式PI控制器）
    this->declare_parameter<double>("track_yaw_kp", 1.5);
    this->declare_parameter<double>("track_yaw_ki", 0.4);
    this->declare_parameter<double>("track_pitch_kp", -0.6);
    this->declare_parameter<double>("track_pitch_ki", -0.2);
    this->declare_parameter<double>("track_exit_timeout", 0.5);

    // 卡尔曼滤波参数
    this->declare_parameter<double>("kf_q_angle",   0.01);  ///< 角度过程噪声
    this->declare_parameter<double>("kf_q_velocity", 1.0);  ///< 角速度过程噪声
    this->declare_parameter<double>("kf_r_yaw",     0.005); ///< Yaw 观测噪声
    this->declare_parameter<double>("kf_r_pitch",   0.005); ///< Pitch 观测噪声
    // 目标锁定参数
    this->declare_parameter<double>("target_match_threshold", 0.3); ///< 最大匹配角度误差 rad
    this->declare_parameter<double>("target_lost_timeout",    0.3); ///< 目标丢失后继续预测时长 s

    // 追踪模式 Yaw/Pitch 瞄准偏移校准（硬件偏差补偿）
    this->declare_parameter<double>("track_yaw_offset", 0.0);
    this->declare_parameter<double>("track_pitch_offset", 0.1);

    // 弹道下坠补偿参数
    this->declare_parameter<double>("bullet_velocity", 10.0);
    this->declare_parameter<double>("gravity", 9.81);

    // 普通模式控制定时器频率
    this->declare_parameter<int>("control_rate", 200);

    // 读取参数
    rc_min_value_ = this->get_parameter("rc_min_value").as_int();
    rc_max_value_ = this->get_parameter("rc_max_value").as_int();
    rc_mid_value_ = this->get_parameter("rc_mid_value").as_int();
    deadzone_ = this->get_parameter("deadzone").as_int();
    motor1_channel_index_ = this->get_parameter("motor1_channel_index").as_int();
    motor2_channel_index_ = this->get_parameter("motor2_channel_index").as_int();
    motor1_max_velocity_ = this->get_parameter("motor1_max_velocity").as_double();
    motor1_position_limit_enabled_ = this->get_parameter("motor1_position_limit_enabled").as_bool();
    motor1_min_position_ = this->get_parameter("motor1_min_position").as_double();
    motor1_max_position_ = this->get_parameter("motor1_max_position").as_double();
    chassis_gyro_compensation_enabled_ =
        this->get_parameter("chassis_gyro_compensation_enabled").as_bool();
    chassis_gyro_compensation_gain_ =
        this->get_parameter("chassis_gyro_compensation_gain").as_double();
    chassis_gyro_compensation_sign_ =
        this->get_parameter("chassis_gyro_compensation_sign").as_double();
    chassis_gyro_compensation_sign_ =
        (chassis_gyro_compensation_sign_ >= 0.0) ? 1.0 : -1.0;
    chassis_gyro_filter_alpha_ = this->get_parameter("chassis_gyro_filter_alpha").as_double();
    chassis_gyro_filter_alpha_ = std::max(0.0, std::min(1.0, chassis_gyro_filter_alpha_));
    chassis_gyro_apply_in_tracking_ =
        this->get_parameter("chassis_gyro_apply_in_tracking").as_bool();
    chassis_rotation_activation_threshold_ =
        this->get_parameter("chassis_rotation_activation_threshold").as_double();
    if (chassis_rotation_activation_threshold_ < 0.0) {
      chassis_rotation_activation_threshold_ = 0.0;
    }
    gimbal_gyro_stabilization_enabled_ =
        this->get_parameter("gimbal_gyro_stabilization_enabled").as_bool();
    gimbal_gyro_stabilization_gain_ =
        this->get_parameter("gimbal_gyro_stabilization_gain").as_double();
    gimbal_gyro_filter_alpha_ = this->get_parameter("gimbal_gyro_filter_alpha").as_double();
    gimbal_gyro_filter_alpha_ = std::max(0.0, std::min(1.0, gimbal_gyro_filter_alpha_));
    gimbal_gyro_apply_in_tracking_ =
        this->get_parameter("gimbal_gyro_apply_in_tracking").as_bool();
    if (motor1_min_position_ > motor1_max_position_) {
      RCLCPP_WARN(this->get_logger(),
                  "motor1_min_position(%.4f) > motor1_max_position(%.4f)，将自动交换",
                  motor1_min_position_, motor1_max_position_);
      std::swap(motor1_min_position_, motor1_max_position_);
    }

    motor2_min_position_ = this->get_parameter("motor2_min_position").as_double();
    motor2_max_position_ = this->get_parameter("motor2_max_position").as_double();
    motor2_control_speed_ = this->get_parameter("motor2_control_speed").as_double();

    mouse_yaw_sensitivity_ = this->get_parameter("mouse_yaw_sensitivity").as_double();
    mouse_pitch_sensitivity_ = this->get_parameter("mouse_pitch_sensitivity").as_double();
    mouse_max_speed_ = this->get_parameter("mouse_max_speed").as_int();
    const std::string src_str = this->get_parameter("gimbal_control_source").as_string();
    gimbal_control_source_ = parse_gimbal_control_source(src_str);
    input_priority_timeout_ = this->get_parameter("input_priority_timeout").as_double();

    track_yaw_kp_ = this->get_parameter("track_yaw_kp").as_double();
    track_yaw_ki_ = this->get_parameter("track_yaw_ki").as_double();
    track_pitch_kp_ = this->get_parameter("track_pitch_kp").as_double();
    track_pitch_ki_ = this->get_parameter("track_pitch_ki").as_double();
    track_exit_timeout_ = this->get_parameter("track_exit_timeout").as_double();

    kf_q_angle_   = this->get_parameter("kf_q_angle").as_double();
    kf_q_velocity_ = this->get_parameter("kf_q_velocity").as_double();
    kf_r_yaw_     = this->get_parameter("kf_r_yaw").as_double();
    kf_r_pitch_   = this->get_parameter("kf_r_pitch").as_double();
    target_match_threshold_ = this->get_parameter("target_match_threshold").as_double();
    target_lost_timeout_    = this->get_parameter("target_lost_timeout").as_double();

    track_yaw_offset_ = this->get_parameter("track_yaw_offset").as_double();
    track_pitch_offset_ = this->get_parameter("track_pitch_offset").as_double();

    bullet_velocity_ = this->get_parameter("bullet_velocity").as_double();
    gravity_ = this->get_parameter("gravity").as_double();

    int control_rate = this->get_parameter("control_rate").as_int();

    RCLCPP_INFO(this->get_logger(), "自动瞄准控制节点配置:");
    RCLCPP_INFO(this->get_logger(), "  遥控: RC范围%d~%d(中值%d), ch%d/ch%d, 死区%d",
                rc_min_value_, rc_max_value_, rc_mid_value_,
                motor1_channel_index_, motor2_channel_index_, deadzone_);
    RCLCPP_INFO(this->get_logger(), "  鼠标: yaw灵敏度=%.4f, pitch灵敏度=%.4f, 限幅=%d, 优先窗口=%.2fs",
                mouse_yaw_sensitivity_, mouse_pitch_sensitivity_,
                mouse_max_speed_, input_priority_timeout_);
    RCLCPP_INFO(this->get_logger(), "  输入源: %s", gimbal_control_source_to_string(gimbal_control_source_));
    RCLCPP_INFO(this->get_logger(), "  追踪PI: Yaw(Kp=%.2f,Ki=%.2f), Pitch(Kp=%.2f,Ki=%.2f), 退出=%.1fs",
                track_yaw_kp_, track_yaw_ki_, track_pitch_kp_, track_pitch_ki_, track_exit_timeout_);
    RCLCPP_INFO(this->get_logger(), "  KF: q_angle=%.4f, q_vel=%.4f, r_yaw=%.4f, r_pitch=%.4f",
                kf_q_angle_, kf_q_velocity_, kf_r_yaw_, kf_r_pitch_);
    RCLCPP_INFO(this->get_logger(), "  目标锁定: 匹配阈值=%.2f rad, 丢失超时=%.2f s",
                target_match_threshold_, target_lost_timeout_);
    RCLCPP_INFO(this->get_logger(), "  电机1限幅: %s, 范围=[%.4f, %.4f] rad",
                motor1_position_limit_enabled_ ? "启用" : "关闭",
                motor1_min_position_, motor1_max_position_);
    RCLCPP_INFO(this->get_logger(),
                "  底盘角速度补偿: %s, 增益=%.3f, 符号=%.0f, alpha=%.3f, 旋转触发阈值=%.3frad/s, tracking中%s",
                chassis_gyro_compensation_enabled_ ? "启用" : "关闭",
                chassis_gyro_compensation_gain_,
                chassis_gyro_compensation_sign_,
                chassis_gyro_filter_alpha_,
                chassis_rotation_activation_threshold_,
                chassis_gyro_apply_in_tracking_ ? "启用" : "关闭");
    RCLCPP_INFO(this->get_logger(),
                "  云台角速度稳定: %s, 增益=%.3f, alpha=%.3f, tracking中%s",
                gimbal_gyro_stabilization_enabled_ ? "启用" : "关闭",
                gimbal_gyro_stabilization_gain_,
                gimbal_gyro_filter_alpha_,
                gimbal_gyro_apply_in_tracking_ ? "启用" : "关闭");
    RCLCPP_INFO(this->get_logger(), "  偏移校准: Yaw=%.4f rad, Pitch=%.4f rad",
                track_yaw_offset_, track_pitch_offset_);
    RCLCPP_INFO(this->get_logger(), "  弹道补偿: 弹速=%.1f m/s, 重力=%.2f m/s²",
                bullet_velocity_, gravity_);

    // 发布者：向 gimbal_control_node 发送控制指令
    gimbal_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gimbal/cmd", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // 订阅遥控器通道
    rc_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/channels", qos,
        std::bind(&AutoAimNode::rc_callback, this, std::placeholders::_1));

    // 订阅鼠标输入
    mouse_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/mouse", qos,
        std::bind(&AutoAimNode::mouse_callback, this, std::placeholders::_1));

    // 订阅开关（trigger 用于追踪模式切换）
    switches_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/switches", qos,
        std::bind(&AutoAimNode::switches_callback, this, std::placeholders::_1));

    // 订阅键盘位图（mouse 模式使用 C 键长按触发追踪）
    keyboard_subscription_ = this->create_subscription<std_msgs::msg::UInt16>(
        "/vt_remote/keyboard", qos,
        std::bind(&AutoAimNode::keyboard_callback, this, std::placeholders::_1));

    // 订阅装甲板检测结果（追踪模式 PI 控制输入）
    armors_subscription_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors",
        rclcpp::SensorDataQoS(),
        std::bind(&AutoAimNode::armors_callback, this, std::placeholders::_1));

    // 订阅电机实际位置
    motor1_pos_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor1/multi_turn_position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          motor1_actual_position_ = msg->data;
          motor1_actual_valid_ = true;
        });
    motor2_pos_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor2/multi_turn_position", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          motor2_actual_position_ = msg->data;
          motor2_actual_valid_ = true;
        });
    chassis_gyro_z_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/chassis/gyro_z", 20,
        std::bind(&AutoAimNode::chassis_gyro_callback, this, std::placeholders::_1));
    gimbal_gyro_z_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/gimbal/gyro_z", 20,
        std::bind(&AutoAimNode::gimbal_gyro_callback, this, std::placeholders::_1));

    // 控制定时器（位置积分 + /gimbal/cmd 发布）
    int period_ms = 1000 / control_rate;
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&AutoAimNode::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "自动瞄准控制节点已启动，控制频率=%dHz", control_rate);
  }

private:
  /**
   * @brief 根据 /vt_remote/switches 的 mode 获取当前生效输入源
   * @return 生效输入源
   */
  GimbalControlSource get_active_gimbal_control_source() const {
    if (mode_switch_valid_) {
      if (mode_switch_ == 0) {
        return GimbalControlSource::MOUSE;
      }
      if (mode_switch_ == 1) {
        return GimbalControlSource::REMOTE;
      }
    }
    return gimbal_control_source_;
  }

  /**
   * @brief 解析输入源参数字符串
   * @param source 参数字符串
   * @return 输入源枚举值
   */
  GimbalControlSource parse_gimbal_control_source(const std::string &source) const {
    if (source == "mouse") return GimbalControlSource::MOUSE;
    if (source == "remote") return GimbalControlSource::REMOTE;
    if (source == "hybrid") return GimbalControlSource::HYBRID;
    RCLCPP_WARN(this->get_logger(), "未知 gimbal_control_source='%s'，回退为 hybrid", source.c_str());
    return GimbalControlSource::HYBRID;
  }

  /**
   * @brief 输入源枚举转字符串
   * @param source 输入源枚举
   * @return 对应的字符串
   */
  const char *gimbal_control_source_to_string(GimbalControlSource source) const {
    switch (source) {
      case GimbalControlSource::MOUSE:  return "mouse";
      case GimbalControlSource::REMOTE: return "remote";
      default:                          return "hybrid";
    }
  }

  /**
   * @brief 将遥控器值线性映射为速度
   * @param rc_value 遥控器原始值
   * @param max_velocity 最大速度 rad/s
   * @return 映射后的速度 [-max_velocity, +max_velocity]
   */
  double map_rc_to_velocity(int rc_value, double max_velocity) {
    int clamped = std::max(rc_min_value_, std::min(rc_max_value_, rc_value));
    if (std::abs(clamped - rc_mid_value_) < deadzone_) {
      return 0.0;
    }
    double normalized = static_cast<double>(clamped - rc_mid_value_) /
                        static_cast<double>(rc_max_value_ - rc_mid_value_);
    normalized = std::max(-1.0, std::min(1.0, normalized));
    return normalized * max_velocity;
  }

  /**
   * @brief 将遥控器值线性映射为位置
   * @param rc_value 遥控器原始值
   * @param min_pos 目标最小位置 rad
   * @param max_pos 目标最大位置 rad
   * @return 映射后的目标位置 rad
   */
  double map_rc_to_pos(int rc_value, double min_pos, double max_pos) {
    int clamped = std::max(rc_min_value_, std::min(rc_max_value_, rc_value));
    if (std::abs(clamped - rc_mid_value_) < deadzone_) {
      clamped = rc_mid_value_;
    }
    double normalized = static_cast<double>(clamped - rc_min_value_) /
                        static_cast<double>(rc_max_value_ - rc_min_value_);
    return min_pos + normalized * (max_pos - min_pos);
  }

  double clamp_motor1_position(double position) const {
    if (!motor1_position_limit_enabled_) {
      return position;
    }
    return std::max(motor1_min_position_, std::min(motor1_max_position_, position));
  }

  /**
   * @brief 发布 /gimbal/cmd 绝对位置指令
   */
  void publish_cmd() {
    auto msg = std_msgs::msg::Float64MultiArray();
    double yaw_cmd = clamp_motor1_position(motor1_target_position_);
    double pitch_cmd = motor2_target_position_;
    if (tracking_mode_) {
      yaw_cmd += track_yaw_offset_;
      pitch_cmd += track_pitch_offset_;
    }
    msg.data = {yaw_cmd, pitch_cmd};
    gimbal_cmd_publisher_->publish(msg);
  }

  /**
   * @brief 遥控器通道消息回调，缓存最新遥控器数据
   * @param msg /vt_remote/channels 消息
   */
  void rc_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    int max_index = std::max(motor1_channel_index_, motor2_channel_index_);
    if (static_cast<int>(msg->data.size()) <= max_index) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "RC消息数据长度不足: %zu，需要 %d",
                           msg->data.size(), max_index + 1);
      return;
    }
    latest_rc_.assign(msg->data.begin(), msg->data.end());
    rc_data_valid_ = true;
  }

  /**
   * @brief 底盘角速度回调，一阶低通滤波 /chassis/gyro_z
   * @param msg 底盘 Z 轴角速度（rad/s）
   */
  void chassis_gyro_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    const double raw = static_cast<double>(msg->data);
    if (!std::isfinite(raw)) {
      return;
    }
    if (!chassis_gyro_filter_seeded_) {
      filtered_chassis_gyro_z_ = raw;
      chassis_gyro_filter_seeded_ = true;
      return;
    }
    filtered_chassis_gyro_z_ =
        chassis_gyro_filter_alpha_ * raw +
        (1.0 - chassis_gyro_filter_alpha_) * filtered_chassis_gyro_z_;
  }

  /**
   * @brief 云台角速度回调，一阶低通滤波 /gimbal/gyro_z
   * @param msg 云台 Z 轴角速度（rad/s）
   */
  void gimbal_gyro_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    const double raw = msg->data;
    if (!std::isfinite(raw)) {
      return;
    }
    if (!gimbal_gyro_filter_seeded_) {
      filtered_gimbal_gyro_z_ = raw;
      gimbal_gyro_filter_seeded_ = true;
      return;
    }
    filtered_gimbal_gyro_z_ =
        gimbal_gyro_filter_alpha_ * raw +
        (1.0 - gimbal_gyro_filter_alpha_) * filtered_gimbal_gyro_z_;
  }

  /**
   * @brief 鼠标消息回调，更新鼠标输入状态
   *
   * 消息格式: [mouse_x, mouse_y, mouse_z, left, right, middle]
   * @param msg /vt_remote/mouse 消息
   */
  void mouse_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    const GimbalControlSource active_source = get_active_gimbal_control_source();
    if (active_source == GimbalControlSource::REMOTE) {
      return;
    }
    if (msg->data.size() < 2) {
      return;
    }
    int cx = std::max(-mouse_max_speed_, std::min(mouse_max_speed_, static_cast<int>(msg->data[0])));
    int cy = std::max(-mouse_max_speed_, std::min(mouse_max_speed_, static_cast<int>(msg->data[1])));
    mouse_x_ = static_cast<int16_t>(cx);
    mouse_y_ = static_cast<int16_t>(cy);
    if (cx != 0 || cy != 0) {
      last_mouse_time_ = this->now();
      mouse_time_valid_ = true;
    }
  }

  /**
   * @brief 根据“追踪触发键是否按下”处理追踪模式进入/退出
   * @param pressed 当前触发键是否按下
   */
  void handle_track_button_state(bool pressed) {
    if (pressed) {
      if (!tracking_mode_) {
        tracking_mode_ = true;
        // 重置卡尔曼滤波器和 PI 状态，等待 armors_callback 重新锁定
        kf_initialized_            = false;
        kf_time_valid_             = false;
        kf_target_lost_            = false;
        kf_target_lost_time_valid_ = false;
        track_yaw_error_prev_      = 0.0;
        track_pitch_error_prev_    = 0.0;
        RCLCPP_INFO(this->get_logger(), ">>> 进入追踪模式");
        // 立即更新播种位置
        if (motor1_actual_valid_) motor1_target_position_ = clamp_motor1_position(motor1_actual_position_);
        if (motor2_actual_valid_) motor2_target_position_ = motor2_actual_position_;
        publish_cmd();
      }
      trigger_release_time_valid_ = false;
    } else {
      if (tracking_mode_) {
        if (!trigger_release_time_valid_) {
          trigger_release_time_ = std::chrono::steady_clock::now();
          trigger_release_time_valid_ = true;
        } else {
          double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - trigger_release_time_).count();
          if (elapsed > track_exit_timeout_) {
            tracking_mode_    = false;
            kf_initialized_   = false;
            kf_time_valid_    = false;
            kf_target_lost_   = false;
            RCLCPP_INFO(this->get_logger(), "<<< 退出追踪模式 (松开 %.1fs)", track_exit_timeout_);
            // 退出时更新播种位置，以防积分跳变
            if (motor1_actual_valid_) motor1_target_position_ = clamp_motor1_position(motor1_actual_position_);
            if (motor2_actual_valid_) motor2_target_position_ = motor2_actual_position_;
          }
        }
      }
    }
  }

  /**
   * @brief 按输入源模式计算当前追踪触发状态
   * @return true 表示“追踪触发键按下”
   */
  bool get_track_button_pressed() const {
    switch (get_active_gimbal_control_source()) {
      case GimbalControlSource::REMOTE:
        return trigger_pressed_;
      case GimbalControlSource::MOUSE:
        return keyboard_c_pressed_;
      case GimbalControlSource::HYBRID:
      default:
        return trigger_pressed_ || keyboard_c_pressed_;
    }
  }

  /**
   * @brief 键盘位图回调，读取 C 键状态
   *
   * C 键位: bit13 (1 << 13)
   * @param msg /vt_remote/keyboard 消息
   */
  void keyboard_callback(const std_msgs::msg::UInt16::SharedPtr msg) {
    keyboard_c_pressed_ = ((msg->data & (1u << 13)) != 0u);
    handle_track_button_state(get_track_button_pressed());
  }

  /**
   * @brief 开关消息回调，处理 trigger 状态并驱动追踪模式进入/退出
   *
   * 消息格式: [mode, pause, fn_left, fn_right, trigger]
   *   trigger (index 4): 0=松开, 1=按下
   *   remote 模式下：trigger 为追踪触发键
   *
   * @param msg /vt_remote/switches 消息
   */
  void switches_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 5) {
      return;
    }
    const int16_t new_mode_switch = msg->data[0];
    if (!mode_switch_valid_ || mode_switch_ != new_mode_switch) {
      mode_switch_ = new_mode_switch;
      mode_switch_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "mode 已切换为 %d，当前输入源=%s",
                  mode_switch_,
                  gimbal_control_source_to_string(get_active_gimbal_control_source()));
    }
    trigger_pressed_ = (msg->data[4] != 0);
    handle_track_button_state(get_track_button_pressed());
  }

  /**
   * @brief 在候选装甲板中寻找与卡尔曼预测位置最接近的目标
   *
   * 遍历所有有效装甲板，计算测量角度与预测角度的欧式距离，
   * 返回角度误差最小且小于 target_match_threshold_ 的装甲板指针。
   * 若无匹配则返回 nullptr。
   *
   * @param armors 本帧检测到的装甲板列表
   * @return 匹配到的装甲板指针，未匹配为 nullptr
   */
  const auto_aim_interfaces::msg::Armor* find_matching_armor(
      const std::vector<auto_aim_interfaces::msg::Armor>& armors)
  {
    const auto_aim_interfaces::msg::Armor* best = nullptr;
    double best_err = target_match_threshold_;
    for (const auto& a : armors) {
      if (a.pose.position.z < 0.01) continue;
      double yaw_meas   = std::atan2(a.pose.position.x, a.pose.position.z);
      double pitch_meas = std::atan2(a.pose.position.y, a.pose.position.z);
      double err = std::hypot(yaw_meas - kf_yaw_.angle, pitch_meas - kf_pitch_.angle);
      if (err < best_err) {
        best_err = err;
        best = &a;
      }
    }
    return best;
  }

  /**
   * @brief 装甲板检测结果回调
   *
   * 控制流程：
   *   1. KF 首次初始化 — 锁定距图像中心最近的装甲板
   *   2. 每帧做 KF 预测步骤（匀速外推）
   *   3. 若找到匹配目标 → KF 更新；目标丢失 → 仅用预测值，计时
   *   4. 丢失超过 target_lost_timeout_ → 重置 KF，等待下次锁定
   *   5. 控制误差 = KF 滤波角度 + 角速度 × 飞行时间（飞行预测）
   *      + 弹道下坠补偿（Pitch）
   *   6. 增量式 PI 将误差累加到目标位置
   *
   * @param msg /detector/armors 消息（相机坐标系 x=右/y=下/z=前）
   */
  void armors_callback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (!tracking_mode_) {
      return;
    }

    auto now = std::chrono::steady_clock::now();

    // ── 计算时间步长 ─────────────────────────────────────────────────
    double dt = 0.0;
    if (kf_time_valid_) {
      dt = std::chrono::duration<double>(now - kf_last_time_).count();
      if (dt > 0.5) {
        dt = 0.0;  // 间隔过大视为断续，放弃本次预测
      }
    }
    kf_last_time_ = now;
    kf_time_valid_ = true;

    // ── KF 未初始化：从最近目标播种 ─────────────────────────────────
    if (!kf_initialized_) {
      if (msg->armors.empty()) return;
      // 选距图像中心最近的装甲板作为初始锁定目标
      const auto* seed = &msg->armors[0];
      for (size_t i = 1; i < msg->armors.size(); ++i) {
        if (msg->armors[i].distance_to_image_center < seed->distance_to_image_center) {
          seed = &msg->armors[i];
        }
      }
      if (seed->pose.position.z < 0.01) return;
      kf_yaw_.reset(std::atan2(seed->pose.position.x, seed->pose.position.z));
      kf_pitch_.reset(std::atan2(seed->pose.position.y, seed->pose.position.z));
      kf_last_distance_ = std::sqrt(
          seed->pose.position.x * seed->pose.position.x +
          seed->pose.position.y * seed->pose.position.y +
          seed->pose.position.z * seed->pose.position.z);
      // 用初始角度播种 prev_error，避免首帧 P 项激增
      track_yaw_error_prev_   = std::atan2(seed->pose.position.x, seed->pose.position.z);
      track_pitch_error_prev_ = std::atan2(seed->pose.position.y, seed->pose.position.z);
      kf_initialized_ = true;
      kf_target_lost_ = false;
      RCLCPP_INFO(this->get_logger(), "[追踪] 锁定目标, 距离=%.2fm, yaw0=%.1f° pitch0=%.1f°",
                  kf_last_distance_,
                  track_yaw_error_prev_   * 180.0 / M_PI,
                  track_pitch_error_prev_ * 180.0 / M_PI);
      return;
    }

    // ── KF 预测步骤（每帧都做） ──────────────────────────────────────
    if (dt > 0.001) {
      kf_yaw_.predict(dt,   kf_q_angle_, kf_q_velocity_);
      kf_pitch_.predict(dt, kf_q_angle_, kf_q_velocity_);
    }

    // ── 寻找匹配目标并更新 KF ────────────────────────────────────────
    const auto* matched = find_matching_armor(msg->armors);
    if (matched) {
      double x = matched->pose.position.x;
      double y = matched->pose.position.y;
      double z = matched->pose.position.z;
      kf_yaw_.update(std::atan2(x, z),   kf_r_yaw_);
      kf_pitch_.update(std::atan2(y, z), kf_r_pitch_);
      kf_last_distance_ = std::sqrt(x * x + y * y + z * z);
      kf_target_lost_        = false;
      kf_target_lost_time_valid_ = false;
    } else {
      // 目标本帧未找到
      if (!kf_target_lost_) {
        kf_target_lost_      = true;
        kf_target_lost_time_ = now;
        kf_target_lost_time_valid_ = true;
      }
      double lost_elapsed = std::chrono::duration<double>(
          now - kf_target_lost_time_).count();
      if (lost_elapsed > target_lost_timeout_) {
        // 超出预测窗口，重置锁定
        kf_initialized_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[追踪] 目标丢失超过 %.2fs，等待重新锁定", target_lost_timeout_);
        return;
      }
      // PREDICT 期间：仅保持当前目标位置，不运行 PI，防止对外推误差积分跑飞
      // 更新 prev_error 以避免目标重新出现时产生 P 项激增
      double distance_p  = kf_last_distance_;
      double t_flight_p  = distance_p / bullet_velocity_;
      double drop_comp_p = std::atan2(0.5 * gravity_ * t_flight_p * t_flight_p, distance_p);
      track_yaw_error_prev_   = kf_yaw_.angle   + kf_yaw_.velocity   * t_flight_p;
      track_pitch_error_prev_ = kf_pitch_.angle - drop_comp_p;
      return;
    }

    // ── 弹道下坠补偿 ─────────────────────────────────────────────────
    double distance = kf_last_distance_;
    double t_flight  = distance / bullet_velocity_;
    double drop_comp = std::atan2(0.5 * gravity_ * t_flight * t_flight, distance);

    // ── 控制误差 = KF滤波角度 + 角速度×飞行时间（飞行预测） ──────────
    double yaw_error   = kf_yaw_.angle   + kf_yaw_.velocity   * t_flight;
    double pitch_error = kf_pitch_.angle - drop_comp;

    // ── 增量式 PI: Δu = Kp*[e(k)-e(k-1)] + Ki*e(k) ──────────────────
    double yaw_delta = track_yaw_kp_   * (yaw_error   - track_yaw_error_prev_)
                     + track_yaw_ki_   *  yaw_error;
    track_yaw_error_prev_ = yaw_error;

    double pitch_delta = track_pitch_kp_ * (pitch_error - track_pitch_error_prev_)
                       + track_pitch_ki_ *  pitch_error;
    track_pitch_error_prev_ = pitch_error;

    motor1_target_position_ += yaw_delta;
    motor1_target_position_ = clamp_motor1_position(motor1_target_position_);
    motor2_target_position_ += pitch_delta;
    motor2_target_position_ = std::max(motor2_min_position_,
                                       std::min(motor2_max_position_, motor2_target_position_));

    // ── 降频日志 ─────────────────────────────────────────────────────
    static int track_log_counter = 0;
    if (++track_log_counter >= 30) {
      RCLCPP_INFO(this->get_logger(),
                  "[追踪] d=%.2fm t_fl=%.3fs | "
                  "yaw_err=%.2f°(vel=%.1f°/s) pitch_err=%.2f° drop=%.2f° | %s",
                  distance, t_flight,
                  yaw_error   * 180.0 / M_PI,
                  kf_yaw_.velocity * 180.0 / M_PI,
                  pitch_error * 180.0 / M_PI,
                  drop_comp   * 180.0 / M_PI,
                  kf_target_lost_ ? "PREDICT" : "TRACK");
      track_log_counter = 0;
    }
  }

  /**
   * @brief 控制定时器回调（默认 200Hz）
   *
   * 负责维护目标位置积分，发布 /gimbal/cmd 绝对位置指令。
   */
  void control_timer_callback() {
    auto now = this->now();
    auto now_steady = std::chrono::steady_clock::now();

    if (!last_time_valid_) {
      last_time_ = now_steady;
      last_time_valid_ = true;
      return;
    }

    double dt = std::chrono::duration<double>(now_steady - last_time_).count();
    last_time_ = now_steady;
    if (dt > 0.05) {
      dt = 0.05;
    }

    if (!motor1_actual_valid_ || !motor2_actual_valid_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "等待电机实际位置反馈...");
      return;
    }

    if (!motor1_target_seeded_) {
      motor1_target_position_ = clamp_motor1_position(motor1_actual_position_);
      motor2_target_position_ = motor2_actual_position_;
      motor1_target_seeded_ = true;
      return;
    }

    const bool chassis_rotating =
        std::fabs(filtered_chassis_gyro_z_) >= chassis_rotation_activation_threshold_;
    const double chassis_feedforward_yaw_velocity =
        (chassis_gyro_compensation_enabled_ && chassis_rotating)
            ? (chassis_gyro_compensation_sign_ *
               chassis_gyro_compensation_gain_ * filtered_chassis_gyro_z_)
            : 0.0;
    const double gimbal_stabilize_yaw_velocity =
        (gimbal_gyro_stabilization_enabled_ && chassis_rotating)
            ? (-gimbal_gyro_stabilization_gain_ * filtered_gimbal_gyro_z_)
            : 0.0;

    if (!tracking_mode_) {
      const bool mouse_valid = mouse_time_valid_ &&
          ((now - last_mouse_time_).seconds() <= input_priority_timeout_);
      const GimbalControlSource active_source = get_active_gimbal_control_source();
      const bool use_mouse_yaw =
          (active_source == GimbalControlSource::MOUSE) ||
          ((active_source == GimbalControlSource::HYBRID) && mouse_valid && mouse_x_ != 0);
      const bool use_mouse_pitch =
          (active_source == GimbalControlSource::MOUSE) ||
          ((active_source == GimbalControlSource::HYBRID) && mouse_valid);

      // 计算 Yaw 速度并积分
      double yaw_velocity = 0.0;
      if (use_mouse_yaw) {
        yaw_velocity = static_cast<double>(mouse_x_) * mouse_yaw_sensitivity_;
        yaw_velocity = std::max(-motor1_max_velocity_, std::min(motor1_max_velocity_, yaw_velocity));
      } else if (rc_data_valid_) {
        yaw_velocity = map_rc_to_velocity(latest_rc_[motor1_channel_index_], motor1_max_velocity_);
      }
      yaw_velocity += chassis_feedforward_yaw_velocity;
      yaw_velocity += gimbal_stabilize_yaw_velocity;
      yaw_velocity = std::max(-motor1_max_velocity_, std::min(motor1_max_velocity_, yaw_velocity));
      motor1_target_position_ += yaw_velocity * dt;
      motor1_target_position_ = clamp_motor1_position(motor1_target_position_);

      // 计算 Pitch 目标位置
      if (use_mouse_pitch) {
        double pitch_vel = static_cast<double>(mouse_y_) * mouse_pitch_sensitivity_;
        pitch_vel = std::max(-motor2_control_speed_, std::min(motor2_control_speed_, pitch_vel));
        motor2_target_position_ += pitch_vel * dt;
        motor2_target_position_ = std::max(motor2_min_position_,
                                           std::min(motor2_max_position_, motor2_target_position_));
      } else if (rc_data_valid_) {
        motor2_target_position_ = map_rc_to_pos(
            latest_rc_[motor2_channel_index_], motor2_min_position_, motor2_max_position_);
      }
    } else {
      double tracking_rate_comp = 0.0;
      if (chassis_gyro_compensation_enabled_ && chassis_gyro_apply_in_tracking_) {
        tracking_rate_comp += chassis_feedforward_yaw_velocity;
      }
      if (gimbal_gyro_stabilization_enabled_ && gimbal_gyro_apply_in_tracking_) {
        tracking_rate_comp += gimbal_stabilize_yaw_velocity;
      }
      if (tracking_rate_comp != 0.0) {
        motor1_target_position_ += tracking_rate_comp * dt;
        motor1_target_position_ = clamp_motor1_position(motor1_target_position_);
      }
    }

    publish_cmd();
  }

  // ===== ROS 通信 =====
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rc_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr mouse_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr switches_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr keyboard_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor1_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr motor2_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr chassis_gyro_z_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_gyro_z_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ===== 遥控器参数 =====
  int rc_min_value_;
  int rc_max_value_;
  int rc_mid_value_;
  int deadzone_;
  int motor1_channel_index_;
  int motor2_channel_index_;
  double motor1_max_velocity_;
  bool motor1_position_limit_enabled_;
  double motor1_min_position_;
  double motor1_max_position_;
  bool chassis_gyro_compensation_enabled_{true};   ///< 是否启用底盘角速度补偿
  double chassis_gyro_compensation_gain_{1.0};     ///< 底盘角速度补偿增益
  double chassis_gyro_compensation_sign_{1.0};     ///< 补偿方向符号（+1/-1）
  double chassis_gyro_filter_alpha_{0.2};          ///< 底盘角速度低通滤波系数
  bool chassis_gyro_apply_in_tracking_{false};     ///< 追踪模式中是否应用补偿
  double chassis_rotation_activation_threshold_{0.08};  ///< 底盘旋转触发阈值(rad/s)
  bool gimbal_gyro_stabilization_enabled_{true};   ///< 是否启用云台角速度稳定闭环
  double gimbal_gyro_stabilization_gain_{1.0};     ///< 云台角速度稳定增益
  double gimbal_gyro_filter_alpha_{0.2};           ///< 云台角速度低通滤波系数
  bool gimbal_gyro_apply_in_tracking_{false};      ///< 追踪模式中是否应用稳定闭环

  // ===== 电机2 限位（本节点用于映射与限幅） =====
  double motor2_min_position_;
  double motor2_max_position_;
  double motor2_control_speed_;

  // ===== 鼠标控制参数 =====
  double mouse_yaw_sensitivity_;
  double mouse_pitch_sensitivity_;
  int mouse_max_speed_;
  GimbalControlSource gimbal_control_source_{GimbalControlSource::HYBRID};
  double input_priority_timeout_;

  // ===== 遥控器状态 =====
  std::vector<int16_t> latest_rc_;
  bool rc_data_valid_{false};

  // ===== 鼠标输入状态 =====
  int16_t mouse_x_{0};
  int16_t mouse_y_{0};
  rclcpp::Time last_mouse_time_;
  bool mouse_time_valid_{false};

  // ===== 控制与目标状态 =====
  double motor1_actual_position_{0.0};
  double motor2_actual_position_{0.0};
  bool motor1_actual_valid_{false};
  bool motor2_actual_valid_{false};

  double motor1_target_position_{0.0};
  double motor2_target_position_{0.0};
  bool motor1_target_seeded_{false};
  double filtered_chassis_gyro_z_{0.0};            ///< 滤波后的底盘角速度
  bool chassis_gyro_filter_seeded_{false};         ///< 低通滤波初始化状态
  double filtered_gimbal_gyro_z_{0.0};             ///< 滤波后的云台角速度
  bool gimbal_gyro_filter_seeded_{false};          ///< 云台角速度滤波初始化状态

  std::chrono::steady_clock::time_point last_time_;
  bool last_time_valid_{false};

  // ===== 追踪模式相关 =====
  bool tracking_mode_{false};
  int16_t mode_switch_{1};
  bool mode_switch_valid_{false};
  bool trigger_pressed_{false};
  bool keyboard_c_pressed_{false};
  std::chrono::steady_clock::time_point trigger_release_time_;
  bool trigger_release_time_valid_{false};
  double track_exit_timeout_;

  // ===== 增量式PI控制器参数与状态 =====
  double track_yaw_kp_;
  double track_yaw_ki_;
  double track_pitch_kp_;
  double track_pitch_ki_;
  double track_yaw_error_prev_{0.0};
  double track_pitch_error_prev_{0.0};

  // ===== 卡尔曼滤波器参数 =====
  double kf_q_angle_{0.01};
  double kf_q_velocity_{1.0};
  double kf_r_yaw_{0.005};
  double kf_r_pitch_{0.005};

  // ===== 目标锁定参数 =====
  double target_match_threshold_{0.3};
  double target_lost_timeout_{0.3};

  // ===== 卡尔曼滤波器状态 =====
  KalmanAngle kf_yaw_{};
  KalmanAngle kf_pitch_{};
  bool kf_initialized_{false};
  double kf_last_distance_{1.0};
  std::chrono::steady_clock::time_point kf_last_time_;
  bool kf_time_valid_{false};
  bool kf_target_lost_{false};
  std::chrono::steady_clock::time_point kf_target_lost_time_;
  bool kf_target_lost_time_valid_{false};

  // ===== Yaw/Pitch 瞄准偏移校准（硬件偏差补偿） =====
  double track_yaw_offset_{0.0};
  double track_pitch_offset_{0.0};

  // ===== 弹道补偿参数 =====
  double bullet_velocity_;
  double gravity_;
};

/**
 * @brief 主函数
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<AutoAimNode>();
  executor.add_node(node);

  rclcpp::WallRate loop_rate(200.0);
  while (rclcpp::ok()) {
    try {
      executor.spin_some();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node->get_logger(), "执行器捕获异常，继续运行: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "执行器捕获未知异常，继续运行");
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
