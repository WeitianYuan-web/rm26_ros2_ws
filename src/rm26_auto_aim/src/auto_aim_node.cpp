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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

    // Yaw 速度预测前馈参数
    this->declare_parameter<double>("track_yaw_ff_gain", 2.0);
    this->declare_parameter<double>("track_yaw_ff_filter", 0.2);
    this->declare_parameter<double>("track_yaw_ff_deadzone", 0.1);

    // 弹道下坠补偿参数
    this->declare_parameter<double>("bullet_velocity", 20.0);
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

    track_yaw_ff_gain_ = this->get_parameter("track_yaw_ff_gain").as_double();
    track_yaw_ff_filter_ = this->get_parameter("track_yaw_ff_filter").as_double();
    track_yaw_ff_deadzone_ = this->get_parameter("track_yaw_ff_deadzone").as_double();

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
    RCLCPP_INFO(this->get_logger(), "  前馈: 增益=%.2f, 滤波=%.2f, 死区=%.2f rad/s",
                track_yaw_ff_gain_, track_yaw_ff_filter_, track_yaw_ff_deadzone_);
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

    // 订阅装甲板检测结果（追踪模式 PI 控制输入）
    armors_subscription_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors",
        rclcpp::SensorDataQoS(),
        std::bind(&AutoAimNode::armors_callback, this, std::placeholders::_1));

    // 普通模式控制定时器（鼠标速度积分 + /gimbal/cmd 发布）
    int period_ms = 1000 / control_rate;
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&AutoAimNode::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "自动瞄准控制节点已启动，控制频率=%dHz", control_rate);
  }

private:
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

  /**
   * @brief 发布 /gimbal/cmd 普通模式指令
   * @param yaw_velocity Yaw 速度 rad/s
   * @param pitch_position Pitch 绝对目标位置 rad
   */
  void publish_normal_cmd(double yaw_velocity, double pitch_position) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {0.0, yaw_velocity, pitch_position};
    gimbal_cmd_publisher_->publish(msg);
  }

  /**
   * @brief 发布 /gimbal/cmd 追踪模式指令（增量）
   * @param yaw_delta Yaw 位置增量 rad
   * @param pitch_delta Pitch 位置增量 rad
   */
  void publish_tracking_cmd(double yaw_delta, double pitch_delta) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {1.0, yaw_delta, pitch_delta};
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
   * @brief 鼠标消息回调，更新鼠标输入状态
   *
   * 消息格式: [mouse_x, mouse_y, mouse_z, left, right, middle]
   * @param msg /vt_remote/mouse 消息
   */
  void mouse_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (gimbal_control_source_ == GimbalControlSource::REMOTE) {
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
   * @brief 开关消息回调，处理追踪模式的进入/退出
   *
   * 消息格式: [mode, pause, fn_left, fn_right, trigger]
   *   trigger (index 4): 0=松开, 1=按下
   *   按下立即进入追踪模式；松开超过 track_exit_timeout_ 后退出
   *
   * @param msg /vt_remote/switches 消息
   */
  void switches_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 5) {
      return;
    }

    bool trigger_pressed = (msg->data[4] != 0);

    if (trigger_pressed) {
      if (!tracking_mode_) {
        tracking_mode_ = true;
        track_yaw_error_prev_ = 0.0;
        track_pitch_error_prev_ = 0.0;
        yaw_angular_velocity_filtered_ = 0.0;
        yaw_ff_prev_ = 0.0;
        track_armors_time_valid_ = false;
        RCLCPP_INFO(this->get_logger(), ">>> 进入追踪模式");
        // 立即通知 gimbal_control_node 切换模式（触发 Pitch 播种）
        publish_tracking_cmd(0.0, 0.0);
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
            tracking_mode_ = false;
            motor2_mouse_target_valid_ = false;
            RCLCPP_INFO(this->get_logger(), "<<< 退出追踪模式 (松开 %.1fs)", track_exit_timeout_);
            // control_timer_ 将自动以普通模式发布下一条指令
          }
        }
      }
    }
  }

  /**
   * @brief 装甲板检测结果回调，追踪模式下执行增量式 PI + 速度预测前馈控制
   *
   * 控制律:
   *   Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)  （增量式PI）
   *   Pitch 融合弹道下坠补偿: pitch_error = atan2(y,z) - drop_compensation
   *   Yaw 前馈: yaw_ff = filtered_angular_velocity × t_flight × ff_gain
   *
   * @param msg /detector/armors 消息（装甲板位姿，相机坐标系 x=右/y=下/z=前）
   */
  void armors_callback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (!tracking_mode_) {
      return;
    }
    if (msg->armors.empty()) {
      return;
    }

    // 选择距图像中心最近的装甲板
    const auto *best = &msg->armors[0];
    for (size_t i = 1; i < msg->armors.size(); ++i) {
      if (msg->armors[i].distance_to_image_center < best->distance_to_image_center) {
        best = &msg->armors[i];
      }
    }

    double x = best->pose.position.x;
    double y = best->pose.position.y;
    double z = best->pose.position.z;

    if (z < 0.01) {
      return;
    }

    // 弹道下坠补偿: t=d/v0, Δh=½gt², θ_comp=atan(Δh/d)
    double distance = std::sqrt(x * x + y * y + z * z);
    double t_flight = distance / bullet_velocity_;
    double bullet_drop = 0.5 * gravity_ * t_flight * t_flight;
    double drop_compensation = std::atan2(bullet_drop, distance);

    double yaw_error = std::atan2(x, z);

    /**
     * @brief Pitch 误差融合弹道补偿
     *
     * pitch_error = atan2(y,z) - drop_compensation
     * PI 控制器驱动枪口抬高，使弹丸下落后命中目标。
     */
    double pitch_error = std::atan2(y, z) - drop_compensation;

    /**
     * @brief Yaw 速度预测前馈
     *
     * 通过目标 Yaw 角速度估计弹丸飞行期间的目标位移，以增量式方式施加前馈，
     * 角速度经过一阶低通滤波器平滑以抑制检测噪声。
     */
    auto armors_now = std::chrono::steady_clock::now();
    double yaw_angular_velocity = 0.0;
    if (track_armors_time_valid_) {
      double armors_dt = std::chrono::duration<double>(
          armors_now - track_armors_last_time_).count();
      if (armors_dt > 0.001 && armors_dt < 0.5) {
        yaw_angular_velocity = (yaw_error - track_yaw_error_prev_) / armors_dt;
      }
    }
    track_armors_last_time_ = armors_now;
    track_armors_time_valid_ = true;

    if (std::abs(yaw_angular_velocity) < track_yaw_ff_deadzone_) {
      yaw_angular_velocity = 0.0;
    }

    yaw_angular_velocity_filtered_ = track_yaw_ff_filter_ * yaw_angular_velocity
                                   + (1.0 - track_yaw_ff_filter_) * yaw_angular_velocity_filtered_;

    // 增量式PI: Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
    double yaw_delta = track_yaw_kp_ * (yaw_error - track_yaw_error_prev_)
                     + track_yaw_ki_ * yaw_error;
    track_yaw_error_prev_ = yaw_error;

    double pitch_delta = track_pitch_kp_ * (pitch_error - track_pitch_error_prev_)
                       + track_pitch_ki_ * pitch_error;
    track_pitch_error_prev_ = pitch_error;

    // 速度预测前馈增量
    double yaw_ff_new = yaw_angular_velocity_filtered_ * t_flight * track_yaw_ff_gain_;
    double yaw_ff_delta = yaw_ff_new - yaw_ff_prev_;
    yaw_ff_prev_ = yaw_ff_new;

    publish_tracking_cmd(yaw_delta + yaw_ff_delta, pitch_delta);

    // 降频日志
    static int track_log_counter = 0;
    if (++track_log_counter >= 30) {
      RCLCPP_INFO(this->get_logger(),
                  "[追踪] 距离=%.2fm, 误差: yaw=%.2f° pitch=%.2f°, "
                  "弹道补偿=%.2f°, Yaw角速度=%.1f°/s 前馈=%.2f°",
                  distance,
                  yaw_error * 180.0 / M_PI,
                  pitch_error * 180.0 / M_PI,
                  drop_compensation * 180.0 / M_PI,
                  yaw_angular_velocity_filtered_ * 180.0 / M_PI,
                  yaw_ff_new * 180.0 / M_PI);
      track_log_counter = 0;
    }
  }

  /**
   * @brief 普通模式控制定时器回调（默认 200Hz）
   *
   * 融合鼠标与遥控器输入，计算 Yaw 速度和 Pitch 目标位置，
   * 发布 /gimbal/cmd 普通模式指令。
   * 追踪模式下跳过（由 armors_callback 负责发布）。
   */
  void control_timer_callback() {
    if (tracking_mode_) {
      return;
    }

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

    const bool mouse_valid = mouse_time_valid_ &&
        ((now - last_mouse_time_).seconds() <= input_priority_timeout_);
    const bool use_mouse_yaw =
        (gimbal_control_source_ == GimbalControlSource::MOUSE) ||
        ((gimbal_control_source_ == GimbalControlSource::HYBRID) && mouse_valid && mouse_x_ != 0);
    const bool use_mouse_pitch =
        (gimbal_control_source_ == GimbalControlSource::MOUSE) ||
        ((gimbal_control_source_ == GimbalControlSource::HYBRID) && mouse_valid);

    // 计算 Yaw 速度指令
    double yaw_velocity = 0.0;
    if (use_mouse_yaw) {
      yaw_velocity = static_cast<double>(mouse_x_) * mouse_yaw_sensitivity_;
      yaw_velocity = std::max(-motor1_max_velocity_, std::min(motor1_max_velocity_, yaw_velocity));
    } else if (rc_data_valid_) {
      yaw_velocity = map_rc_to_velocity(latest_rc_[motor1_channel_index_], motor1_max_velocity_);
    }

    // 计算 Pitch 目标位置
    if (use_mouse_pitch) {
      if (!motor2_mouse_target_valid_) {
        // 首次进入鼠标模式：以遥控器当前映射位置为起始点
        if (rc_data_valid_) {
          motor2_mouse_target_ = map_rc_to_pos(
              latest_rc_[motor2_channel_index_], motor2_min_position_, motor2_max_position_);
        }
        motor2_mouse_target_valid_ = true;
      }
      double pitch_vel = static_cast<double>(mouse_y_) * mouse_pitch_sensitivity_;
      pitch_vel = std::max(-motor2_control_speed_, std::min(motor2_control_speed_, pitch_vel));
      motor2_mouse_target_ += pitch_vel * dt;
      motor2_mouse_target_ = std::max(motor2_min_position_,
                                      std::min(motor2_max_position_, motor2_mouse_target_));
      motor2_cmd_position_ = motor2_mouse_target_;
    } else if (rc_data_valid_) {
      motor2_cmd_position_ = map_rc_to_pos(
          latest_rc_[motor2_channel_index_], motor2_min_position_, motor2_max_position_);
      motor2_mouse_target_valid_ = false;
    }

    publish_normal_cmd(yaw_velocity, motor2_cmd_position_);
  }

  // ===== ROS 通信 =====
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rc_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr mouse_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr switches_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ===== 遥控器参数 =====
  int rc_min_value_;
  int rc_max_value_;
  int rc_mid_value_;
  int deadzone_;
  int motor1_channel_index_;
  int motor2_channel_index_;
  double motor1_max_velocity_;

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
  double motor2_mouse_target_{0.0};            ///< 鼠标模式 Pitch 积分目标位置 rad
  bool motor2_mouse_target_valid_{false};      ///< 鼠标目标是否已播种

  // ===== 普通模式控制状态 =====
  double motor2_cmd_position_{0.0};            ///< 当前 Pitch 目标位置 rad
  std::chrono::steady_clock::time_point last_time_;
  bool last_time_valid_{false};

  // ===== 追踪模式相关 =====
  bool tracking_mode_{false};
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

  // ===== Yaw 速度预测前馈参数与状态 =====
  double track_yaw_ff_gain_;
  double track_yaw_ff_filter_;
  double track_yaw_ff_deadzone_;
  double yaw_angular_velocity_filtered_{0.0};
  double yaw_ff_prev_{0.0};
  std::chrono::steady_clock::time_point track_armors_last_time_;
  bool track_armors_time_valid_{false};

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
