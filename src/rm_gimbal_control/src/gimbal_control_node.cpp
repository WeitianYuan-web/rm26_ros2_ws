/**
 * @file gimbal_control_node.cpp
 * @brief 云台控制节点，使用图传遥控器（/vt_remote/channels）控制 Pitch/Yaw 轴电机
 * @author Auto-generated
 * @date 2026
 */

#include "rm_gimbal_control/motor_cfg.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <cmath>
#include <auto_aim_interfaces/msg/armors.hpp>

/**
 * @brief 云台输入源模式
 */
enum class GimbalControlSource {
  MOUSE = 0,   ///< 仅鼠标
  REMOTE = 1,  ///< 仅遥控器
  HYBRID = 2   ///< 混合模式（鼠标优先窗口）
};

/**
 * @brief 云台控制节点类，负责云台 Pitch/Yaw 轴运动控制
 */
class GimbalControlNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   */
  GimbalControlNode()
      : rclcpp::Node("gimbal_control_node"),
        motor1_initialized_(false),
        motor2_initialized_(false) {

    // 声明参数 - 电机1 (Yaw轴速度积分模式，可连续旋转)
    this->declare_parameter<std::string>("motor1_can_interface", "can0");
    this->declare_parameter<int>("motor1_id", 0x02);
    this->declare_parameter<double>("motor1_max_velocity", 8.0);  // 电机1最大速度 rad/s
    this->declare_parameter<int>("motor1_channel_index", 0);      // 电机1使用的通道索引 (ch0)
    this->declare_parameter<int>("motor1_control_rate", 200);     // 电机1控制环频率 Hz

    // 声明参数 - 电机2 (Pitch轴建议配置)
    this->declare_parameter<std::string>("motor2_can_interface", "can1");
    this->declare_parameter<int>("motor2_id", 0x01);
    this->declare_parameter<double>("motor2_min_position", -0.45); // 电机2最小位置 rad
    this->declare_parameter<double>("motor2_max_position", 0.45);  // 电机2最大位置 rad
    this->declare_parameter<int>("motor2_channel_index", 1);      // 电机2使用的通道索引 (ch1)

    // 电机1控制参数
    this->declare_parameter<double>("motor1_control_speed", 10.0);        // 电机1控制速度 rad/s
    this->declare_parameter<double>("motor1_control_acceleration", 8.0); // 电机1加速度

    // 电机2控制参数
    this->declare_parameter<double>("motor2_control_speed", 4.0);        // 电机2控制速度 rad/s
    this->declare_parameter<double>("motor2_control_acceleration", 4.0); // 电机2加速度

    // 通用参数
    this->declare_parameter<int>("master_id", 0xFF);
    this->declare_parameter<int>("actuator_type", 5); // RS05
    this->declare_parameter<int>("rc_min_value", 364);
    this->declare_parameter<int>("rc_max_value", 1684);
    this->declare_parameter<int>("rc_mid_value", 1024);
    this->declare_parameter<int>("deadzone", 2); // 遥控器数值死区

    // 鼠标控制参数
    this->declare_parameter<double>("mouse_yaw_sensitivity", 0.02);    // 鼠标X -> Yaw速度 (rad/s)/count
    this->declare_parameter<double>("mouse_pitch_sensitivity", 0.02); // 鼠标Y -> Pitch速度 (rad/s)/count
    this->declare_parameter<int>("mouse_max_speed", 1500);              // 鼠标速度限幅 count
    this->declare_parameter<std::string>("gimbal_control_source", "hybrid");
    this->declare_parameter<double>("input_priority_timeout", 0.3);      // 键鼠优先窗口 s

    // 追踪模式参数
    this->declare_parameter<double>("track_yaw_kp", 1.5);       // 追踪Yaw方向比例增益
    this->declare_parameter<double>("track_yaw_ki", 0.4);       // 追踪Yaw方向积分增益
    this->declare_parameter<double>("track_pitch_kp", -0.6);     // 追踪Pitch方向比例增益
    this->declare_parameter<double>("track_pitch_ki", -0.2);     // 追踪Pitch方向积分增益
    this->declare_parameter<double>("track_exit_timeout", 0.5); // 松开trigger后退出追踪的超时时间 s

    // Yaw瞄准偏移校准
    this->declare_parameter<double>("track_yaw_offset", -0.2);      // Yaw瞄准偏移 rad（偏右设负值，偏左设正值）

    // Yaw速度预测前馈参数
    this->declare_parameter<double>("track_yaw_ff_gain",  2.0);    // 前馈增益（>1加大预测量，<1减小）
    this->declare_parameter<double>("track_yaw_ff_filter", 0.2);   
    this->declare_parameter<double>("track_yaw_ff_deadzone", 0.1); // 前馈角速度死区 rad/s（忽略小于此值的角速度）
    // 弹道下坠补偿参数
    this->declare_parameter<double>("bullet_velocity", 20.0);  // 弹丸初速度 m/s
    this->declare_parameter<double>("gravity", 9.81);           // 重力加速度 m/s²

    // 获取参数
    motor1_can_interface_ = this->get_parameter("motor1_can_interface").as_string();
    int motor1_id = this->get_parameter("motor1_id").as_int();
    motor1_max_velocity_ = this->get_parameter("motor1_max_velocity").as_double();
    motor1_channel_index_ = this->get_parameter("motor1_channel_index").as_int();
    motor1_control_rate_ = this->get_parameter("motor1_control_rate").as_int();

    motor2_can_interface_ = this->get_parameter("motor2_can_interface").as_string();
    int motor2_id = this->get_parameter("motor2_id").as_int();
    motor2_min_position_ = this->get_parameter("motor2_min_position").as_double();
    motor2_max_position_ = this->get_parameter("motor2_max_position").as_double();
    motor2_channel_index_ = this->get_parameter("motor2_channel_index").as_int();

    motor1_control_speed_ = this->get_parameter("motor1_control_speed").as_double();
    motor1_control_acceleration_ = this->get_parameter("motor1_control_acceleration").as_double();
    motor2_control_speed_ = this->get_parameter("motor2_control_speed").as_double();
    motor2_control_acceleration_ = this->get_parameter("motor2_control_acceleration").as_double();

    int master_id = this->get_parameter("master_id").as_int();
    int actuator_type = this->get_parameter("actuator_type").as_int();
    
    rc_min_value_ = this->get_parameter("rc_min_value").as_int();
    rc_max_value_ = this->get_parameter("rc_max_value").as_int();
    rc_mid_value_ = this->get_parameter("rc_mid_value").as_int();
    deadzone_ = this->get_parameter("deadzone").as_int();
    mouse_yaw_sensitivity_ = this->get_parameter("mouse_yaw_sensitivity").as_double();
    mouse_pitch_sensitivity_ = this->get_parameter("mouse_pitch_sensitivity").as_double();
    mouse_max_speed_ = this->get_parameter("mouse_max_speed").as_int();
    const std::string gimbal_control_source =
        this->get_parameter("gimbal_control_source").as_string();
    input_priority_timeout_ = this->get_parameter("input_priority_timeout").as_double();
    gimbal_control_source_ = parse_gimbal_control_source(gimbal_control_source);

    track_yaw_kp_ = this->get_parameter("track_yaw_kp").as_double();
    track_yaw_ki_ = this->get_parameter("track_yaw_ki").as_double();
    track_pitch_kp_ = this->get_parameter("track_pitch_kp").as_double();
    track_pitch_ki_ = this->get_parameter("track_pitch_ki").as_double();
    track_exit_timeout_ = this->get_parameter("track_exit_timeout").as_double();

    track_yaw_offset_ = this->get_parameter("track_yaw_offset").as_double();

    track_yaw_ff_gain_ = this->get_parameter("track_yaw_ff_gain").as_double();
    track_yaw_ff_filter_ = this->get_parameter("track_yaw_ff_filter").as_double();
    track_yaw_ff_deadzone_ = this->get_parameter("track_yaw_ff_deadzone").as_double();

    bullet_velocity_ = this->get_parameter("bullet_velocity").as_double();
    gravity_ = this->get_parameter("gravity").as_double();

    // 使用参数创建电机对象
    motor1_ = std::make_unique<RobStrideMotor>(
        motor1_can_interface_, 
        static_cast<uint8_t>(master_id), 
        static_cast<uint8_t>(motor1_id), 
        actuator_type);

    motor2_ = std::make_unique<RobStrideMotor>(
        motor2_can_interface_, 
        static_cast<uint8_t>(master_id), 
        static_cast<uint8_t>(motor2_id), 
        actuator_type);

    RCLCPP_INFO(this->get_logger(), "云台控制节点配置:");
    RCLCPP_INFO(this->get_logger(), "  电机1(速度积分模式): 接口=%s, ID=0x%02X, 最大速度=%.2f rad/s, 通道=ch%d, 控制频率=%dHz", 
                motor1_can_interface_.c_str(), motor1_id, motor1_max_velocity_, motor1_channel_index_, motor1_control_rate_);
    RCLCPP_INFO(this->get_logger(), "  电机1 CSP参数: 控制速度=%.2f rad/s, 加速度=%.2f", motor1_control_speed_, motor1_control_acceleration_);
    RCLCPP_INFO(this->get_logger(), "  电机2(位置映射模式): 接口=%s, ID=0x%02X, 范围=[%.2f, %.2f], 通道=ch%d", 
                motor2_can_interface_.c_str(), motor2_id, motor2_min_position_, motor2_max_position_, motor2_channel_index_);
    RCLCPP_INFO(this->get_logger(), "  电机2 CSP参数: 控制速度=%.2f rad/s, 加速度=%.2f", motor2_control_speed_, motor2_control_acceleration_);
    RCLCPP_INFO(this->get_logger(), "  RC范围: %d~%d (中值%d)", rc_min_value_, rc_max_value_, rc_mid_value_);
    RCLCPP_INFO(this->get_logger(), "  鼠标控制: yaw灵敏度=%.4f, pitch灵敏度=%.4f, 限幅=%d, 优先窗口=%.2fs",
                mouse_yaw_sensitivity_, mouse_pitch_sensitivity_, mouse_max_speed_, input_priority_timeout_);
    RCLCPP_INFO(this->get_logger(), "  云台输入源: %s",
                gimbal_control_source_to_string(gimbal_control_source_));
    RCLCPP_INFO(this->get_logger(), "  追踪参数: Yaw(Kp=%.2f, Ki=%.2f), Pitch(Kp=%.2f, Ki=%.2f), 退出超时=%.1fs",
                track_yaw_kp_, track_yaw_ki_, track_pitch_kp_, track_pitch_ki_, track_exit_timeout_);
    RCLCPP_INFO(this->get_logger(), "  Yaw偏移=%.4f rad (%.2f°), 前馈: 增益=%.2f, 滤波=%.2f, 死区=%.2f rad/s",
                track_yaw_offset_, track_yaw_offset_ * 180.0 / M_PI, track_yaw_ff_gain_, track_yaw_ff_filter_, track_yaw_ff_deadzone_);
    RCLCPP_INFO(this->get_logger(), "  弹道补偿: 弹速=%.1f m/s, 重力=%.2f m/s²", bullet_velocity_, gravity_);

    // 创建发布者（用于监控当前目标位置）
    motor1_target_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor1/target_position", 10);
    motor2_target_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor2/target_position", 10);
    motor1_actual_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor1/actual_position", 10);
    motor2_actual_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor2/actual_position", 10);
    motor1_multi_turn_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor1/multi_turn_position", 10);
    motor2_multi_turn_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor2/multi_turn_position", 10);
    motor1_turn_count_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "motor1/turn_count", 10);
    motor2_turn_count_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
        "motor2/turn_count", 10);

    // 创建 QoS 配置
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // 订阅 /vt_remote/channels 消息
    rc_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/channels",
        qos,
        std::bind(&GimbalControlNode::rc_callback, this, std::placeholders::_1));

    // 订阅 /vt_remote/mouse 消息（手动云台控制）
    mouse_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/mouse",
        qos,
        std::bind(&GimbalControlNode::mouse_callback, this, std::placeholders::_1));

    // 订阅 /vt_remote/switches 消息（用于追踪模式切换）
    switches_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "/vt_remote/switches",
        qos,
        std::bind(&GimbalControlNode::switches_callback, this, std::placeholders::_1));

    // 订阅 /detector/armors 消息（用于追踪模式）
    armors_subscription_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
        "/detector/armors",
        rclcpp::SensorDataQoS(),
        std::bind(&GimbalControlNode::armors_callback, this, std::placeholders::_1));

    // 创建电机1的高频控制定时器（速度积分 + 下发指令）
    int period_ms = 1000 / motor1_control_rate_;
    motor1_control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&GimbalControlNode::motor1_control_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "  电机1控制定时器: 周期=%dms (%dHz)", period_ms, motor1_control_rate_);

    // 初始化电机（使用异步方式，避免阻塞）
    init_motor(motor1_, motor1_initialized_, "电机1");
    init_motor(motor2_, motor2_initialized_, "电机2");

    RCLCPP_INFO(this->get_logger(), "等待遥控器信号...");
  }

  /**
   * @brief 析构函数
   */
  ~GimbalControlNode() {
    RCLCPP_INFO(this->get_logger(), "正在停止电机...");
    if (motor1_initialized_ && motor1_) {
      try {
        motor1_->Disenable_Motor(0);
        RCLCPP_INFO(this->get_logger(), "电机1已停止");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "停止电机1时出错: %s", e.what());
      }
    }
    if (motor2_initialized_ && motor2_) {
      try {
        motor2_->Disenable_Motor(0);
        RCLCPP_INFO(this->get_logger(), "电机2已停止");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "停止电机2时出错: %s", e.what());
      }
    }
  }

private:
  /**
   * @brief 解析云台输入源参数
   * @param source 参数字符串
   * @return 输入源枚举
   */
  GimbalControlSource parse_gimbal_control_source(const std::string &source) const {
    if (source == "mouse") {
      return GimbalControlSource::MOUSE;
    }
    if (source == "remote") {
      return GimbalControlSource::REMOTE;
    }
    if (source == "hybrid") {
      return GimbalControlSource::HYBRID;
    }
    RCLCPP_WARN(this->get_logger(),
                "未知 gimbal_control_source='%s'，回退为 hybrid",
                source.c_str());
    return GimbalControlSource::HYBRID;
  }

  /**
   * @brief 云台输入源转字符串
   * @param source 输入源枚举
   * @return 输入源字符串
   */
  const char *gimbal_control_source_to_string(GimbalControlSource source) const {
    switch (source) {
      case GimbalControlSource::MOUSE:
        return "mouse";
      case GimbalControlSource::REMOTE:
        return "remote";
      case GimbalControlSource::HYBRID:
      default:
        return "hybrid";
    }
  }

  void update_multi_turn_position(double raw_position,
                                  bool &multi_turn_valid,
                                  double &last_raw_position,
                                  int &turn_count,
                                  double &multi_turn_position) {
    if (!multi_turn_valid) {
      last_raw_position = raw_position;
      multi_turn_position = raw_position;
      turn_count = 0;
      multi_turn_valid = true;
      return;
    }

    constexpr double kWrapRange = 2.0 * M_PI;
    constexpr double kWrapPeriod = 4.0 * M_PI;
    const double delta = raw_position - last_raw_position;
    if (delta > kWrapRange) {
      --turn_count;
    } else if (delta < -kWrapRange) {
      ++turn_count;
    }
    multi_turn_position = raw_position + static_cast<double>(turn_count) * kWrapPeriod;
    last_raw_position = raw_position;
  }

  /**
   * @brief 初始化电机（异步方式，带超时）
   * @param motor 电机对象指针
   * @param initialized 初始化标志
   * @param motor_name 电机名称（用于日志）
   */
  void init_motor(std::unique_ptr<RobStrideMotor>& motor, std::atomic<bool>& initialized, 
                  const std::string& motor_name) {
    if (!motor) {
      RCLCPP_ERROR(this->get_logger(), "%s对象未创建", motor_name.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "正在初始化%s...", motor_name.c_str());
    
    // 在单独线程中初始化，避免阻塞主线程
    std::thread init_thread([this, &motor, &initialized, motor_name]() {
      const int max_retries = 5;

      /**
       * @brief 设置CAN接收超时（2秒），防止 receive_status_frame() 永久阻塞
       *
       * 原始 receive() 在 timeout_sec=0 时不设置超时，若电机未上电或CAN断开，
       * recv() 将永久阻塞导致初始化线程卡死。设置超时后，recv() 超时返回，
       * receive_status_frame() 抛出异常，由重试循环处理。
       */
      struct timeval recv_timeout;
      recv_timeout.tv_sec = 2;
      recv_timeout.tv_usec = 0;
      if (setsockopt(motor->socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                     &recv_timeout, sizeof(recv_timeout)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "%s: 设置CAN接收超时失败", motor_name.c_str());
      }

      for (int attempt = 1; attempt <= max_retries; ++attempt) {
        try {
          RCLCPP_INFO(this->get_logger(), "%s: 初始化尝试 %d/%d ...",
                      motor_name.c_str(), attempt, max_retries);

          // 第一步：失能电机并清除错误（clear_error=1）
          RCLCPP_INFO(this->get_logger(), "%s: 失能电机并清除错误...", motor_name.c_str());
          motor->Disenable_Motor(1);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          // 第二步：读取当前电机模式
          motor->Get_RobStrite_Motor_parameter(0x7005);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          
          // 第三步：如果不是CSP模式（mode=5），切换到CSP模式
          if (motor->drw.run_mode.data != 5) {
            RCLCPP_INFO(this->get_logger(), "%s当前模式: %.0f, 切换到CSP模式...", 
                        motor_name.c_str(), motor->drw.run_mode.data);
            motor->Set_RobStrite_Motor_parameter(0x7005, 5, 'j'); // 设置为CSP位置模式
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            motor->Get_RobStrite_Motor_parameter(0x7005);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          
          // 第四步：使能电机
          motor->enable_motor();
          initialized = true;
          RCLCPP_INFO(this->get_logger(), "%s已使能（CSP位置控制模式，错误已清除）", motor_name.c_str());
          RCLCPP_INFO(this->get_logger(), "%s启动位置: %.4f rad (%.2f deg)",
                      motor_name.c_str(), motor->position_, motor->position_ * 180.0 / M_PI);
          if (motor1_initialized_ && motor2_initialized_ && motor1_ && motor2_) {
            RCLCPP_INFO(this->get_logger(),
                        "启动时电机位置 -> 电机1: %.4f rad (%.2f deg), 电机2: %.4f rad (%.2f deg)",
                        motor1_->position_, motor1_->position_ * 180.0 / M_PI,
                        motor2_->position_, motor2_->position_ * 180.0 / M_PI);
          }

          // 初始化成功，清除接收超时（恢复阻塞模式用于正常高频通信）
          struct timeval no_timeout{0, 0};
          setsockopt(motor->socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                     &no_timeout, sizeof(no_timeout));

          return; // 成功，退出重试循环
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), 
                      "%s: 初始化尝试 %d/%d 失败: %s",
                      motor_name.c_str(), attempt, max_retries, e.what());
          if (attempt < max_retries) {
            RCLCPP_INFO(this->get_logger(), "%s: 等待1秒后重试...", motor_name.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
        }
      }

      // 所有重试都失败
      RCLCPP_ERROR(this->get_logger(), 
                  "%s初始化失败（已重试%d次）", motor_name.c_str(), max_retries);
      RCLCPP_ERROR(this->get_logger(), 
                  "请检查: 1) CAN接口是否正确配置 2) 电机是否连接并上电 3) 电机ID是否正确");
      initialized = false;
    });
    init_thread.detach(); // 分离线程，不等待完成
  }

  /**
   * @brief RC消息回调函数
   * @param msg /vt_remote/channels 消息
   */
  void rc_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    // 检查是否有足够的数据
    int max_index = std::max(motor1_channel_index_, motor2_channel_index_);
    if (msg->data.size() <= static_cast<size_t>(max_index)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "RC消息数据长度不足: %zu, 需要至少 %d",
                           msg->data.size(), max_index + 1);
      return;
    }

    // 追踪模式下不处理遥控器控制
    if (tracking_mode_) {
      return;
    }

    try {
      const auto now = this->now();
      const bool mouse_valid =
          mouse_time_valid_ &&
          ((now - last_mouse_time_).seconds() <= input_priority_timeout_);
      const bool use_mouse_yaw =
          (gimbal_control_source_ == GimbalControlSource::MOUSE) ||
          ((gimbal_control_source_ == GimbalControlSource::HYBRID) && mouse_valid && mouse_x_ != 0);
      const bool use_mouse_pitch =
          (gimbal_control_source_ == GimbalControlSource::MOUSE) ||
          ((gimbal_control_source_ == GimbalControlSource::HYBRID) && mouse_valid);

      // 电机1：只更新目标速度，实际控制由高频定时器执行
      if (motor1_initialized_) {
        if (use_mouse_yaw) {
          double mouse_vel = static_cast<double>(mouse_x_) * mouse_yaw_sensitivity_;
          mouse_vel = std::max(-motor1_max_velocity_, std::min(motor1_max_velocity_, mouse_vel));
          motor1_current_velocity_ = mouse_vel;
        } else {
          motor1_current_velocity_ = map_rc_to_velocity(
              msg->data[motor1_channel_index_], motor1_max_velocity_);
        }
      }

      // 控制电机2
      if (motor2_initialized_ && motor2_) {
        if (use_mouse_pitch) {
          if (!motor2_mouse_target_valid_) {
            motor2_mouse_target_position_ = motor2_->position_;
            motor2_mouse_target_valid_ = true;
          }
        } else {
          control_motor(motor2_,
                        msg->data[motor2_channel_index_],
                        motor2_min_position_,
                        motor2_max_position_,
                        motor2_control_speed_,
                        motor2_target_position_publisher_,
                        "电机2",
                        motor2_channel_index_);
          motor2_mouse_target_valid_ = false;
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "控制电机时出错: %s", e.what());
    }
  }

  /**
   * @brief 鼠标消息回调，更新云台手动控制输入
   * @param msg /vt_remote/mouse 消息 [mouse_x, mouse_y, mouse_z, left, right, middle]
   */
  void mouse_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (gimbal_control_source_ == GimbalControlSource::REMOTE) {
      return;
    }

    if (msg->data.size() < 2) {
      return;
    }
    int clamped_x = std::max(-mouse_max_speed_, std::min(mouse_max_speed_, static_cast<int>(msg->data[0])));
    int clamped_y = std::max(-mouse_max_speed_, std::min(mouse_max_speed_, static_cast<int>(msg->data[1])));
    mouse_x_ = static_cast<int16_t>(clamped_x);
    mouse_y_ = static_cast<int16_t>(clamped_y);
    if ((clamped_x != 0) || (clamped_y != 0)) {
      last_mouse_time_ = this->now();
      mouse_time_valid_ = true;
    }
  }

  /**
   * @brief Switches消息回调，处理追踪模式切换
   *
   * switches消息格式: [mode, pause, fn_left, fn_right, trigger]
   * - trigger (index 4): 0=松开, 1=按下
   * - 按下trigger立即进入追踪模式
   * - 松开trigger超过track_exit_timeout_后退出追踪模式
   * @param msg /vt_remote/switches 消息
   */
  void switches_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    if (msg->data.size() < 5) {
      return;
    }

    bool trigger_pressed = (msg->data[4] != 0);

    if (trigger_pressed) {
      // trigger按下 → 进入追踪模式
      if (!tracking_mode_) {
        tracking_mode_ = true;
        // 重置增量PI控制器状态
        track_yaw_error_prev_ = 0.0;
        track_pitch_error_prev_ = 0.0;
        // 重置速度预测前馈状态
        yaw_angular_velocity_filtered_ = 0.0;
        yaw_ff_prev_ = 0.0;
        track_armors_time_valid_ = false;
        // 初始化追踪模式下电机2的目标位置为当前实际位置
        if (motor2_initialized_ && motor2_) {
          motor2_track_target_position_ = motor2_->position_;
        }
        RCLCPP_INFO(this->get_logger(), ">>> 进入追踪模式");
      }
      // 重置松开计时
      trigger_release_time_valid_ = false;
    } else {
      // trigger松开
      if (tracking_mode_) {
        if (!trigger_release_time_valid_) {
          // 首次检测到松开，记录时间
          trigger_release_time_ = std::chrono::steady_clock::now();
          trigger_release_time_valid_ = true;
        } else {
          // 检查松开时间是否超过阈值
          double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - trigger_release_time_).count();
          if (elapsed > track_exit_timeout_) {
            tracking_mode_ = false;
            // 退出追踪时，将电机1目标位置更新为当前实际位置，确保摇杆控制平滑恢复
            if (motor1_initialized_ && motor1_) {
              motor1_target_position_ = motor1_->position_;
            }
            RCLCPP_INFO(this->get_logger(), "<<< 退出追踪模式 (松开超过%.1fs)", track_exit_timeout_);
          }
        }
      }
    }
  }

  /**
   * @brief Armors消息回调，在追踪模式下执行增量PI控制
   *
   * 使用增量式PI控制器: Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
   * - Yaw方向 (电机1): 根据目标水平角度偏差调整
   * - Pitch方向 (电机2): 根据目标垂直角度偏差调整
   * @param msg /detector/armors 消息（包含检测到的装甲板位姿信息）
   */
  void armors_callback(const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
    if (!tracking_mode_) {
      return;
    }
    if (msg->armors.empty()) {
      return;
    }

    // 选择距离图像中心最近的装甲板
    const auto* best_armor = &msg->armors[0];
    for (size_t i = 1; i < msg->armors.size(); ++i) {
      if (msg->armors[i].distance_to_image_center < best_armor->distance_to_image_center) {
        best_armor = &msg->armors[i];
      }
    }

    // 获取装甲板在相机坐标系下的位置 (x=右, y=下, z=前)
    double x = best_armor->pose.position.x;
    double y = best_armor->pose.position.y;
    double z = best_armor->pose.position.z;

    if (z < 0.01) {
      return; // 距离过近或无效数据
    }

    // 弹道下坠补偿：根据目标距离计算弹丸抛物线下坠量
    // 物理模型: t = d/v0, Δh = ½gt², θ_comp = atan(Δh/d)
    double distance = std::sqrt(x * x + y * y + z * z);
    double t_flight = distance / bullet_velocity_;
    double bullet_drop = 0.5 * gravity_ * t_flight * t_flight;
    double drop_compensation = std::atan2(bullet_drop, distance); // 补偿角 (rad)，始终为正

    // 计算角度误差 (rad)
    double yaw_error = std::atan2(x, z);    // 正值 = 目标在右侧

    /**
     * @brief Pitch误差融合弹道补偿
     *
     * 将弹道下坠补偿融入pitch误差信号，而不是作为累加偏移。
     * 补偿后 pitch_error = atan2(y,z) - drop_compensation：
     *   PI 控制器驱动枪口抬高，使弹丸下落后刚好命中目标。
     */
    double pitch_error = std::atan2(y, z) - drop_compensation;

    /**
     * @brief Yaw方向速度预测前馈
     *
     * 通过计算目标的Yaw角速度，结合弹丸飞行时间，预测目标在弹丸到达时的位置偏移。
     * 使用增量式前馈：每次只应用前馈变化量，避免累积误差。
     * 角速度使用一阶低通滤波器平滑，抑制检测噪声。
     *
     * 前馈量 = 滤波角速度 × 弹丸飞行时间 × 前馈增益
     */
    auto armors_now = std::chrono::steady_clock::now();
    double yaw_angular_velocity = 0.0;
    if (track_armors_time_valid_) {
      double armors_dt = std::chrono::duration<double>(
          armors_now - track_armors_last_time_).count();
      if (armors_dt > 0.001 && armors_dt < 0.5) { // 合理的时间间隔
        yaw_angular_velocity = (yaw_error - track_yaw_error_prev_) / armors_dt;
      }
    }
    track_armors_last_time_ = armors_now;
    track_armors_time_valid_ = true;

    // 死区：忽略微小角速度（抑制静止时检测噪声导致的前馈漂移）
    if (std::abs(yaw_angular_velocity) < track_yaw_ff_deadzone_) {
      yaw_angular_velocity = 0.0;
    }

    // 低通滤波平滑角速度
    yaw_angular_velocity_filtered_ = track_yaw_ff_filter_ * yaw_angular_velocity
                                   + (1.0 - track_yaw_ff_filter_) * yaw_angular_velocity_filtered_;

    // 增量式PI: Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
    double yaw_delta = track_yaw_kp_ * (yaw_error - track_yaw_error_prev_)
                     + track_yaw_ki_ * yaw_error;
    track_yaw_error_prev_ = yaw_error;

    double pitch_delta = track_pitch_kp_ * (pitch_error - track_pitch_error_prev_)
                       + track_pitch_ki_ * pitch_error;
    track_pitch_error_prev_ = pitch_error;

    // 速度预测前馈：增量式更新，只应用前馈变化量
    double yaw_ff_new = yaw_angular_velocity_filtered_ * t_flight * track_yaw_ff_gain_;
    double yaw_ff_delta = yaw_ff_new - yaw_ff_prev_;
    yaw_ff_prev_ = yaw_ff_new;

    // 更新电机1 (Yaw) 目标位置 = PI增量 + 前馈增量
    motor1_target_position_ += yaw_delta + yaw_ff_delta;

    // 更新电机2 (Pitch) 目标位置
    motor2_track_target_position_ += pitch_delta;

    // 限幅
    motor2_track_target_position_ = std::max(motor2_min_position_,
                                     std::min(motor2_max_position_, motor2_track_target_position_));

    // 定期打印追踪信息
    static int track_log_counter = 0;
    if (++track_log_counter >= 30) { // 约每秒打印一次（假设30Hz检测频率）
      RCLCPP_INFO(this->get_logger(),
                  "[追踪] 距离=%.2fm, 误差: yaw=%.2f° pitch=%.2f°, "
                  "弹道补偿=%.2f°, "
                  "Yaw角速度=%.1f°/s 前馈=%.2f°",
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
   * @brief 将RC值映射到速度值
   * @param rc_value 遥控器值 (364 ~ 1684)
   * @param max_velocity 最大速度 rad/s
   * @return 映射后的速度值 [-max_velocity, +max_velocity]，中值对应0
   */
  double map_rc_to_velocity(int rc_value, double max_velocity) {
    // 限制输入范围
    int clamped_value = std::max(rc_min_value_, std::min(rc_max_value_, rc_value));

    // 死区处理：中值附近视为0速度
    if (std::abs(clamped_value - rc_mid_value_) < deadzone_) {
      return 0.0;
    }

    // 归一化到 [-1.0, +1.0]，以中值为零点
    double normalized = static_cast<double>(clamped_value - rc_mid_value_) /
                        static_cast<double>(rc_max_value_ - rc_mid_value_);

    // 限幅到 [-1.0, +1.0]
    normalized = std::max(-1.0, std::min(1.0, normalized));

    return normalized * max_velocity;
  }

  /**
   * @brief 电机1高频控制定时器回调
   *
   * 以固定高频率（默认200Hz）执行，根据当前目标速度做积分得到目标位置，
   * 并下发 CSP 位置指令。不做位置限幅，电机可连续旋转。
   * RC 回调只负责更新 motor1_current_velocity_。
   */
  void motor1_control_timer_callback() {
    if (!motor1_initialized_ || !motor1_) {
      return;
    }

    try {
      // 首次进入，用电机当前位置作为初始目标
      if (!motor1_last_time_valid_) {
        motor1_last_time_ = std::chrono::steady_clock::now();
        motor1_target_position_ = motor1_->position_;
        motor1_last_time_valid_ = true;
        return;
      }

      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - motor1_last_time_).count();
      motor1_last_time_ = now;

      // 限制 dt 防止异常跳变
      if (dt > 0.05) {
        dt = 0.05;
      }

      if (!tracking_mode_) {
        // 普通模式：使用当前速度做积分
        double velocity = motor1_current_velocity_;
        motor1_target_position_ += velocity * dt;

        // 鼠标控制电机2：将鼠标Y轴速度积分为目标位置（与电机1一致）
        const auto now_ros = this->now();
        const bool mouse_valid =
            mouse_time_valid_ &&
            ((now_ros - last_mouse_time_).seconds() <= input_priority_timeout_);
        const bool use_mouse_pitch =
            (gimbal_control_source_ == GimbalControlSource::MOUSE) ||
            ((gimbal_control_source_ == GimbalControlSource::HYBRID) && mouse_valid);
        if (motor2_initialized_ && motor2_ && use_mouse_pitch) {
          if (!motor2_mouse_target_valid_) {
            motor2_mouse_target_position_ = motor2_->position_;
            motor2_mouse_target_valid_ = true;
          }

          double mouse_pitch_velocity =
              static_cast<double>(mouse_y_) * mouse_pitch_sensitivity_;
          mouse_pitch_velocity = std::max(-motor2_control_speed_,
                                          std::min(motor2_control_speed_, mouse_pitch_velocity));

          motor2_mouse_target_position_ += mouse_pitch_velocity * dt;
          motor2_mouse_target_position_ = std::max(
              motor2_min_position_,
              std::min(motor2_max_position_, motor2_mouse_target_position_));

          motor2_->RobStrite_Motor_PosCSP_control(
              motor2_control_speed_,
              motor2_mouse_target_position_);

          auto target_msg2 = std_msgs::msg::Float32();
          target_msg2.data = static_cast<float>(motor2_mouse_target_position_);
          motor2_target_position_publisher_->publish(target_msg2);
        }
      }
      // 追踪模式下，motor1_target_position_ 由 armors_callback 更新

      // 下发电机1 CSP 位置指令（追踪模式下叠加瞄准偏移校准）
      double motor1_cmd_position = motor1_target_position_;
      if (tracking_mode_) {
        motor1_cmd_position += track_yaw_offset_;
      }
      motor1_->RobStrite_Motor_PosCSP_control(
          motor1_control_speed_,
          motor1_cmd_position);

      // 追踪模式下同时以高频率下发电机2指令
      if (tracking_mode_ && motor2_initialized_ && motor2_) {
        motor2_->RobStrite_Motor_PosCSP_control(
            motor2_control_speed_,
            motor2_track_target_position_);
        auto target_msg2 = std_msgs::msg::Float32();
        target_msg2.data = motor2_track_target_position_;
        motor2_target_position_publisher_->publish(target_msg2);
      }

      if (motor1_initialized_ && motor1_) {
        update_multi_turn_position(motor1_->position_,
                                   motor1_multi_turn_valid_,
                                   motor1_last_raw_position_,
                                   motor1_turn_count_,
                                   motor1_multi_turn_position_);
        auto actual_msg1 = std_msgs::msg::Float32();
        actual_msg1.data = static_cast<float>(motor1_->position_);
        motor1_actual_position_publisher_->publish(actual_msg1);
        auto multi_turn_msg1 = std_msgs::msg::Float32();
        multi_turn_msg1.data = static_cast<float>(motor1_multi_turn_position_);
        motor1_multi_turn_position_publisher_->publish(multi_turn_msg1);
        auto turn_count_msg1 = std_msgs::msg::Int32();
        turn_count_msg1.data = motor1_turn_count_;
        motor1_turn_count_publisher_->publish(turn_count_msg1);
      }
      if (motor2_initialized_ && motor2_) {
        update_multi_turn_position(motor2_->position_,
                                   motor2_multi_turn_valid_,
                                   motor2_last_raw_position_,
                                   motor2_turn_count_,
                                   motor2_multi_turn_position_);
        auto actual_msg2 = std_msgs::msg::Float32();
        actual_msg2.data = static_cast<float>(motor2_->position_);
        motor2_actual_position_publisher_->publish(actual_msg2);
        auto multi_turn_msg2 = std_msgs::msg::Float32();
        multi_turn_msg2.data = static_cast<float>(motor2_multi_turn_position_);
        motor2_multi_turn_position_publisher_->publish(multi_turn_msg2);
        auto turn_count_msg2 = std_msgs::msg::Int32();
        turn_count_msg2.data = motor2_turn_count_;
        motor2_turn_count_publisher_->publish(turn_count_msg2);
      }

      // 发布电机1目标位置
      auto target_msg = std_msgs::msg::Float32();
      target_msg.data = motor1_target_position_;
      motor1_target_position_publisher_->publish(target_msg);

      // 定期打印信息（降低频率）
      static int counter1 = 0;
      if (++counter1 >= motor1_control_rate_) { // 约每秒打印一次
        if (tracking_mode_) {
          RCLCPP_INFO(this->get_logger(),
                      "电机1(追踪模式@%dHz): "
                      "目标位置=%.4f rad (%.2f deg), "
                      "实际位置=%.4f rad (%.2f deg)",
                      motor1_control_rate_,
                      motor1_target_position_, motor1_target_position_ * 180.0 / M_PI,
                      motor1_->position_, motor1_->position_ * 180.0 / M_PI);
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "电机1(速度积分@%dHz): 速度=%.4f rad/s, "
                      "目标位置=%.4f rad (%.2f deg), "
                      "实际位置=%.4f rad (%.2f deg)",
                      motor1_control_rate_,
                      motor1_current_velocity_,
                      motor1_target_position_, motor1_target_position_ * 180.0 / M_PI,
                      motor1_->position_, motor1_->position_ * 180.0 / M_PI);
        }
        counter1 = 0;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "电机控制循环异常，已暂停控制避免程序退出: %s", e.what());
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_last_time_valid_ = false;
      motor2_mouse_target_valid_ = false;
    } catch (...) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "电机控制循环发生未知异常，已暂停控制避免程序退出");
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_last_time_valid_ = false;
      motor2_mouse_target_valid_ = false;
    }
  }

  /**
   * @brief 将RC值映射到电机位置
   * @param rc_value 遥控器值 (364 ~ 1684)
   * @param min_pos 目标最小位置
   * @param max_pos 目标最大位置
   * @return 映射后的目标位置
   */
  double map_rc_to_pos(int rc_value, double min_pos, double max_pos) {
    // 限制输入范围
    int clamped_value = std::max(rc_min_value_, std::min(rc_max_value_, rc_value));

    // 死区处理
    if (std::abs(clamped_value - rc_mid_value_) < deadzone_) {
        // 在死区内，对应输出范围的中值？
        // 或者我们可以定义死区内的输出值。这里简单地映射到中点位置
        // 注意：如果 min_pos 和 max_pos 不是对称的，这就需要调整
        // 这里假设线性映射，死区对应的是 rc_mid_value_ 对应的输出位置
        clamped_value = rc_mid_value_; 
    }

    // 归一化 (0.0 ~ 1.0)
    double normalized = static_cast<double>(clamped_value - rc_min_value_) / 
                        static_cast<double>(rc_max_value_ - rc_min_value_);
    
    // 映射到输出范围
    return min_pos + normalized * (max_pos - min_pos);
  }

  /**
   * @brief 控制单个电机
   * @param motor 电机对象指针
   * @param rc_value 遥控器通道值
   * @param min_position 最小目标位置
   * @param max_position 最大目标位置
   * @param control_speed 控制速度 rad/s
   * @param publisher 目标位置发布者
   * @param motor_name 电机名称（用于日志）
   * @param channel_index 遥控器通道索引
   */
  void control_motor(std::unique_ptr<RobStrideMotor>& motor,
                     int rc_value,
                     double min_position,
                     double max_position,
                     double control_speed,
                     rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher,
                     const std::string& motor_name,
                     int channel_index) {
    
    // 计算目标位置
    double target_position = map_rc_to_pos(rc_value, min_position, max_position);

    // 使用CSP位置模式控制
    motor->RobStrite_Motor_PosCSP_control(
        control_speed, 
        target_position);

    // 发布目标位置
    auto target_msg = std_msgs::msg::Float32();
    target_msg.data = target_position;
    publisher->publish(target_msg);

    // 定期打印信息（降低频率）
    static int counter = 0;
    if (++counter >= 50) { // 每50次打印一次
      RCLCPP_INFO(this->get_logger(),
                  "%s: ch[%d]=%d -> 目标位置: %.4f rad (%.2f deg), "
                  "实际位置: %.4f rad (%.2f deg)",
                  motor_name.c_str(), channel_index, rc_value,
                  target_position, target_position * 180.0 / M_PI,
                  motor->position_, motor->position_ * 180.0 / M_PI);
      counter = 0;
    }
  }

  std::unique_ptr<RobStrideMotor> motor1_;
  std::unique_ptr<RobStrideMotor> motor2_;
  std::atomic<bool> motor1_initialized_;
  std::atomic<bool> motor2_initialized_;
  
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr rc_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr mouse_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor1_target_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor2_target_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor1_actual_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor2_actual_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor1_multi_turn_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor2_multi_turn_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor1_turn_count_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor2_turn_count_publisher_;
  rclcpp::TimerBase::SharedPtr motor1_control_timer_;      ///< 电机1高频控制定时器

  // 电机1参数（速度积分模式）
  std::string motor1_can_interface_;
  double motor1_max_velocity_;             ///< 遥控器映射最大速度 rad/s
  int motor1_channel_index_;
  int motor1_control_rate_;                ///< 控制环频率 Hz
  double motor1_control_speed_;            ///< CSP 位置跟踪速度 rad/s
  double motor1_control_acceleration_;     ///< CSP 加速度
  double motor1_current_velocity_{0.0};    ///< RC 回调更新的当前目标速度 rad/s
  double motor1_target_position_{0.0};     ///< 积分累加的目标位置 rad
  std::chrono::steady_clock::time_point motor1_last_time_; ///< 上次控制时间戳
  bool motor1_last_time_valid_{false};     ///< 时间戳是否已初始化
  bool motor1_multi_turn_valid_{false};
  double motor1_last_raw_position_{0.0};
  int motor1_turn_count_{0};
  double motor1_multi_turn_position_{0.0};

  // 电机2参数（位置映射模式）
  std::string motor2_can_interface_;
  double motor2_min_position_;
  double motor2_max_position_;
  int motor2_channel_index_;
  double motor2_control_speed_;
  double motor2_control_acceleration_;
  bool motor2_multi_turn_valid_{false};
  double motor2_last_raw_position_{0.0};
  int motor2_turn_count_{0};
  double motor2_multi_turn_position_{0.0};

  // 通用参数
  int rc_min_value_;
  int rc_max_value_;
  int rc_mid_value_;
  int deadzone_;
  double mouse_yaw_sensitivity_;
  double mouse_pitch_sensitivity_;
  int mouse_max_speed_;
  GimbalControlSource gimbal_control_source_{GimbalControlSource::HYBRID};
  double input_priority_timeout_;

  // 鼠标输入状态
  int16_t mouse_x_{0};
  int16_t mouse_y_{0};
  rclcpp::Time last_mouse_time_;
  bool mouse_time_valid_{false};
  double motor2_mouse_target_position_{0.0};
  bool motor2_mouse_target_valid_{false};

  // 追踪模式相关
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr switches_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_subscription_;
  bool tracking_mode_{false};                                       ///< 是否处于追踪模式
  std::chrono::steady_clock::time_point trigger_release_time_;      ///< trigger松开时间戳
  bool trigger_release_time_valid_{false};                          ///< trigger松开时间是否有效
  double track_exit_timeout_;                                       ///< 松开trigger后退出追踪超时 s

  // 增量式PI控制器参数
  double track_yaw_kp_;                                             ///< Yaw方向比例增益
  double track_yaw_ki_;                                             ///< Yaw方向积分增益
  double track_pitch_kp_;                                           ///< Pitch方向比例增益
  double track_pitch_ki_;                                           ///< Pitch方向积分增益
  double track_yaw_error_prev_{0.0};                                ///< Yaw方向上一次误差
  double track_pitch_error_prev_{0.0};                              ///< Pitch方向上一次误差
  double motor2_track_target_position_{0.0};                        ///< 追踪模式下电机2目标位置

  // Yaw瞄准偏移校准
  double track_yaw_offset_;                                         ///< Yaw瞄准偏移 rad

  // Yaw速度预测前馈
  double track_yaw_ff_gain_;                                        ///< 前馈增益
  double track_yaw_ff_filter_;                                      ///< 角速度低通滤波系数
  double track_yaw_ff_deadzone_;                                    ///< 前馈角速度死区 rad/s
  double yaw_angular_velocity_filtered_{0.0};                       ///< 滤波后的Yaw角速度 rad/s
  double yaw_ff_prev_{0.0};                                         ///< 上一次前馈值（增量式更新）
  std::chrono::steady_clock::time_point track_armors_last_time_;    ///< 上次armors回调时间
  bool track_armors_time_valid_{false};                             ///< armors时间戳是否有效

  // 弹道下坠补偿参数
  double bullet_velocity_;                                          ///< 弹丸初速度 m/s
  double gravity_;                                                  ///< 重力加速度 m/s²
};

/**
 * @brief 主函数
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GimbalControlNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
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
