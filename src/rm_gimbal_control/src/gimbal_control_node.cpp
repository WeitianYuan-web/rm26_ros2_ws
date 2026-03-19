/**
 * @file gimbal_control_node.cpp
 * @brief 云台底层电机控制节点
 *
 * 负责 Pitch/Yaw 轴电机的初始化与高频驱动。
 * 控制指令由上层 auto_aim_node 通过 /gimbal/cmd 话题下发，本节点不包含任何控制逻辑。
 *
 * /gimbal/cmd 消息格式 (std_msgs/Float64MultiArray):
 *   data[0]: Yaw 绝对目标位置 (rad)
 *   data[1]: Pitch 绝对目标位置 (rad)
 */

#include "rm_gimbal_control/rs_05_can_sdk.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

/**
 * @brief 云台底层电机控制节点
 *
 * 仅负责硬件层：电机初始化、高频 CSP 位置指令下发。
 * 所有控制逻辑（遥控/鼠标/自瞄 PI）均在 auto_aim_node 中实现。
 */
class GimbalControlNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，声明并读取参数，初始化电机与 ROS 通信
   */
  GimbalControlNode()
      : rclcpp::Node("gimbal_control_node"),
        motor1_initialized_(false),
        motor2_initialized_(false) {

    // 电机1 (Yaw轴) 参数
    this->declare_parameter<std::string>("motor1_can_interface", "can0");
    this->declare_parameter<int>("motor1_id", 0x02);
    this->declare_parameter<double>("motor1_max_velocity", 12.0);
    this->declare_parameter<int>("motor1_control_rate", 200);
    this->declare_parameter<double>("motor1_control_speed", 12.0);
    this->declare_parameter<double>("motor1_control_acceleration", 10.0);
    this->declare_parameter<double>("motor1_direction_sign", 1.0);

    // 电机2 (Pitch轴) 参数
    this->declare_parameter<std::string>("motor2_can_interface", "can0");
    this->declare_parameter<int>("motor2_id", 0x01);
    this->declare_parameter<double>("motor2_min_position", -0.21);
    this->declare_parameter<double>("motor2_max_position", 0.31);
    this->declare_parameter<double>("motor2_control_speed", 4.0);
    this->declare_parameter<double>("motor2_control_acceleration", 4.0);

    // 通用参数
    this->declare_parameter<int>("master_id", 0xFD);
    this->declare_parameter<int>("actuator_type", 5);

    // 指令超时保护：超过此时间未收到 /gimbal/cmd 则停止目标更新
    this->declare_parameter<double>("cmd_timeout", 0.5);

    // 故障后自动重新初始化的冷却时间（s），防止快速反复重试
    this->declare_parameter<double>("recovery_cooldown", 5.0);
    /**
     * @brief 电机处于未就绪状态时，主动恢复触发的最小间隔（s）
     */
    this->declare_parameter<double>("unready_recovery_retry_interval", 1.0);

    /**
     * @brief 电机 socket 正常工作时的 CAN 接收超时（毫秒）
     *
     * 初始化成功后，将 SO_RCVTIMEO 设为此值而非 0（永久阻塞）。
     * 当电机异常不回复时，recv() 会在超时后返回 EAGAIN 并由电机库抛出异常，
     * 防止定时器回调永久冻结。5ms 在 1Mbps CAN 下对正常电机有充足余量（响应 < 1ms）。
     */
    this->declare_parameter<int>("motor_recv_timeout_ms", 5);
    this->declare_parameter<double>("motor1_reduction_ratio", 1.0);
    this->declare_parameter<double>("motor2_reduction_ratio", 1.0);
    this->declare_parameter<bool>("reconnect_restore_enabled", true);
    this->declare_parameter<double>("reconnect_restore_output_tolerance", 0.08);

    this->declare_parameter<int>("init_command_burst_count", 3);
    this->declare_parameter<int>("init_command_burst_interval_ms", 5);

    // 读取参数
    motor1_can_interface_ = this->get_parameter("motor1_can_interface").as_string();
    int motor1_id = this->get_parameter("motor1_id").as_int();
    motor1_control_rate_ = this->get_parameter("motor1_control_rate").as_int();
    motor1_max_velocity_ = this->get_parameter("motor1_max_velocity").as_double();
    motor1_control_speed_ = this->get_parameter("motor1_control_speed").as_double();
    motor1_control_acceleration_ = this->get_parameter("motor1_control_acceleration").as_double();
    motor1_direction_sign_ = this->get_parameter("motor1_direction_sign").as_double();
    motor1_direction_sign_ = (motor1_direction_sign_ >= 0.0) ? 1.0 : -1.0;

    motor2_can_interface_ = this->get_parameter("motor2_can_interface").as_string();
    int motor2_id = this->get_parameter("motor2_id").as_int();
    motor2_min_position_ = this->get_parameter("motor2_min_position").as_double();
    motor2_max_position_ = this->get_parameter("motor2_max_position").as_double();
    motor2_control_speed_ = this->get_parameter("motor2_control_speed").as_double();
    motor2_control_acceleration_ = this->get_parameter("motor2_control_acceleration").as_double();

    int master_id = this->get_parameter("master_id").as_int();
    // int actuator_type = this->get_parameter("actuator_type").as_int();

    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    recovery_cooldown_ = this->get_parameter("recovery_cooldown").as_double();
    unready_recovery_retry_interval_ =
        this->get_parameter("unready_recovery_retry_interval").as_double();
    motor_recv_timeout_ms_ = this->get_parameter("motor_recv_timeout_ms").as_int();
    motor1_reduction_ratio_ = this->get_parameter("motor1_reduction_ratio").as_double();
    motor2_reduction_ratio_ = this->get_parameter("motor2_reduction_ratio").as_double();
    reconnect_restore_enabled_ = this->get_parameter("reconnect_restore_enabled").as_bool();
    reconnect_restore_output_tolerance_ =
        this->get_parameter("reconnect_restore_output_tolerance").as_double();
    if (motor1_reduction_ratio_ <= 0.0) motor1_reduction_ratio_ = 1.0;
    if (motor2_reduction_ratio_ <= 0.0) motor2_reduction_ratio_ = 1.0;
    if (reconnect_restore_output_tolerance_ < 0.0) reconnect_restore_output_tolerance_ = 0.0;

    init_command_burst_count_ = this->get_parameter("init_command_burst_count").as_int();
    init_command_burst_interval_ms_ = this->get_parameter("init_command_burst_interval_ms").as_int();
    if (init_command_burst_count_ < 1) init_command_burst_count_ = 1;
    if (init_command_burst_interval_ms_ < 0) init_command_burst_interval_ms_ = 0;

    motor1_ = std::make_unique<rs_05_can_sdk::RsMotorController>(
        motor1_can_interface_,
        static_cast<uint16_t>(master_id),
        static_cast<uint8_t>(motor1_id));

    motor2_ = std::make_unique<rs_05_can_sdk::RsMotorController>(
        motor2_can_interface_,
        static_cast<uint16_t>(master_id),
        static_cast<uint8_t>(motor2_id));

    RCLCPP_INFO(this->get_logger(), "云台底层控制节点配置:");
    RCLCPP_INFO(this->get_logger(), "  电机1(Yaw): %s ID=0x%02X, 控制频率=%dHz, 速度=%.2f rad/s",
                motor1_can_interface_.c_str(), motor1_id, motor1_control_rate_, motor1_control_speed_);
    RCLCPP_INFO(this->get_logger(), "  电机1(Yaw) 方向符号: %.0f",
                motor1_direction_sign_);
    RCLCPP_INFO(this->get_logger(), "  电机2(Pitch): %s ID=0x%02X, 范围=[%.2f, %.2f] rad, 速度=%.2f rad/s",
                motor2_can_interface_.c_str(), motor2_id,
                motor2_min_position_, motor2_max_position_, motor2_control_speed_);
    RCLCPP_INFO(this->get_logger(), "  自动恢复冷却: %.1fs，CAN接收超时: %dms",
                recovery_cooldown_, motor_recv_timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "  未就绪恢复触发间隔: %.1fs",
                unready_recovery_retry_interval_);
    RCLCPP_INFO(this->get_logger(),
                "  重连角度恢复: %s, 减速比[m1=%.3f,m2=%.3f], 输出轴容差=%.4f rad",
                reconnect_restore_enabled_ ? "开启" : "关闭",
                motor1_reduction_ratio_, motor2_reduction_ratio_,
                reconnect_restore_output_tolerance_);

    // 发布者：监控目标位置与多圈角
    motor1_target_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor1/target_position", 10);
    motor2_target_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor2/target_position", 10);
    motor1_multi_turn_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor1/multi_turn_position", 10);
    motor2_multi_turn_position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor2/multi_turn_position", 10);

    // 订阅来自 auto_aim_node 的云台控制指令
    gimbal_cmd_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gimbal/cmd",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&GimbalControlNode::gimbal_cmd_callback, this, std::placeholders::_1));

    // 高频电机控制定时器
    int period_ms = 1000 / motor1_control_rate_;
    motor_control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&GimbalControlNode::motor_control_timer_callback, this));

    init_motor(motor1_, motor1_initialized_, motor1_init_in_progress_, "电机1");
    init_motor(motor2_, motor2_initialized_, motor2_init_in_progress_, "电机2");

    RCLCPP_INFO(this->get_logger(), "云台底层控制节点已启动，等待 /gimbal/cmd 指令...");
  }

  /**
   * @brief 析构函数，安全失能电机
   */
  ~GimbalControlNode() {
    RCLCPP_INFO(this->get_logger(), "正在停止电机...");
    if (motor1_initialized_ && motor1_) {
      try {
        motor1_->disable_motor(0);
        RCLCPP_INFO(this->get_logger(), "电机1已停止");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "停止电机1时出错: %s", e.what());
      }
    }
    if (motor2_initialized_ && motor2_) {
      try {
        motor2_->disable_motor(0);
        RCLCPP_INFO(this->get_logger(), "电机2已停止");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "停止电机2时出错: %s", e.what());
      }
    }
  }

private:
  /**
   * @brief 读取电机原始多圈角（mechPos），失败时保留上次有效值
   * @param motor 电机对象指针
   * @param motor_name 电机名称（日志用）
   * @param fallback_position 读取失败时的回退值
   * @return 电机机械位置 rad
   */
  /**
   * @brief 读取电机原始多圈角（mechPos），失败时保留上次有效值
   *
   * 此函数发送参数读请求（0x7019）并等待电机回复。依赖 socket 上已设置的
   * SO_RCVTIMEO（motor_recv_timeout_ms_）来限制等待时间，防止在电机故障时阻塞。
   * 异常由本函数内部吞掉（返回 fallback），不触发上层恢复流程——
   * 真正的故障会在 RobStrite_Motor_PosCSP_control 的 recv 超时时抛出并被
   * motor_control_timer_callback 的 catch 块捕获。
   *
   * @param motor        电机对象指针
   * @param motor_name   电机名称（日志用）
   * @param fallback_position 读取失败时的回退值
   * @return 电机机械位置 rad
   */
  double read_motor_mech_position(std::unique_ptr<rs_05_can_sdk::RsMotorController> &motor,
                                  const std::string &motor_name,
                                  double fallback_position) {
    if (!motor) {
      return fallback_position;
    }
    try {
      return static_cast<double>(motor->get_mech_position());
    } catch (const std::exception &e) {
      // 500ms 节流，避免日志刷屏；超时说明电机未响应参数读，后续 CSP 会检测到真正故障
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "%s读取多圈角(mechPos)超时/失败: %s",
                           motor_name.c_str(), e.what());
      return fallback_position;
    }
  }

  /**
   * @brief 在判定故障前缓存当前多圈角，用于重连后尝试恢复
   */
  void snapshot_pre_drop_positions() {
    if (motor1_ && motor1_initialized_) {
      double v = motor1_multi_turn_position_;
      if (!std::isfinite(v)) {
        v = motor1_->position_.load();
      }
      motor1_pre_drop_multi_turn_.store(v);
      motor1_pre_drop_valid_.store(std::isfinite(v));
    }
    if (motor2_ && motor2_initialized_) {
      double v = motor2_multi_turn_position_;
      if (!std::isfinite(v)) {
        v = motor2_->position_.load();
      }
      motor2_pre_drop_multi_turn_.store(v);
      motor2_pre_drop_valid_.store(std::isfinite(v));
    }
  }

  /**
   * @brief 尝试在重连后恢复掉线前多圈角
   *
   * 恢复判据：将“当前读到的电机角”换算到输出轴角度后，按 2π 周期寻找与掉线前输出轴角最接近的同位角，
   * 若差值小于容差，认为掉线期间电机仅小幅转动，可安全恢复掉线前多圈角。
   *
   * @param motor 电机对象
   * @param motor_name 电机名称
   * @param startup_position 重连后读取到的当前多圈角
   * @param pre_drop_valid 掉线前角度缓存是否有效
   * @param pre_drop_position 掉线前多圈角
   * @param reduction_ratio 减速比（电机角/输出轴角）
   * @return 作为当前逻辑多圈角使用的值（恢复成功则为掉线前角，否则为 startup_position）
   */
  double restore_multi_turn_if_needed(
      std::unique_ptr<rs_05_can_sdk::RsMotorController> &motor,
      const std::string &motor_name,
      double startup_position,
      std::atomic<bool> &pre_drop_valid,
      std::atomic<double> &pre_drop_position,
      double reduction_ratio) {
    if (!motor || !reconnect_restore_enabled_) {
      pre_drop_valid.store(false);
      return startup_position;
    }
    if (!pre_drop_valid.load()) {
      return startup_position;
    }
    if (reduction_ratio <= 0.0 || !std::isfinite(startup_position)) {
      pre_drop_valid.store(false);
      return startup_position;
    }

    const double pre = pre_drop_position.load();
    if (!std::isfinite(pre)) {
      pre_drop_valid.store(false);
      return startup_position;
    }

    const double pre_output = pre / reduction_ratio;
    const double startup_output = startup_position / reduction_ratio;
    const double cycle = 2.0 * M_PI;
    const double k = std::round((pre_output - startup_output) / cycle);
    const double aligned_output = startup_output + k * cycle;
    const double diff = std::fabs(aligned_output - pre_output);

    pre_drop_valid.store(false);

    if (diff <= reconnect_restore_output_tolerance_) {
      motor->position_.store(pre);
      RCLCPP_WARN(this->get_logger(),
                  "%s重连后恢复多圈角: startup=%.4f -> restore=%.4f rad, 输出轴偏差=%.6f rad",
                  motor_name.c_str(), startup_position, pre, diff);
      return pre;
    }

    RCLCPP_WARN(this->get_logger(),
                "%s重连后未恢复掉线前多圈角: startup=%.4f, pre_drop=%.4f, 输出轴偏差=%.6f rad(阈值=%.6f)",
                motor_name.c_str(), startup_position, pre, diff, reconnect_restore_output_tolerance_);
    return startup_position;
  }

  /**
   * @brief 异步初始化电机（含超时保护与重试）
   * @param motor 电机对象指针
   * @param initialized 初始化完成标志（原子变量）
   * @param motor_name 电机名称（日志用）
   */
  void init_motor(std::unique_ptr<rs_05_can_sdk::RsMotorController> &motor,
                  std::atomic<bool> &initialized,
                  std::atomic<bool> &init_in_progress,
                  const std::string &motor_name,
                  std::function<void(bool)> on_finished = {}) {
    if (!motor) {
      RCLCPP_ERROR(this->get_logger(), "%s对象未创建", motor_name.c_str());
      if (on_finished) {
        on_finished(false);
      }
      return;
    }
    if (init_in_progress.exchange(true)) {
      RCLCPP_WARN(this->get_logger(), "%s初始化已在进行中，跳过重复触发", motor_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "正在初始化%s...", motor_name.c_str());

    std::thread init_thread([this, &motor, &initialized, &init_in_progress, motor_name,
                             on_finished = std::move(on_finished)]() mutable {
      auto finish_init = [&](bool ok) {
        initialized = ok;
        init_in_progress = false;
        if (on_finished) {
          on_finished(ok);
        }
      };

      const int max_retries = 5;

      /**
       * @brief 设置 CAN 接收超时（2秒），防止 receive_status_frame() 在电机断电时永久阻塞
       */
      struct timeval recv_timeout;
      recv_timeout.tv_sec = 2;
      recv_timeout.tv_usec = 0;
      if (setsockopt(motor->socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
                     &recv_timeout, sizeof(recv_timeout)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "%s: 设置CAN接收超时失败", motor_name.c_str());
      }

      for (int attempt = 1; attempt <= max_retries; ++attempt) {
        try {
          auto execute_burst = [this, &motor_name](
                                   const std::function<void()> &send_once,
                                   const std::string &command_name) {
            std::exception_ptr last_error;
            for (int i = 0; i < init_command_burst_count_; ++i) {
              try {
                send_once();
                if (i > 0) {
                  RCLCPP_INFO(this->get_logger(), "%s: %s在第%d次连发成功",
                              motor_name.c_str(), command_name.c_str(), i + 1);
                }
                return;
              } catch (...) {
                last_error = std::current_exception();
              }
              if (i + 1 < init_command_burst_count_ && init_command_burst_interval_ms_ > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(init_command_burst_interval_ms_));
              }
            }
            if (last_error) {
              std::rethrow_exception(last_error);
            }
          };

          RCLCPP_INFO(this->get_logger(), "%s: 初始化尝试 %d/%d ...",
                      motor_name.c_str(), attempt, max_retries);

          try {
            RCLCPP_INFO(this->get_logger(), "%s: 读取初始化前状态...", motor_name.c_str());
            // TODO: Receive and update motor position state internally
            RCLCPP_INFO(this->get_logger(),
                        "%s初始化前状态: pos=%.4f rad",
                        motor_name.c_str(),
                        motor->position_.load());
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "%s: 读取初始化前状态失败，继续: %s",
                        motor_name.c_str(), e.what());
          }

          RCLCPP_INFO(this->get_logger(), "%s: 失能电机并清除错误...", motor_name.c_str());
          execute_burst([&motor]() { motor->disable_motor(1); }, "失能清错");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          execute_burst([&motor]() { motor->get_mode(); }, "读取运行模式");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          // 假定5表示CSP模式，如果不是则切换
          RCLCPP_INFO(this->get_logger(), "%s切换到CSP模式...", motor_name.c_str());
          execute_burst([&motor]() { motor->set_mode(5); },
                        "设置CSP模式");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          execute_burst([&motor]() { motor->enable_motor(); }, "电机使能");

          try {
            // 发送 mechPos 读取请求，等待 receive_loop 处理响应后再读取位置
            execute_burst([&motor]() { motor->get_mech_position(); }, "读取mechPos");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "%s: 读取mechPos失败，回退到状态帧位置: %s",
                        motor_name.c_str(), e.what());
          }

          double startup_position = motor->position_.load();
          startup_position = (motor.get() == motor1_.get())
                                 ? restore_multi_turn_if_needed(
                                       motor, motor_name, startup_position,
                                       motor1_pre_drop_valid_, motor1_pre_drop_multi_turn_,
                                       motor1_reduction_ratio_)
                                 : restore_multi_turn_if_needed(
                                       motor, motor_name, startup_position,
                                       motor2_pre_drop_valid_, motor2_pre_drop_multi_turn_,
                                       motor2_reduction_ratio_);

          if (motor.get() == motor1_.get()) {
            motor1_target_position_ = startup_position;
            motor1_target_seeded_ = true;
            motor1_multi_turn_position_ = startup_position;
          } else if (motor.get() == motor2_.get()) {
            motor2_cmd_position_ = std::max(motor2_min_position_,
                                            std::min(motor2_max_position_, startup_position));
            motor2_multi_turn_position_ = startup_position;
          }

          RCLCPP_INFO(this->get_logger(), "%s已使能，启动位置: %.4f rad (%.2f°)",
                      motor_name.c_str(), startup_position, startup_position * 180.0 / M_PI);

          struct timeval ctrl_timeout;
          ctrl_timeout.tv_sec = 0;
          ctrl_timeout.tv_usec = static_cast<suseconds_t>(motor_recv_timeout_ms_ * 1000);
          setsockopt(motor->socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
                     &ctrl_timeout, sizeof(ctrl_timeout));
          RCLCPP_INFO(this->get_logger(), "%s: CAN接收超时已设置为 %dms（防止控制循环阻塞）",
                      motor_name.c_str(), motor_recv_timeout_ms_);
          finish_init(true);
          return;

        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "%s: 初始化尝试 %d/%d 失败: %s",
                      motor_name.c_str(), attempt, max_retries, e.what());
          if (attempt < max_retries) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
        }
      }

      RCLCPP_ERROR(this->get_logger(), "%s初始化失败（已重试%d次）", motor_name.c_str(), max_retries);
      RCLCPP_ERROR(this->get_logger(), "请检查: 1) CAN接口 2) 电机连接与上电 3) 电机ID");
      finish_init(false);
    });
    init_thread.detach();
  }

  /**
   * @brief 控制指令消息回调，接收来自 auto_aim_node 的绝对位置指令
   *
   * 消息格式:
   *   data[0]: Yaw 绝对目标位置 (rad)
   *   data[1]: Pitch 绝对目标位置 (rad)
   *
   * @param msg 控制指令消息
   */
  void gimbal_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "/gimbal/cmd 数据长度不足: %zu (需要 2)", msg->data.size());
      return;
    }

    if (motor1_initialized_ && motor1_) {
      motor1_target_position_ = motor1_direction_sign_ * msg->data[0];
      motor1_target_seeded_ = true;
    }

    if (motor2_initialized_ && motor2_) {
      motor2_cmd_position_ = std::max(motor2_min_position_,
                                      std::min(motor2_max_position_, msg->data[1]));
    }

    last_cmd_time_ = this->now();
    cmd_received_ = true;
  }

  /**
   * @brief 触发电机自动恢复（带冷却时间，通过延迟定时器避免在回调内直接阻塞初始化）
   *
   * 每次故障后等待 recovery_cooldown_ 秒再重新初始化，防止快速反复重试。
   */
  void trigger_motor_recovery() {
    if (recovery_in_progress_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "已有恢复流程进行中，跳过本次触发");
      return;
    }

    if (last_recovery_time_valid_) {
      double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_recovery_time_).count();
      if (elapsed < recovery_cooldown_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "距上次恢复仅 %.1fs，冷却中（%.1fs后可再试）",
                             elapsed, recovery_cooldown_ - elapsed);
        return;
      }
    }

    recovery_in_progress_ = true;
    last_recovery_time_ = std::chrono::steady_clock::now();
    last_recovery_time_valid_ = true;

    RCLCPP_WARN(this->get_logger(),
                "电机故障，%.1fs 后自动重新初始化（电机使能）", recovery_cooldown_);

    // 通过一次性定时器延迟执行，避免在主控制回调内长时间阻塞
    recovery_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(recovery_cooldown_ * 1000)),
        [this]() {
          recovery_timer_.reset();
          RCLCPP_WARN(this->get_logger(), "开始自动重新初始化电机...");

          recovery_jobs_total_ = 2;
          recovery_jobs_done_ = 0;
          recovery_jobs_success_ = 0;

          auto on_job_finished = [this](bool ok) {
            if (ok) {
              ++recovery_jobs_success_;
            }
            ++recovery_jobs_done_;

            if (recovery_jobs_done_ >= recovery_jobs_total_) {
              RCLCPP_WARN(this->get_logger(),
                          "恢复尝试结束: 成功 %d/%d（电机1=%s, 电机2=%s）",
                          recovery_jobs_success_, recovery_jobs_total_,
                          motor1_initialized_ ? "OK" : "FAIL",
                          motor2_initialized_ ? "OK" : "FAIL");
              recovery_in_progress_ = false;
            }
          };

          init_motor(motor1_, motor1_initialized_, motor1_init_in_progress_, "电机1", on_job_finished);
          init_motor(motor2_, motor2_initialized_, motor2_init_in_progress_, "电机2", on_job_finished);
        });
  }

  /**
   * @brief 高频电机控制定时器回调（默认 200Hz）
   *
   * 负责将来自 auto_aim_node 的绝对目标位置直接下发给电机。
   *
   * 关键保护机制:
   *   - 电机故障检测: 周期性检查 error_code，非零时自动触发恢复。
   *   - 自动恢复: 异常后不永久停止，调用 trigger_motor_recovery() 等待冷却后重新初始化。
   *   - 超时保护: 超过 cmd_timeout_ 未收到指令，则保持当前目标位置不动。
   */
  void motor_control_timer_callback() {
    const bool motor1_ready = motor1_initialized_ && motor1_;
    const bool motor2_ready = motor2_initialized_ && motor2_;

    if (!motor1_ready && !motor2_ready) {
      const auto now = std::chrono::steady_clock::now();
      const bool init_in_progress = motor1_init_in_progress_.load() || motor2_init_in_progress_.load();
      if (!recovery_in_progress_ && !init_in_progress) {
        const bool can_retry =
            !last_unready_recovery_trigger_time_valid_ ||
            (std::chrono::duration<double>(now - last_unready_recovery_trigger_time_).count() >=
             unready_recovery_retry_interval_);
        if (can_retry) {
          last_unready_recovery_trigger_time_ = now;
          last_unready_recovery_trigger_time_valid_ = true;
          RCLCPP_WARN(this->get_logger(), "电机未就绪，主动触发恢复流程");
          trigger_motor_recovery();
        }
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "双电机均未就绪%s，跳过控制指令",
                           recovery_in_progress_ ? "（自动恢复进行中）"
                                                 : (init_in_progress ? "（初始化进行中）" : "（等待初始化）"));
      return;
    }

    if (!motor1_ready || !motor2_ready) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "部分电机未就绪: motor1=%s, motor2=%s，继续控制可用电机并重试恢复",
                           motor1_ready ? "OK" : "NOT_READY",
                           motor2_ready ? "OK" : "NOT_READY");
      if (!recovery_in_progress_ && !motor1_init_in_progress_.load() && !motor2_init_in_progress_.load()) {
        const auto now = std::chrono::steady_clock::now();
        const bool can_retry =
            !last_unready_recovery_trigger_time_valid_ ||
            (std::chrono::duration<double>(now - last_unready_recovery_trigger_time_).count() >=
             unready_recovery_retry_interval_);
        if (can_retry) {
          last_unready_recovery_trigger_time_ = now;
          last_unready_recovery_trigger_time_valid_ = true;
          trigger_motor_recovery();
        }
      }
    }

    last_unready_recovery_trigger_time_valid_ = false;

    try {
      if (motor1_ready && !motor1_target_seeded_) {
        motor1_target_position_ = motor1_->position_.load();
        motor1_target_seeded_ = true;
      }

      // ── 电机故障码周期性检查（每 3 秒检测一次）──────────────────────────────
      static int health_counter = 0;
      if (++health_counter >= motor1_control_rate_ * 3) {
        health_counter = 0;
        if (motor1_ready && motor1_->error_code_.load() != 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "电机1 故障码 0x%02X，触发自动恢复", motor1_->error_code_.load());
          snapshot_pre_drop_positions();
          motor1_initialized_ = false;
          motor2_initialized_ = false;
          motor1_target_seeded_ = false;
          trigger_motor_recovery();
          return;
        }
        if (motor2_ready && motor2_->error_code_.load() != 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "电机2 故障码 0x%02X，触发自动恢复", motor2_->error_code_.load());
          snapshot_pre_drop_positions();
          motor1_initialized_ = false;
          motor2_initialized_ = false;
          motor1_target_seeded_ = false;
          trigger_motor_recovery();
          return;
        }
      }

      const bool cmd_active = cmd_received_ &&
          ((this->now() - last_cmd_time_).seconds() <= cmd_timeout_);

      if (!cmd_active) {
        // 无有效上层指令时，回贴当前反馈角，避免上电后被旧目标拉动产生自转
        if (motor1_ready) {
          motor1_target_position_ = motor1_->position_.load();
        }
        if (motor2_ready) {
          motor2_cmd_position_ = std::max(motor2_min_position_,
                                          std::min(motor2_max_position_, motor2_->position_.load()));
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "/gimbal/cmd 指令超时，保持当前位置");
      }

      // ── 下发电机指令 ──────────────────────────────────────────────────────────
      if (cmd_active) {
        if (motor1_ready && motor1_target_seeded_) {
          motor1_->set_pos_csp(motor1_control_speed_, motor1_target_position_);
        }

        if (motor2_ready) {
          motor2_->set_pos_csp(motor2_control_speed_, motor2_cmd_position_);
        }
      }

      if (motor2_ready) {
        auto msg2 = std_msgs::msg::Float32();
        msg2.data = static_cast<float>(motor2_cmd_position_);
        motor2_target_position_publisher_->publish(msg2);
      }

      // 发布电机1目标位置
      if (motor1_ready) {
        auto msg1 = std_msgs::msg::Float32();
        msg1.data = static_cast<float>(motor1_direction_sign_ * motor1_target_position_);
        motor1_target_position_publisher_->publish(msg1);
      }

      // 发布多圈角反馈
      if (motor1_ready) {
        motor1_multi_turn_position_ = read_motor_mech_position(
            motor1_, "电机1", motor1_multi_turn_position_);
        auto multi1 = std_msgs::msg::Float32();
        multi1.data = static_cast<float>(motor1_direction_sign_ * motor1_multi_turn_position_);
        motor1_multi_turn_position_publisher_->publish(multi1);
      }
      if (motor2_ready) {
        motor2_multi_turn_position_ = read_motor_mech_position(
            motor2_, "电机2", motor2_multi_turn_position_);
        auto multi2 = std_msgs::msg::Float32();
        multi2.data = static_cast<float>(motor2_multi_turn_position_);
        motor2_multi_turn_position_publisher_->publish(multi2);
      }

      // 降频日志（每秒一次，含目标-实际偏差供监控）
      static int log_counter = 0;
      if (motor1_ready && ++log_counter >= motor1_control_rate_) {
        double pos_err = motor1_target_position_ - motor1_->position_.load();
        RCLCPP_INFO(this->get_logger(),
                    "电机1(控制@%dHz): 目标=%.4f rad, 实际=%.4f rad, 偏差=%.3f rad",
                    motor1_control_rate_,
                    motor1_target_position_,
                    motor1_->position_.load(),
                    pos_err);
        log_counter = 0;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "电机控制异常: %s", e.what());
      snapshot_pre_drop_positions();
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_target_seeded_ = false;
      trigger_motor_recovery();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "电机控制发生未知异常");
      snapshot_pre_drop_positions();
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_target_seeded_ = false;
      trigger_motor_recovery();
    }
  }

  // ===== 电机对象 =====
  std::unique_ptr<rs_05_can_sdk::RsMotorController> motor1_;
  std::unique_ptr<rs_05_can_sdk::RsMotorController> motor2_;
  std::atomic<bool> motor1_initialized_;
  std::atomic<bool> motor2_initialized_;
  std::atomic<bool> motor1_init_in_progress_{false};
  std::atomic<bool> motor2_init_in_progress_{false};

  // ===== ROS 通信 =====
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor1_target_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor2_target_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor1_multi_turn_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor2_multi_turn_position_publisher_;
  rclcpp::TimerBase::SharedPtr motor_control_timer_;
  rclcpp::TimerBase::SharedPtr recovery_timer_;        ///< 故障恢复延迟定时器（一次性）

  // ===== 电机1 (Yaw) 参数与状态 =====
  std::string motor1_can_interface_;
  int motor1_control_rate_;
  double motor1_max_velocity_;
  double motor1_control_speed_;
  double motor1_control_acceleration_;
  double motor1_reduction_ratio_{1.0};         ///< Yaw 电机减速比（电机角/输出轴角）
  double motor1_direction_sign_{1.0};           ///< Yaw 逻辑坐标到电机原始坐标的方向符号（+1/-1）
  double motor1_target_position_{0.0};          ///< Yaw 电机原始目标位置（rad）
  bool motor1_target_seeded_{false};            ///< 目标位置是否已从电机实际位置初始化
  double motor1_multi_turn_position_{0.0};
  std::atomic<bool> motor1_pre_drop_valid_{false};   ///< 掉线前多圈角缓存是否有效
  std::atomic<double> motor1_pre_drop_multi_turn_{0.0};  ///< 掉线前多圈角缓存

  // ===== 电机2 (Pitch) 参数与状态 =====
  std::string motor2_can_interface_;
  double motor2_min_position_;
  double motor2_max_position_;
  double motor2_control_speed_;
  double motor2_control_acceleration_;
  double motor2_reduction_ratio_{1.0};         ///< Pitch 电机减速比（电机角/输出轴角）
  double motor2_cmd_position_{0.0};            ///< Pitch 目标位置 rad
  double motor2_multi_turn_position_{0.0};
  std::atomic<bool> motor2_pre_drop_valid_{false};   ///< 掉线前多圈角缓存是否有效
  std::atomic<double> motor2_pre_drop_multi_turn_{0.0};  ///< 掉线前多圈角缓存

  // ===== 控制状态（来自 /gimbal/cmd） =====
  // ===== 指令超时保护 =====
  rclcpp::Time last_cmd_time_;
  bool cmd_received_{false};
  double cmd_timeout_;

  // ===== 自动恢复 =====
  double recovery_cooldown_;                                   ///< 故障恢复冷却时间 s
  double unready_recovery_retry_interval_;                     ///< 未就绪时主动恢复触发的最小间隔 s
  int motor_recv_timeout_ms_;                                  ///< 控制阶段 CAN socket 接收超时 ms
  bool reconnect_restore_enabled_{true};                       ///< 是否启用重连后角度恢复
  double reconnect_restore_output_tolerance_{0.08};            ///< 重连恢复判据：输出轴角误差阈值 rad
  bool recovery_in_progress_{false};                          ///< 是否正在等待恢复
  std::chrono::steady_clock::time_point last_recovery_time_;  ///< 上次触发恢复的时间
  bool last_recovery_time_valid_{false};                      ///< 上次恢复时间是否有效
  std::chrono::steady_clock::time_point last_unready_recovery_trigger_time_;  ///< 上次未就绪恢复触发时间
  bool last_unready_recovery_trigger_time_valid_{false};                     ///< 未就绪恢复触发时间是否有效
  int recovery_jobs_total_{0};                                                ///< 当前恢复批次任务总数
  int recovery_jobs_done_{0};                                                 ///< 当前恢复批次已完成任务数
  int recovery_jobs_success_{0};                                              ///< 当前恢复批次成功任务数

  int init_command_burst_count_;
  int init_command_burst_interval_ms_;
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
