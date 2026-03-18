/**
 * @file gimbal_control_node.cpp
 * @brief 云台底层电机控制节点
 *
 * 负责 Pitch/Yaw 轴电机的初始化、CAN 总线管理与高频驱动。
 * 控制指令由上层 auto_aim_node 通过 /gimbal/cmd 话题下发，本节点不包含任何控制逻辑。
 *
 * /gimbal/cmd 消息格式 (std_msgs/Float64MultiArray):
 *   data[0]: mode       — 0.0=普通模式, 1.0=追踪模式
 *   data[1]: yaw_cmd    — 普通模式: Yaw 速度 (rad/s); 追踪模式: Yaw 位置增量 (rad)
 *   data[2]: pitch_cmd  — 普通模式: Pitch 绝对目标位置 (rad); 追踪模式: Pitch 位置增量 (rad)
 */

#include "rm_gimbal_control/motor_cfg.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>

/**
 * @brief 云台底层电机控制节点
 *
 * 仅负责硬件层：电机初始化、CAN 接口管理、高频 CSP 位置指令下发。
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
    this->declare_parameter<std::string>("motor1_can_interface", "can1");
    this->declare_parameter<int>("motor1_id", 0x02);
    this->declare_parameter<double>("motor1_max_velocity", 8.0);
    this->declare_parameter<int>("motor1_control_rate", 200);
    this->declare_parameter<double>("motor1_control_speed", 10.0);
    this->declare_parameter<double>("motor1_control_acceleration", 8.0);

    // 电机2 (Pitch轴) 参数
    this->declare_parameter<std::string>("motor2_can_interface", "can1");
    this->declare_parameter<int>("motor2_id", 0x01);
    this->declare_parameter<double>("motor2_min_position", -0.21);
    this->declare_parameter<double>("motor2_max_position", 0.31);
    this->declare_parameter<double>("motor2_control_speed", 4.0);
    this->declare_parameter<double>("motor2_control_acceleration", 4.0);

    // 通用参数
    this->declare_parameter<int>("master_id", 0xFF);
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

    // CAN 接口管理参数
    this->declare_parameter<bool>("restart_can_on_startup", true);
    this->declare_parameter<int>("can_bitrate", 1000000);
    this->declare_parameter<int>("can_restart_ms", 100);
    this->declare_parameter<bool>("can_use_sudo", true);
    this->declare_parameter<std::string>("can_sudo_password", "");
    this->declare_parameter<int>("init_command_burst_count", 3);
    this->declare_parameter<int>("init_command_burst_interval_ms", 5);

    // 读取参数
    motor1_can_interface_ = this->get_parameter("motor1_can_interface").as_string();
    int motor1_id = this->get_parameter("motor1_id").as_int();
    motor1_control_rate_ = this->get_parameter("motor1_control_rate").as_int();
    motor1_control_speed_ = this->get_parameter("motor1_control_speed").as_double();
    motor1_control_acceleration_ = this->get_parameter("motor1_control_acceleration").as_double();

    motor2_can_interface_ = this->get_parameter("motor2_can_interface").as_string();
    int motor2_id = this->get_parameter("motor2_id").as_int();
    motor2_min_position_ = this->get_parameter("motor2_min_position").as_double();
    motor2_max_position_ = this->get_parameter("motor2_max_position").as_double();
    motor2_control_speed_ = this->get_parameter("motor2_control_speed").as_double();
    motor2_control_acceleration_ = this->get_parameter("motor2_control_acceleration").as_double();

    int master_id = this->get_parameter("master_id").as_int();
    int actuator_type = this->get_parameter("actuator_type").as_int();

    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    recovery_cooldown_ = this->get_parameter("recovery_cooldown").as_double();
    unready_recovery_retry_interval_ =
        this->get_parameter("unready_recovery_retry_interval").as_double();
    motor_recv_timeout_ms_ = this->get_parameter("motor_recv_timeout_ms").as_int();

    restart_can_on_startup_ = this->get_parameter("restart_can_on_startup").as_bool();
    can_bitrate_ = this->get_parameter("can_bitrate").as_int();
    can_restart_ms_ = this->get_parameter("can_restart_ms").as_int();
    can_use_sudo_ = this->get_parameter("can_use_sudo").as_bool();
    can_sudo_password_ = this->get_parameter("can_sudo_password").as_string();
    init_command_burst_count_ = this->get_parameter("init_command_burst_count").as_int();
    init_command_burst_interval_ms_ = this->get_parameter("init_command_burst_interval_ms").as_int();
    if (init_command_burst_count_ < 1) init_command_burst_count_ = 1;
    if (init_command_burst_interval_ms_ < 0) init_command_burst_interval_ms_ = 0;

    if (restart_can_on_startup_) {
      restart_configured_can_interfaces();
    }

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

    RCLCPP_INFO(this->get_logger(), "云台底层控制节点配置:");
    RCLCPP_INFO(this->get_logger(), "  电机1(Yaw): %s ID=0x%02X, 控制频率=%dHz, 速度=%.2f rad/s",
                motor1_can_interface_.c_str(), motor1_id, motor1_control_rate_, motor1_control_speed_);
    RCLCPP_INFO(this->get_logger(), "  电机2(Pitch): %s ID=0x%02X, 范围=[%.2f, %.2f] rad, 速度=%.2f rad/s",
                motor2_can_interface_.c_str(), motor2_id,
                motor2_min_position_, motor2_max_position_, motor2_control_speed_);
    RCLCPP_INFO(this->get_logger(), "  自动恢复冷却: %.1fs，CAN接收超时: %dms",
                recovery_cooldown_, motor_recv_timeout_ms_);
    RCLCPP_INFO(this->get_logger(), "  未就绪恢复触发间隔: %.1fs",
                unready_recovery_retry_interval_);

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

    init_motor(motor1_, motor1_initialized_, "电机1");
    init_motor(motor2_, motor2_initialized_, "电机2");

    RCLCPP_INFO(this->get_logger(), "云台底层控制节点已启动，等待 /gimbal/cmd 指令...");
  }

  /**
   * @brief 析构函数，安全失能电机
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
  double read_motor_mech_position(std::unique_ptr<RobStrideMotor> &motor,
                                  const std::string &motor_name,
                                  double fallback_position) {
    if (!motor) {
      return fallback_position;
    }
    try {
      motor->Get_RobStrite_Motor_parameter(0x7019);
      return static_cast<double>(motor->drw.mechPos.data);
    } catch (const std::exception &e) {
      // 500ms 节流，避免日志刷屏；超时说明电机未响应参数读，后续 CSP 会检测到真正故障
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "%s读取多圈角(mechPos)超时/失败: %s",
                           motor_name.c_str(), e.what());
      return fallback_position;
    }
  }

  /**
   * @brief 对 shell 单引号内的特殊字符进行转义
   * @param input 原始字符串
   * @return 转义后的字符串
   */
  std::string shell_single_quote_escape(const std::string &input) {
    std::string escaped;
    escaped.reserve(input.size() + 8);
    for (const char c : input) {
      if (c == '\'') {
        escaped += "'\\''";
      } else {
        escaped.push_back(c);
      }
    }
    return escaped;
  }

  /**
   * @brief 构造带权限提升的 shell 指令（sudo）
   * @param command 原始命令
   * @return 完整的可执行命令字符串
   */
  std::string build_privileged_command(const std::string &command) {
    if (!can_use_sudo_) {
      return command;
    }
    if (can_sudo_password_.empty()) {
      return "sudo -n " + command;
    }
    const std::string escaped_password = shell_single_quote_escape(can_sudo_password_);
    return "bash -lc \"printf '%s\\n' '" + escaped_password +
           "' | sudo -S -p '' " + command + "\"";
  }

  /**
   * @brief 执行 shell 命令
   * @param command 命令字符串
   * @return 执行成功返回 true
   */
  bool run_shell_command(const std::string &command) {
    const std::string actual_command = build_privileged_command(command);
    return std::system(actual_command.c_str()) == 0;
  }

  /**
   * @brief 重启单个 CAN 接口并配置波特率与自动恢复
   * @param interface_name CAN 接口名（如 can0/can1）
   * @return 执行成功返回 true
   */
  bool restart_can_interface(const std::string &interface_name) {
    if (interface_name.empty()) {
      return false;
    }
    const std::string down_cmd = "ip link set " + interface_name + " down";
    const std::string config_cmd = "ip link set " + interface_name +
                                   " type can bitrate " + std::to_string(can_bitrate_) +
                                   " restart-ms " + std::to_string(can_restart_ms_);
    const std::string up_cmd = "ip link set " + interface_name + " up";

    RCLCPP_INFO(this->get_logger(), "重启CAN接口 %s: bitrate=%d, restart-ms=%d",
                interface_name.c_str(), can_bitrate_, can_restart_ms_);

    bool ok = true;
    if (!run_shell_command(down_cmd)) { ok = false; }
    if (!run_shell_command(config_cmd)) { ok = false; }
    if (!run_shell_command(up_cmd)) { ok = false; }

    if (ok) {
      RCLCPP_INFO(this->get_logger(), "CAN接口 %s 重启完成", interface_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "CAN接口 %s 重启未完全成功（检查权限和接口状态）",
                  interface_name.c_str());
    }
    return ok;
  }

  /**
   * @brief 重启本节点使用到的所有 CAN 接口（去重后依次执行）
   */
  void restart_configured_can_interfaces() {
    std::set<std::string> interfaces;
    interfaces.insert(motor1_can_interface_);
    interfaces.insert(motor2_can_interface_);
    for (const auto &iface : interfaces) {
      if (!iface.empty()) {
        restart_can_interface(iface);
      }
    }
  }

  /**
   * @brief 异步初始化电机（含超时保护与重试）
   * @param motor 电机对象指针
   * @param initialized 初始化完成标志（原子变量）
   * @param motor_name 电机名称（日志用）
   */
  void init_motor(std::unique_ptr<RobStrideMotor> &motor,
                  std::atomic<bool> &initialized,
                  const std::string &motor_name,
                  std::function<void(bool)> on_finished = {}) {
    if (!motor) {
      RCLCPP_ERROR(this->get_logger(), "%s对象未创建", motor_name.c_str());
      if (on_finished) {
        on_finished(false);
      }
      return;
    }
    RCLCPP_INFO(this->get_logger(), "正在初始化%s...", motor_name.c_str());

    std::thread init_thread([this, &motor, &initialized, motor_name,
                             on_finished = std::move(on_finished)]() mutable {
      const int max_retries = 5;

      /**
       * @brief 设置 CAN 接收超时（2秒），防止 receive_status_frame() 在电机断电时永久阻塞
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
            motor->receive_status_frame();
            RCLCPP_INFO(this->get_logger(),
                        "%s初始化前状态: pos=%.4f rad, vel=%.4f rad/s, torque=%.4f Nm, temp=%.1f C, err=0x%02X",
                        motor_name.c_str(),
                        static_cast<double>(motor->position_),
                        static_cast<double>(motor->velocity_),
                        static_cast<double>(motor->torque_),
                        static_cast<double>(motor->temperature_),
                        static_cast<unsigned int>(motor->error_code));
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "%s: 读取初始化前状态失败，继续: %s",
                        motor_name.c_str(), e.what());
          }

          RCLCPP_INFO(this->get_logger(), "%s: 失能电机并清除错误...", motor_name.c_str());
          execute_burst([&motor]() { motor->Disenable_Motor(1); }, "失能清错");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          execute_burst([&motor]() { motor->Get_RobStrite_Motor_parameter(0x7005); }, "读取运行模式");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));

          if (motor->drw.run_mode.data != 5) {
            RCLCPP_INFO(this->get_logger(), "%s当前模式: %.0f, 切换到CSP模式...",
                        motor_name.c_str(), motor->drw.run_mode.data);
            execute_burst([&motor]() { motor->Set_RobStrite_Motor_parameter(0x7005, 5, 'j'); },
                          "设置CSP模式");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            execute_burst([&motor]() { motor->Get_RobStrite_Motor_parameter(0x7005); }, "复读运行模式");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }

          execute_burst([&motor]() { motor->enable_motor(); }, "电机使能");
          initialized = true;

          double startup_position = static_cast<double>(motor->position_);
          try {
            execute_burst([&motor]() { motor->Get_RobStrite_Motor_parameter(0x7019); }, "读取mechPos");
            startup_position = static_cast<double>(motor->drw.mechPos.data);
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "%s: 读取mechPos失败，回退到状态帧位置: %s",
                        motor_name.c_str(), e.what());
          }

          if (motor.get() == motor1_.get()) {
            motor1_target_position_ = startup_position;
            motor1_target_seeded_ = true;
            motor1_last_time_valid_ = false;
          }

          RCLCPP_INFO(this->get_logger(), "%s已使能，启动位置: %.4f rad (%.2f°)",
                      motor_name.c_str(), startup_position, startup_position * 180.0 / M_PI);

          /**
           * @brief 设置控制阶段 CAN 接收超时（motor_recv_timeout_ms_，默认 5ms）
           *
           * 不再使用 {0,0}（永久阻塞）。当电机进入故障态不回复报文时，
           * recv() 会在超时后返回 EAGAIN，电机库随之抛出异常，使控制回调能够
           * 正常捕获错误、打印报警并触发自动恢复，而不是永久冻结整个定时器线程。
           */
          struct timeval ctrl_timeout;
          ctrl_timeout.tv_sec = 0;
          ctrl_timeout.tv_usec = static_cast<suseconds_t>(motor_recv_timeout_ms_ * 1000);
          setsockopt(motor->socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                     &ctrl_timeout, sizeof(ctrl_timeout));
          RCLCPP_INFO(this->get_logger(), "%s: CAN接收超时已设置为 %dms（防止控制循环阻塞）",
                      motor_name.c_str(), motor_recv_timeout_ms_);
          if (on_finished) {
            on_finished(true);
          }
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
      initialized = false;
      if (on_finished) {
        on_finished(false);
      }
    });
    init_thread.detach();
  }

  /**
   * @brief 接收来自 auto_aim_node 的云台控制指令
   *
   * 消息格式:
   *   data[0]: mode       — 0.0=普通, 1.0=追踪
   *   data[1]: yaw_cmd    — 普通:速度(rad/s); 追踪:位置增量(rad)
   *   data[2]: pitch_cmd  — 普通:绝对位置(rad); 追踪:位置增量(rad)
   *
   * 模式切换时：
   *   普通→追踪: 以电机2当前实际位置为追踪起始点
   *   追踪→普通: 以电机1当前实际位置为速度积分起始点（避免位置跳变）
   *
   * @param msg 控制指令消息
   */
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
      motor1_target_position_ = msg->data[0];
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
   * 每次故障后等待 recovery_cooldown_ 秒再重新初始化，防止 CAN 总线未稳定时反复重试。
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
                "电机故障，%.1fs 后自动重新初始化（CAN重启+电机使能）", recovery_cooldown_);

    // 通过一次性定时器延迟执行，避免在主控制回调内长时间阻塞
    recovery_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(recovery_cooldown_ * 1000)),
        [this]() {
          recovery_timer_.reset();
          RCLCPP_WARN(this->get_logger(), "开始自动重新初始化电机...");
          restart_configured_can_interfaces();

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

          init_motor(motor1_, motor1_initialized_, "电机1", on_job_finished);
          init_motor(motor2_, motor2_initialized_, "电机2", on_job_finished);
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
    if (!motor1_initialized_ || !motor1_) {
      const auto now = std::chrono::steady_clock::now();
      if (!recovery_in_progress_) {
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
                           "电机未就绪%s，跳过控制指令",
                           recovery_in_progress_ ? "（自动恢复进行中）" : "（等待初始化）");
      return;
    }

    last_unready_recovery_trigger_time_valid_ = false;

    try {
      if (!motor1_target_seeded_) {
        motor1_target_position_ = static_cast<double>(motor1_->position_);
        motor1_target_seeded_ = true;
        return;
      }

      // ── 电机故障码周期性检查（每 3 秒检测一次）──────────────────────────────
      static int health_counter = 0;
      if (++health_counter >= motor1_control_rate_ * 3) {
        health_counter = 0;
        if (motor1_ && motor1_->error_code != 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "电机1 故障码 0x%02X，触发自动恢复", motor1_->error_code);
          motor1_initialized_ = false;
          motor2_initialized_ = false;
          motor1_target_seeded_ = false;
          trigger_motor_recovery();
          return;
        }
        if (motor2_ && motor2_->error_code != 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "电机2 故障码 0x%02X，触发自动恢复", motor2_->error_code);
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "/gimbal/cmd 指令超时，保持当前位置");
      }

      // ── 下发电机指令 ──────────────────────────────────────────────────────────
      motor1_->RobStrite_Motor_PosCSP_control(motor1_control_speed_, motor1_target_position_);

      if (motor2_initialized_ && motor2_) {
        motor2_->RobStrite_Motor_PosCSP_control(motor2_control_speed_, motor2_cmd_position_);
        auto msg2 = std_msgs::msg::Float32();
        msg2.data = static_cast<float>(motor2_cmd_position_);
        motor2_target_position_publisher_->publish(msg2);
      }

      // 发布电机1目标位置
      auto msg1 = std_msgs::msg::Float32();
      msg1.data = static_cast<float>(motor1_target_position_);
      motor1_target_position_publisher_->publish(msg1);

      // 发布多圈角反馈
      if (motor1_initialized_ && motor1_) {
        motor1_multi_turn_position_ = read_motor_mech_position(
            motor1_, "电机1", motor1_multi_turn_position_);
        auto multi1 = std_msgs::msg::Float32();
        multi1.data = static_cast<float>(motor1_multi_turn_position_);
        motor1_multi_turn_position_publisher_->publish(multi1);
      }
      if (motor2_initialized_ && motor2_) {
        motor2_multi_turn_position_ = read_motor_mech_position(
            motor2_, "电机2", motor2_multi_turn_position_);
        auto multi2 = std_msgs::msg::Float32();
        multi2.data = static_cast<float>(motor2_multi_turn_position_);
        motor2_multi_turn_position_publisher_->publish(multi2);
      }

      // 降频日志（每秒一次，含目标-实际偏差供监控）
      static int log_counter = 0;
      if (++log_counter >= motor1_control_rate_) {
        double pos_err = motor1_target_position_ - static_cast<double>(motor1_->position_);
        RCLCPP_INFO(this->get_logger(),
                    "电机1(控制@%dHz): 目标=%.4f rad, 实际=%.4f rad, 偏差=%.3f rad",
                    motor1_control_rate_,
                    motor1_target_position_,
                    static_cast<double>(motor1_->position_),
                    pos_err);
        log_counter = 0;
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "电机控制异常: %s", e.what());
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_target_seeded_ = false;
      trigger_motor_recovery();
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "电机控制发生未知异常");
      motor1_initialized_ = false;
      motor2_initialized_ = false;
      motor1_target_seeded_ = false;
      trigger_motor_recovery();
    }
  }

  // ===== 电机对象 =====
  std::unique_ptr<RobStrideMotor> motor1_;
  std::unique_ptr<RobStrideMotor> motor2_;
  std::atomic<bool> motor1_initialized_;
  std::atomic<bool> motor2_initialized_;

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
  double motor1_control_speed_;
  double motor1_control_acceleration_;
  double motor1_target_position_{0.0};          ///< 积分/追踪目标位置 rad
  bool motor1_target_seeded_{false};            ///< 目标位置是否已从电机实际位置初始化
  std::chrono::steady_clock::time_point motor1_last_time_;
  bool motor1_last_time_valid_{false};
  double motor1_multi_turn_position_{0.0};

  // ===== 电机2 (Pitch) 参数与状态 =====
  std::string motor2_can_interface_;
  double motor2_min_position_;
  double motor2_max_position_;
  double motor2_control_speed_;
  double motor2_control_acceleration_;
  double motor2_cmd_position_{0.0};            ///< Pitch 目标位置 rad
  double motor2_multi_turn_position_{0.0};

  // ===== 控制状态（来自 /gimbal/cmd） =====
  // ===== 指令超时保护 =====
  rclcpp::Time last_cmd_time_;
  bool cmd_received_{false};
  double cmd_timeout_;

  // ===== Yaw 瞄准偏移校准（硬件参数，追踪模式下叠加在目标位置上） =====
  double track_yaw_offset_;

  // ===== 自动恢复 =====
  double recovery_cooldown_;                                   ///< 故障恢复冷却时间 s
  double unready_recovery_retry_interval_;                     ///< 未就绪时主动恢复触发的最小间隔 s
  int motor_recv_timeout_ms_;                                  ///< 控制阶段 CAN socket 接收超时 ms
  bool recovery_in_progress_{false};                          ///< 是否正在等待恢复
  std::chrono::steady_clock::time_point last_recovery_time_;  ///< 上次触发恢复的时间
  bool last_recovery_time_valid_{false};                      ///< 上次恢复时间是否有效
  std::chrono::steady_clock::time_point last_unready_recovery_trigger_time_;  ///< 上次未就绪恢复触发时间
  bool last_unready_recovery_trigger_time_valid_{false};                     ///< 未就绪恢复触发时间是否有效
  int recovery_jobs_total_{0};                                                ///< 当前恢复批次任务总数
  int recovery_jobs_done_{0};                                                 ///< 当前恢复批次已完成任务数
  int recovery_jobs_success_{0};                                              ///< 当前恢复批次成功任务数

  // ===== CAN 管理参数 =====
  bool restart_can_on_startup_;
  int can_bitrate_;
  int can_restart_ms_;
  bool can_use_sudo_;
  std::string can_sudo_password_;
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
