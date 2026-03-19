/**
 * @file chassis_control_node.cpp
 * @brief 底盘运动控制节点
 *
 * @details
 * 订阅图传遥控器通道数据 (/vt_remote/channels) 和键鼠数据，
 * 计算目标速度并发布到 /chassis/command，交由通讯节点发送给 MCU。
 *
 * 通道映射：
 *   - ch2  (index 2) → vx  x 方向线速度
 *   - ch3  (index 3) → vy  y 方向线速度
 *   - wheel(index 4) → vw  角速度（逆时针为正）
 *   - 通道值域 364 ~ 1684，中值 1024
 *
 * 供弹速度控制逻辑：
 *   - 订阅 /vt_remote/key_toggles，使用 fn_right 切换状态跟踪 booster 高/低速模式
 *   - 低速模式: 供弹速度固定为 0，fn_left 切换变化忽略
 *   - 高速模式: fn_left 切换状态变化时在 -3200 RPM 与 1200 RPM 之间切换
 *   - 从高速切回低速时，供弹速度自动归零
 *
 * 订阅话题：
 *   - /vt_remote/channels  (std_msgs/Int16MultiArray) : [ch0, ch1, ch2, ch3, wheel]
 *   - /vt_remote/keyboard  (std_msgs/UInt16)
 *   - /vt_remote/mouse     (std_msgs/Int16MultiArray)
 *   - /vt_remote/switches  (std_msgs/Int16MultiArray) : [mode, pause, fn_left, fn_right, trigger]
 *   - /vt_remote/key_toggles  (std_msgs/Int16MultiArray) : [pause, fn_left, fn_right, trigger]
 *   - motor1/multi_turn_position (std_msgs/Float32) : 云台 yaw 电机多圈角（电机侧，rad）
 *
 * 发布话题：
 *   - /chassis/command (std_msgs/Float32MultiArray):
 *     [max_linear_accel, max_angular_accel, max_linear_vel, max_angular_vel, vx, vy, vw, feed_rpm]
 *
 * 参数：
 *   - max_linear_accel  (double) : 最大线加速度 (m/s²)，默认 4.0
 *   - max_angular_accel (double) : 最大角加速度 (rad/s²)，默认 8.0
 *   - max_linear_vel    (double) : 最大线速度 (m/s)，默认 1.5
 *   - max_angular_vel   (double) : 最大角速度 (rad/s)，默认 4.0
 *   - min_linear_vel    (double) : 最小线速度 (m/s)，默认 1.0；当指令速度非零且小于此值时按此速度运动
 *   - deadzone     (int)    : 摇杆死区（原始值），默认 2
 *   - gimbal_yaw_zero_offset (double) : 云台 yaw 零点偏移 (rad)，默认 0.0
 *   - fire_control_source (string) : 发射控制输入源，默认 "hybrid"
 *   - input_priority_timeout (double) : 键鼠优先窗口时长，默认 0.3
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>

/* ========================================================================= */
/*  摇杆通道常量                                                              */
/* ========================================================================= */

static constexpr int16_t CH_CENTER = 1024;
static constexpr int16_t CH_MIN = 364;
static constexpr int16_t CH_MAX = 1684;
static constexpr double CH_HALF_RANGE = static_cast<double>(CH_MAX - CH_CENTER);  // 660
static constexpr double GIMBAL_YAW_REDUCTION_RATIO = 2.0135;

enum KeyboardBit : uint16_t
{
  KB_W = (1u << 0),
  KB_S = (1u << 1),
  KB_A = (1u << 2),
  KB_D = (1u << 3),
  KB_Q = (1u << 6),
  KB_E = (1u << 7)
};

enum class FireControlSource
{
  MOUSE = 0,
  REMOTE = 1,
  HYBRID = 2
};

/**
 * @class ChassisControlNode
 * @brief 底盘运动控制 ROS2 节点
 */
class ChassisControlNode : public rclcpp::Node
{
public:
  ChassisControlNode()
  : Node("chassis_control_node")
  {
    /* -------- 声明 & 读取参数 -------- */
    this->declare_parameter<double>("max_linear_accel", 4.0);
    this->declare_parameter<double>("max_angular_accel", 8.0);
    this->declare_parameter<double>("max_linear_vel", 1.5);
    this->declare_parameter<double>("max_angular_vel", 4.0);
    this->declare_parameter<double>("min_linear_vel", 1.0);
    this->declare_parameter<std::string>("fire_control_source", "hybrid");
    this->declare_parameter<double>("input_priority_timeout", 0.3);
    this->declare_parameter<int>("deadzone", 2);
    this->declare_parameter<double>("gimbal_yaw_zero_offset", 0.0);

    max_linear_accel_  = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    max_linear_vel_    = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_   = this->get_parameter("max_angular_vel").as_double();
    min_linear_vel_    = this->get_parameter("min_linear_vel").as_double();
    const std::string fire_control_source =
      this->get_parameter("fire_control_source").as_string();
    input_priority_timeout_ = this->get_parameter("input_priority_timeout").as_double();
    deadzone_          = this->get_parameter("deadzone").as_int();
    gimbal_yaw_zero_offset_ =
      this->get_parameter("gimbal_yaw_zero_offset").as_double();
    fire_control_source_ = parseFireControlSource(fire_control_source);

    /* -------- 订阅话题 -------- */
    sub_channels_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/channels", 10,
      std::bind(&ChassisControlNode::channelsCallback, this, std::placeholders::_1));

    sub_keyboard_ = this->create_subscription<std_msgs::msg::UInt16>(
      "/vt_remote/keyboard", 10,
      std::bind(&ChassisControlNode::keyboardCallback, this, std::placeholders::_1));

    sub_mouse_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse", 10,
      std::bind(&ChassisControlNode::mouseCallback, this, std::placeholders::_1));

    sub_switches_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/switches", 10,
      std::bind(&ChassisControlNode::switchesCallback, this, std::placeholders::_1));

    sub_gimbal_yaw_multi_turn_ = this->create_subscription<std_msgs::msg::Float32>(
      "motor1/multi_turn_position", 10,
      std::bind(&ChassisControlNode::gimbalYawMultiTurnCallback, this, std::placeholders::_1));

    sub_key_toggles_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/key_toggles", 10,
      std::bind(&ChassisControlNode::keyTogglesCallback, this, std::placeholders::_1));

    /* -------- 发布者 -------- */
    pub_command_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/chassis/command", 10);

    /* -------- 时间戳初始化 -------- */
    last_channel_time_ = this->now();

    /* -------- 200Hz 控制定时器 (5ms) -------- */
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(5000),
      std::bind(&ChassisControlNode::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "ChassisControlNode 已启动 | 控制频率: 200Hz");
    RCLCPP_INFO(this->get_logger(),
                "限制参数: 线加速度=%.2f 角加速度=%.2f | 线速度=%.2f~%.2f 角速度=%.2f | 死区: %d",
                max_linear_accel_, max_angular_accel_, min_linear_vel_, max_linear_vel_, max_angular_vel_, deadzone_);
  }

private:
  /**
   * @brief 根据 /vt_remote/switches 的 mode 获取当前生效输入源
   * @return 生效输入源
   */
  FireControlSource getActiveFireControlSource() const
  {
    if (mode_switch_valid_) {
      if (mode_switch_ == 0) {
        return FireControlSource::MOUSE;
      }
      if (mode_switch_ == 1) {
        return FireControlSource::REMOTE;
      }
    }
    return fire_control_source_;
  }

  FireControlSource parseFireControlSource(const std::string & source) const
  {
    if (source == "mouse") return FireControlSource::MOUSE;
    if (source == "remote") return FireControlSource::REMOTE;
    if (source == "hybrid") return FireControlSource::HYBRID;
    RCLCPP_WARN(this->get_logger(), "未知 fire_control_source='%s'，回退为 hybrid", source.c_str());
    return FireControlSource::HYBRID;
  }

  void channelsCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (getActiveFireControlSource() == FireControlSource::MOUSE) return;
    if (msg->data.size() < 5) return;
    if (keyboard_wasd_active_) return;

    const float vx = mapChannelToVelocity(msg->data[2], max_linear_vel_);
    const float vy = -mapChannelToVelocity(msg->data[3], max_linear_vel_);
    const float vw = mapChannelToVelocity(msg->data[4], max_angular_vel_);

    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      target_vx_ = vx;
      target_vy_ = vy;
      target_vw_ = vw;
    }

    channel_received_ = true;
    last_channel_time_ = this->now();
  }

  void keyboardCallback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
    if (getActiveFireControlSource() == FireControlSource::REMOTE) return;

    const uint16_t kb = msg->data;
    const int w = (kb & KB_W) ? 1 : 0;
    const int s = (kb & KB_S) ? 1 : 0;
    const int a = (kb & KB_A) ? 1 : 0;
    const int d = (kb & KB_D) ? 1 : 0;
    const int q = (kb & KB_Q) ? 1 : 0;
    const int e = (kb & KB_E) ? 1 : 0;

    float vx = static_cast<float>(w - s);
    float vy = static_cast<float>(a - d);
    const float vw = static_cast<float>(e - q);
    const bool active = (vx != 0.0f) || (vy != 0.0f) || (vw != 0.0f);

    if (active && (vx != 0.0f) && (vy != 0.0f)) {
      const float inv_sqrt2 = 0.70710678f;
      vx *= inv_sqrt2;
      vy *= inv_sqrt2;
    }

    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      keyboard_wasd_active_ = active;
      target_vx_ = vx * static_cast<float>(max_linear_vel_);
      target_vy_ = vy * static_cast<float>(max_linear_vel_);
      target_vw_ = vw * static_cast<float>(max_angular_vel_);
    }

    channel_received_ = true;
    last_channel_time_ = this->now();
  }

  void mouseCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    const FireControlSource active_source = getActiveFireControlSource();
    if (active_source == FireControlSource::REMOTE) return;
    if (msg->data.size() < 6) return;

    const bool left_pressed = (msg->data[3] != 0);
    const bool right_pressed = (msg->data[4] != 0);
    const auto now = this->now();
    const bool state_changed =
      !mouse_button_state_initialized_ ||
      (left_pressed != prev_mouse_left_pressed_) ||
      (right_pressed != prev_mouse_right_pressed_);
    if (state_changed) {
      last_mouse_right_toggle_time_ = now;
      mouse_right_toggle_time_valid_ = true;
    }
    mouse_button_state_initialized_ = true;
    prev_mouse_left_pressed_ = left_pressed;
    prev_mouse_right_pressed_ = right_pressed;

    if (!left_pressed) {
      is_booster_high_speed_ = false;
      right_hold_start_valid_ = false;
      right_hold_active_1200_ = false;
      {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        feed_rpm_ = 0.0f;
      }
      return;
    }

    is_booster_high_speed_ = true;

    if (!right_pressed) {
      right_hold_start_valid_ = false;
      right_hold_active_1200_ = false;
      std::lock_guard<std::mutex> lock(vel_mutex_);
      feed_rpm_ = -3200.0f;
      return;
    }

    if (!right_hold_start_valid_) {
      right_hold_start_time_ = now;
      right_hold_start_valid_ = true;
      right_hold_active_1200_ = false;
    } else if ((now - right_hold_start_time_).seconds() >= 0.2) {
      right_hold_active_1200_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      feed_rpm_ = right_hold_active_1200_ ? 1200.0f : -3200.0f;
    }
  }

  void keyTogglesCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    const FireControlSource active_source = getActiveFireControlSource();
    if (active_source == FireControlSource::MOUSE) return;
    if (msg->data.size() < 4) return;

    const bool fn_left_toggle = (msg->data[1] != 0);
    const bool fn_right_toggle = (msg->data[2] != 0);

    const bool mouse_override_active =
      (active_source == FireControlSource::HYBRID) &&
      mouse_right_toggle_time_valid_ &&
      (this->now() - last_mouse_right_toggle_time_).seconds() < input_priority_timeout_;

    if (mouse_override_active) return;

    if (!fn_right_toggle_initialized_) {
      prev_fn_right_toggle_ = fn_right_toggle;
      fn_right_toggle_initialized_ = true;
    } else if (fn_right_toggle != prev_fn_right_toggle_) {
      is_booster_high_speed_ = fn_right_toggle;
      if (!is_booster_high_speed_) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        feed_rpm_ = 0.0f;
      } else {
        fn_left_toggle_initialized_ = true;
        prev_fn_left_toggle_ = fn_left_toggle;
      }
    }
    prev_fn_right_toggle_ = fn_right_toggle;
    if (!fn_left_toggle_initialized_) {
      prev_fn_left_toggle_ = fn_left_toggle;
      fn_left_toggle_initialized_ = true;
      return;
    }

    if (is_booster_high_speed_ && fn_left_toggle != prev_fn_left_toggle_) {
      const float new_feed = fn_left_toggle ? -3200.0f : 1200.0f;
      {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        feed_rpm_ = new_feed;
      }
    }
    prev_fn_left_toggle_ = fn_left_toggle;
  }

  /**
   * @brief 模式开关回调，mode=0 使用 mouse，mode=1 使用 remote
   * @param msg /vt_remote/switches 消息
   */
  void switchesCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }

    const int16_t new_mode = msg->data[0];
    if (!mode_switch_valid_ || mode_switch_ != new_mode) {
      mode_switch_ = new_mode;
      mode_switch_valid_ = true;

      if (mode_switch_ == 1) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        keyboard_wasd_active_ = false;
      }

      const FireControlSource active_source = getActiveFireControlSource();
      const char *source_str = "hybrid";
      if (active_source == FireControlSource::MOUSE) {
        source_str = "mouse";
      } else if (active_source == FireControlSource::REMOTE) {
        source_str = "remote";
      }
      RCLCPP_INFO(this->get_logger(),
                  "mode 已切换为 %d，当前输入源=%s",
                  mode_switch_, source_str);
    }
  }

  void gimbalYawMultiTurnCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    gimbal_yaw_offset_rad_ =
      -static_cast<double>(msg->data) / GIMBAL_YAW_REDUCTION_RATIO -
      gimbal_yaw_zero_offset_;
  }

  void controlTimerCallback()
  {
    float vx, vy, vw, feed;
    double gimbal_yaw_offset_rad;
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      if (!channel_received_ || (this->now() - last_channel_time_).seconds() > 0.5) {
        vx = vy = vw = feed = 0.0f;
      } else {
        vx   = target_vx_;
        vy   = target_vy_;
        vw   = target_vw_;
        feed = feed_rpm_;
      }
      gimbal_yaw_offset_rad = gimbal_yaw_offset_rad_;
    }

    const float linear_mag = std::sqrt(vx * vx + vy * vy);
    if (linear_mag > 1e-6f && linear_mag < static_cast<float>(min_linear_vel_)) {
      const float scale = static_cast<float>(min_linear_vel_) / linear_mag;
      vx *= scale;
      vy *= scale;
    }

    const float cos_theta = static_cast<float>(std::cos(gimbal_yaw_offset_rad));
    const float sin_theta = static_cast<float>(std::sin(gimbal_yaw_offset_rad));
    const float chassis_vx = cos_theta * vx - sin_theta * vy;
    const float chassis_vy = sin_theta * vx + cos_theta * vy;

    std_msgs::msg::Float32MultiArray cmd_msg;
    cmd_msg.data.resize(8);
    cmd_msg.data[0] = static_cast<float>(max_linear_accel_);
    cmd_msg.data[1] = static_cast<float>(max_angular_accel_);
    cmd_msg.data[2] = static_cast<float>(max_linear_vel_);
    cmd_msg.data[3] = static_cast<float>(max_angular_vel_);
    cmd_msg.data[4] = chassis_vx;
    cmd_msg.data[5] = chassis_vy;
    cmd_msg.data[6] = vw;
    cmd_msg.data[7] = feed;

    pub_command_->publish(cmd_msg);
  }

  float mapChannelToVelocity(int16_t raw, double max_vel) const
  {
    const int16_t offset = static_cast<int16_t>(raw - CH_CENTER);
    if (std::abs(static_cast<int>(offset)) <= deadzone_) return 0.0f;

    const double effective_range = CH_HALF_RANGE - static_cast<double>(deadzone_);
    double normalized;
    if (offset > 0) {
      normalized = static_cast<double>(offset - deadzone_) / effective_range;
    } else {
      normalized = static_cast<double>(offset + deadzone_) / effective_range;
    }

    normalized = std::clamp(normalized, -1.0, 1.0);
    return static_cast<float>(normalized * max_vel);
  }

  double max_linear_accel_{};
  double max_angular_accel_{};
  double max_linear_vel_{};
  double max_angular_vel_{};
  double min_linear_vel_{};
  int deadzone_{};
  double gimbal_yaw_zero_offset_{};
  FireControlSource fire_control_source_{FireControlSource::HYBRID};
  double input_priority_timeout_{};

  std::mutex vel_mutex_;
  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_vw_ = 0.0f;
  float feed_rpm_ = 0.0f;
  double gimbal_yaw_offset_rad_ = 0.0;
  bool channel_received_ = false;

  bool is_booster_high_speed_ = false;
  bool fn_left_toggle_initialized_ = false;
  bool prev_fn_left_toggle_ = false;
  bool fn_right_toggle_initialized_ = false;
  bool prev_fn_right_toggle_ = false;
  rclcpp::Time right_hold_start_time_;
  bool right_hold_start_valid_ = false;
  bool right_hold_active_1200_ = false;
  rclcpp::Time last_mouse_right_toggle_time_;
  bool mouse_right_toggle_time_valid_ = false;
  bool mouse_button_state_initialized_ = false;
  bool prev_mouse_left_pressed_ = false;
  bool prev_mouse_right_pressed_ = false;
  bool keyboard_wasd_active_ = false;
  int16_t mode_switch_ = 1;
  bool mode_switch_valid_ = false;

  rclcpp::Time last_channel_time_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_channels_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_keyboard_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_mouse_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_switches_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_gimbal_yaw_multi_turn_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_key_toggles_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_command_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisControlNode>());
  rclcpp::shutdown();
  return 0;
}
