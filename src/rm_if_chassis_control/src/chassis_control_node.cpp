/**
 * @file chassis_control_node.cpp
 * @brief 底盘运动控制节点
 *
 * @details
 * 订阅图传遥控器通道数据 (/vt_remote/channels)，将摇杆量映射为底盘三轴速度，
 * 通过 RS485 串口以 200Hz 频率发送控制帧到 STM32 MCU，
 * 并异步接收里程计/速度反馈帧。
 *
 * 通道映射：
 *   - ch2  (index 2) → vx  x 方向线速度
 *   - ch3  (index 3) → vy  y 方向线速度
 *   - wheel(index 4) → vw  角速度（逆时针为正）
 *   - 通道值域 364 ~ 1684，中值 1024
 *
 * 串口协议（小端序）：
 *   - 控制帧 PC→MCU:
 *       [0xAA] + max_linear_accel(f32) + max_angular_accel(f32)
 *              + max_linear_vel(f32)   + max_angular_vel(f32)
 *              + vx(f32) + vy(f32) + vw(f32) + feed_rpm(f32) + crc16(u16) = 35B
 *   - 反馈帧 MCU→PC: [0x55] + x(f32) + y(f32) + θ(f32) + vx(f32) + vy(f32)
 *                      + vw(f32) + wheel1_v(f32) + wheel2_v(f32)
 *                      + wheel3_v(f32) + wheel4_v(f32) + feed_rpm(f32) + gyro_z(f32)
 *                      + crc16(u16) + [0xAA] = 52B
 *
 * 供弹速度控制逻辑：
 *   - 订阅 /vt_remote/key_toggles，使用 fn_right 切换状态跟踪 booster 高/低速模式
 *   - 低速模式: 供弹速度固定为 0，fn_left 切换变化忽略
 *   - 高速模式: fn_left 切换状态变化时在 -3200 RPM 与 1200 RPM 之间切换
 *   - 从高速切回低速时，供弹速度自动归零
 *
 * 订阅话题：
 *   - /vt_remote/channels  (std_msgs/Int16MultiArray) : [ch0, ch1, ch2, ch3, wheel]
 *   - /vt_remote/key_toggles  (std_msgs/Int16MultiArray) : [pause, fn_left, fn_right, trigger]
 *
 * 发布话题：
 *   - /chassis/feedback    (std_msgs/Float32MultiArray):
 *     [x, y, θ, vx, vy, vw, wheel1_v, wheel2_v, wheel3_v, wheel4_v, feed_rpm, gyro_z]
 *
 * 参数：
 *   - serial_port  (string) : 串口设备路径，默认 "/dev/chassis_usb"
 *   - baud_rate    (int)    : 波特率，默认 460800
 *   - max_linear_accel  (double) : 最大线加速度 (m/s²)，默认 3.0
 *   - max_angular_accel (double) : 最大角加速度 (rad/s²)，默认 6.0
 *   - max_linear_vel    (double) : 最大线速度 (m/s)，默认 4.0
 *   - max_angular_vel   (double) : 最大角速度 (rad/s)，默认 10.0
 *   - min_linear_vel    (double) : 最小线速度 (m/s)，默认 0.5；当指令速度非零且小于此值时按此速度运动
 *   - deadzone     (int)    : 摇杆死区（原始值），默认 20
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>

/* ========================================================================= */
/*  协议常量                                                                  */
/* ========================================================================= */

/** @brief 控制帧帧头 */
static constexpr uint8_t CTRL_FRAME_HEADER = 0xAA;
/** @brief 控制帧 CRC 起始偏移 */
static constexpr size_t CTRL_FRAME_CRC_OFFSET = 33;
/** @brief 控制帧 CRC 计算长度 (从帧头到 feed_rpm，含首不含 CRC) */
static constexpr size_t CTRL_FRAME_CRC_LEN = 33;
/** @brief 控制帧长度 (1B 帧头 + 8×float32 + 2B CRC16) */
static constexpr size_t CTRL_FRAME_SIZE = 35;

/** @brief 反馈帧帧头 */
static constexpr uint8_t FB_FRAME_HEADER = 0x55;
/** @brief 反馈帧帧尾 */
static constexpr uint8_t FB_FRAME_TAIL = 0xAA;
/** @brief 反馈帧 CRC 字段偏移 */
static constexpr size_t FB_FRAME_CRC_OFFSET = 49;
/** @brief 反馈帧 CRC 计算长度 (从帧头到 gyro_z，含首不含 CRC/tail) */
static constexpr size_t FB_FRAME_CRC_LEN = 49;
/** @brief 反馈帧长度 (1B 帧头 + 12×float32 + 2B CRC16 + 1B 帧尾) */
static constexpr size_t FB_FRAME_SIZE = 52;
/** @brief 反馈帧 float 数量 */
static constexpr size_t FB_FLOAT_COUNT = 12;

/**
 * @brief UART控制帧结构体
 * @details 帧格式:
 * [帧头1B][最大线加速度4B][最大角加速度4B][最大线速度4B][最大角速度4B]
 * [vx4B][vy4B][vw4B][feed_rpm4B][crc16 2B]
 */
typedef struct
{
  uint8_t header;            ///< 帧头 (0xAA)
  float max_linear_accel;    ///< 最大线加速度 (m/s²)
  float max_angular_accel;   ///< 最大角加速度 (rad/s²)
  float max_linear_vel;      ///< 最大线速度 (m/s)
  float max_angular_vel;     ///< 最大角速度 (rad/s)
  float vel_x;               ///< x方向线速度 (m/s)
  float vel_y;               ///< y方向线速度 (m/s)
  float vel_angular;         ///< 角速度 (rad/s) 逆时针为正
  float feed_motor_rpm;      ///< 供弹电机转速 (RPM)
  uint16_t crc;              ///< CRC16校验
} __attribute__((packed)) Control_Frame_t;

/**
 * @brief 反馈帧结构体
 * @details 帧格式:
 * [帧头1B][x4B][y4B][theta4B][vx4B][vy4B][vw4B][wheel1_v4B][wheel2_v4B]
 * [wheel3_v4B][wheel4_v4B][feed_rpm4B][gyro_z4B][crc16 2B][帧尾1B]
 */
typedef struct
{
  uint8_t header;            ///< 帧头 (0x55)
  float x;                   ///< 位置x (m)
  float y;                   ///< 位置y (m)
  float theta;               ///< 姿态角 (rad)
  float vel_x;               ///< 实际速度x (m/s)
  float vel_y;               ///< 实际速度y (m/s)
  float vel_angular;         ///< 实际角速度 (rad/s)
  float wheel_vel_1;         ///< 1号轮实际线速度 (m/s)
  float wheel_vel_2;         ///< 2号轮实际线速度 (m/s)
  float wheel_vel_3;         ///< 3号轮实际线速度 (m/s)
  float wheel_vel_4;         ///< 4号轮实际线速度 (m/s)
  float feed_rpm;            ///< 供弹电机实际转速 (RPM)
  float gyro_z;              ///< BMI088 z轴角速度 (rad/s)
  uint16_t crc;              ///< CRC16校验
  uint8_t tail;              ///< 帧尾 (0xAA)
} __attribute__((packed)) Feedback_Frame_t;

static_assert(sizeof(Control_Frame_t) == CTRL_FRAME_SIZE,
              "Control_Frame_t size mismatch with CTRL_FRAME_SIZE");
static_assert(sizeof(Feedback_Frame_t) == FB_FRAME_SIZE,
              "Feedback_Frame_t size mismatch with FB_FRAME_SIZE");

/**
 * @brief CRC16-CCITT 计算
 * @param data 数据起始地址
 * @param length 数据长度（字节）
 * @return CRC16 校验值
 */
static uint16_t CRC16_CCITT(const uint8_t * data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

/**
 * @brief 写入 uint16 小端值
 * @param dst 目标地址（至少 2 字节）
 * @param value 待写入值
 */
static inline void writeU16LE(uint8_t * dst, uint16_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

/**
 * @brief 读取 uint16 小端值
 * @param src 源地址（至少 2 字节）
 * @return 读取出的 uint16 值
 */
static inline uint16_t readU16LE(const uint8_t * src)
{
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

/* ========================================================================= */
/*  摇杆通道常量                                                              */
/* ========================================================================= */

/** @brief 通道中值 */
static constexpr int16_t CH_CENTER = 1024;
/** @brief 通道最小值 */
static constexpr int16_t CH_MIN = 364;
/** @brief 通道最大值 */
static constexpr int16_t CH_MAX = 1684;
/** @brief 通道半程范围 */
static constexpr double CH_HALF_RANGE = static_cast<double>(CH_MAX - CH_CENTER);  // 660

/**
 * @brief 键盘位定义（/vt_remote/keyboard）
 */
enum KeyboardBit : uint16_t
{
  KB_W = (1u << 0),
  KB_S = (1u << 1),
  KB_A = (1u << 2),
  KB_D = (1u << 3),
  KB_Q = (1u << 6),
  KB_E = (1u << 7)
};

/**
 * @brief 发射/供弹控制输入源
 */
enum class FireControlSource
{
  MOUSE = 0,   ///< 仅鼠标
  REMOTE = 1,  ///< 仅遥控器按键
  HYBRID = 2   ///< 混合模式（鼠标优先窗口）
};

/**
 * @class ChassisControlNode
 * @brief 底盘运动控制 ROS2 节点
 *
 * 负责：
 * 1. 订阅遥控器通道数据，映射为底盘速度
 * 2. 以 200Hz 频率通过串口发送控制帧
 * 3. 异步接收并发布 MCU 反馈数据
 */
class ChassisControlNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化参数、串口、订阅者、发布者、定时器
   */
  ChassisControlNode()
  : Node("chassis_control_node"), serial_fd_(-1)
  {
    /* -------- 声明 & 读取参数 -------- */
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 460800);
    this->declare_parameter<double>("max_linear_accel", 4.0);
    this->declare_parameter<double>("max_angular_accel", 8.0);
    this->declare_parameter<double>("max_linear_vel", 1.5);
    this->declare_parameter<double>("max_angular_vel", 4.0);
    this->declare_parameter<double>("min_linear_vel", 1.0);
    this->declare_parameter<std::string>("fire_control_source", "hybrid");
    this->declare_parameter<double>("input_priority_timeout", 0.3);
    this->declare_parameter<int>("deadzone", 2);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_   = this->get_parameter("baud_rate").as_int();
    max_linear_accel_  = this->get_parameter("max_linear_accel").as_double();
    max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();
    max_linear_vel_    = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_   = this->get_parameter("max_angular_vel").as_double();
    min_linear_vel_    = this->get_parameter("min_linear_vel").as_double();
    const std::string fire_control_source =
      this->get_parameter("fire_control_source").as_string();
    input_priority_timeout_ = this->get_parameter("input_priority_timeout").as_double();
    deadzone_          = this->get_parameter("deadzone").as_int();
    fire_control_source_ = parseFireControlSource(fire_control_source);

    /* -------- 订阅遥控器通道 -------- */
    sub_channels_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/channels", 10,
      std::bind(&ChassisControlNode::channelsCallback, this, std::placeholders::_1));

    /* -------- 订阅键盘按键（WASD 平移 + QE 旋转）-------- */
    sub_keyboard_ = this->create_subscription<std_msgs::msg::UInt16>(
      "/vt_remote/keyboard", 10,
      std::bind(&ChassisControlNode::keyboardCallback, this, std::placeholders::_1));

    /* -------- 订阅鼠标数据（发射控制）-------- */
    sub_mouse_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse", 10,
      std::bind(&ChassisControlNode::mouseCallback, this, std::placeholders::_1));

    /* -------- 订阅遥控器按键切换状态 (供弹控制) -------- */
    sub_key_toggles_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/key_toggles", 10,
      std::bind(&ChassisControlNode::keyTogglesCallback, this, std::placeholders::_1));

    /* -------- 反馈发布者 -------- */
    pub_feedback_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/chassis/feedback", 10);
    pub_gyro_z_ = this->create_publisher<std_msgs::msg::Float32>(
      "/chassis/gyro_z", 10);

    /* -------- 打开串口 -------- */
    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s，节点将尝试重连...",
                   serial_port_.c_str());
    }

    /* -------- 预分配 -------- */
    ctrl_frame_[0] = CTRL_FRAME_HEADER;
    feedback_msg_.data.resize(FB_FLOAT_COUNT, 0.0f);
    rx_buffer_.reserve(256);

    /* -------- 时间戳初始化 -------- */
    last_channel_time_  = this->now();
    last_feedback_time_ = this->now();

    /* -------- 200Hz 控制定时器 (5ms) -------- */
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(5000),
      std::bind(&ChassisControlNode::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "ChassisControlNode 已启动 | 串口: %s | 波特率: %d | 控制频率: 200Hz",
                serial_port_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(),
                "限制参数: 线加速度=%.2f 角加速度=%.2f | 线速度=%.2f~%.2f 角速度=%.2f | 死区: %d",
                max_linear_accel_, max_angular_accel_, min_linear_vel_, max_linear_vel_, max_angular_vel_, deadzone_);
    RCLCPP_INFO(this->get_logger(),
                "发射/供弹控制输入源: %s",
                fireControlSourceToString(fire_control_source_));
  }

  /**
   * @brief 析构函数：发送停止指令后关闭串口
   */
  ~ChassisControlNode() override
  {
    sendStopCommand();
    closeSerial();
  }

private:
  // =========================================================================
  //  回调函数
  // =========================================================================

  /**
   * @brief 解析发射/供弹控制输入源参数
   * @param source 参数字符串
   * @return 输入源枚举
   */
  FireControlSource parseFireControlSource(const std::string & source) const
  {
    if (source == "mouse") {
      return FireControlSource::MOUSE;
    }
    if (source == "remote") {
      return FireControlSource::REMOTE;
    }
    if (source == "hybrid") {
      return FireControlSource::HYBRID;
    }
    RCLCPP_WARN(this->get_logger(),
                "未知 fire_control_source='%s'，回退为 hybrid",
                source.c_str());
    return FireControlSource::HYBRID;
  }

  /**
   * @brief 输入源枚举转字符串
   * @param source 输入源枚举
   * @return 输入源字符串
   */
  const char * fireControlSourceToString(FireControlSource source) const
  {
    switch (source) {
      case FireControlSource::MOUSE:
        return "mouse";
      case FireControlSource::REMOTE:
        return "remote";
      case FireControlSource::HYBRID:
      default:
        return "hybrid";
    }
  }

  /**
   * @brief 遥控器通道数据回调（轻量，仅更新缓存速度值）
   * @param msg 通道数据 [ch0, ch1, ch2, ch3, wheel]
   */
  void channelsCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "通道数据不完整: 期望 5 个值，收到 %zu", msg->data.size());
      return;
    }

    /* WASD 按键控制生效时，忽略通道映射 */
    if (keyboard_wasd_active_) {
      return;
    }

    /* ch2 → vx, ch3 → vy, wheel(index 4) → vw */
    const float vx = mapChannelToVelocity(msg->data[2], max_linear_vel_);
    const float vy = -mapChannelToVelocity(msg->data[3], max_linear_vel_);
    const float vw = mapChannelToVelocity(msg->data[4], max_angular_vel_);

    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      target_vx_ = vx;
      target_vy_ = vy;
      target_vw_ = vw;
      /* feed_rpm_ 由其他接口控制，此处不修改 */
    }

    channel_received_ = true;
    last_channel_time_ = this->now();
  }

  /**
   * @brief 键盘数据回调（WASD 控制平移，QE 控制旋转）
   * @param msg keyboard 位图
   */
  void keyboardCallback(const std_msgs::msg::UInt16::SharedPtr msg)
  {
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
      if (active) {
        target_vx_ = vx * static_cast<float>(max_linear_vel_);
        target_vy_ = vy * static_cast<float>(max_linear_vel_);
        target_vw_ = vw * static_cast<float>(max_angular_vel_);
      }
    }

    channel_received_ = true;
    last_channel_time_ = this->now();
  }

  /**
   * @brief 鼠标数据回调（左键状态控制发射，右键长按切换供弹档位）
   * @param msg mouse 数据 [x, y, z, left, right, middle]
   */
  void mouseCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (fire_control_source_ == FireControlSource::REMOTE) {
      return;
    }

    if (msg->data.size() < 6) {
      return;
    }

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

  /**
   * @brief 遥控器按键切换状态回调（供弹速度控制）
   * @param msg key_toggles 数据 [pause, fn_left, fn_right, trigger]
   */
  void keyTogglesCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (fire_control_source_ == FireControlSource::MOUSE) {
      return;
    }

    if (msg->data.size() < 4) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "key_toggles 数据不完整: 期望 4 个值，收到 %zu", msg->data.size());
      return;
    }

    const bool fn_left_toggle = (msg->data[1] != 0);   /* index 1: fn_left */
    const bool fn_right_toggle = (msg->data[2] != 0);  /* index 2: fn_right */

    const bool mouse_override_active =
      (fire_control_source_ == FireControlSource::HYBRID) &&
      mouse_right_toggle_time_valid_ &&
      (this->now() - last_mouse_right_toggle_time_).seconds() < input_priority_timeout_;

    if (mouse_override_active) {
      return;
    }

    if (!fn_right_toggle_initialized_) {
      prev_fn_right_toggle_ = fn_right_toggle;
      fn_right_toggle_initialized_ = true;
    } else if (fn_right_toggle != prev_fn_right_toggle_) {
      is_booster_high_speed_ = fn_right_toggle;
      if (!is_booster_high_speed_) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        feed_rpm_ = 0.0f;
        RCLCPP_INFO(this->get_logger(),
                    "fn_right 切换 -> Booster 低速模式, 供弹速度归零 (feed_rpm = 0)");
      } else {
        fn_left_toggle_initialized_ = true;
        prev_fn_left_toggle_ = fn_left_toggle;
        RCLCPP_INFO(this->get_logger(),
                    "fn_right 切换 -> Booster 高速模式, 供弹速度待 fn_left 切换");
      }
    }
    prev_fn_right_toggle_ = fn_right_toggle;
    if (!fn_left_toggle_initialized_) {
      prev_fn_left_toggle_ = fn_left_toggle;
      fn_left_toggle_initialized_ = true;
      return;
    }

    /* 仅在高速模式下响应 fn_left 切换状态变化 */
    if (is_booster_high_speed_ && fn_left_toggle != prev_fn_left_toggle_) {
      const float new_feed = fn_left_toggle ? -3200.0f : 1200.0f;
      {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        feed_rpm_ = new_feed;
      }
      RCLCPP_INFO(this->get_logger(),
                  "fn_left 切换 -> 供弹速度: %.0f RPM", new_feed);
    }
    prev_fn_left_toggle_ = fn_left_toggle;
  }

  /**
   * @brief 200Hz 控制定时器回调：发送控制帧 + 读取反馈
   */
  void controlTimerCallback()
  {
    /* 串口未打开时尝试重连 */
    if (serial_fd_ < 0) {
      reconnect_counter_++;
      if (reconnect_counter_ >= 200) {          /* 约每 1 秒尝试 */
        reconnect_counter_ = 0;
        RCLCPP_WARN(this->get_logger(), "尝试重新打开串口 %s ...",
                    serial_port_.c_str());
        openSerial();
      }
      return;
    }

    /* 获取目标速度（加锁） */
    float vx, vy, vw, feed;
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);

      /* 安全保护：500ms 未收到遥控数据 → 零速 */
      if (!channel_received_ ||
          (this->now() - last_channel_time_).seconds() > 0.5) {
        vx = vy = vw = feed = 0.0f;
      } else {
        vx   = target_vx_;
        vy   = target_vy_;
        vw   = target_vw_;
        feed = feed_rpm_;
      }
    }

    /* 最小线速度：当指令速度非零且小于 min_linear_vel 时，按 min_linear_vel 运动（保持方向） */
    const float linear_mag = std::sqrt(vx * vx + vy * vy);
    if (linear_mag > 1e-6f && linear_mag < static_cast<float>(min_linear_vel_)) {
      const float scale = static_cast<float>(min_linear_vel_) / linear_mag;
      vx *= scale;
      vy *= scale;
    }

    /* 发送控制帧 */
    sendControlFrame(vx, vy, vw, feed);

    /* 非阻塞读取反馈 */
    readFeedback();

    /* 反馈超时检测 */
    checkFeedbackTimeout();
  }

  // =========================================================================
  //  通道映射
  // =========================================================================

  /**
   * @brief 将原始通道值映射为速度，带死区处理
   * @param raw      原始通道值 (364 ~ 1684, 中值 1024)
   * @param max_vel  该轴最大速度
   * @return 映射后的速度值
   */
  float mapChannelToVelocity(int16_t raw, double max_vel) const
  {
    const int16_t offset = static_cast<int16_t>(raw - CH_CENTER);

    /* 死区内返回零 */
    if (std::abs(static_cast<int>(offset)) <= deadzone_) {
      return 0.0f;
    }

    /* 扣除死区后的有效范围 */
    const double effective_range = CH_HALF_RANGE - static_cast<double>(deadzone_);
    double normalized;
    if (offset > 0) {
      normalized = static_cast<double>(offset - deadzone_) / effective_range;
    } else {
      normalized = static_cast<double>(offset + deadzone_) / effective_range;
    }

    /* 限幅 [-1, 1] */
    normalized = std::clamp(normalized, -1.0, 1.0);

    return static_cast<float>(normalized * max_vel);
  }

  // =========================================================================
  //  串口协议
  // =========================================================================

  /**
   * @brief 发送控制帧到 MCU
   * @param vx       x 方向速度 (m/s)
   * @param vy       y 方向速度 (m/s)
   * @param vw       角速度 (rad/s)
   * @param feed_rpm 供弹电机转速 (RPM)
   */
  void sendControlFrame(float vx, float vy, float vw, float feed_rpm)
  {
    /* 帧头已在构造函数中预填充 */
    const float max_linear_accel_f = static_cast<float>(max_linear_accel_);
    const float max_angular_accel_f = static_cast<float>(max_angular_accel_);
    const float max_linear_vel_f = static_cast<float>(max_linear_vel_);
    const float max_angular_vel_f = static_cast<float>(max_angular_vel_);

    std::memcpy(&ctrl_frame_[1],  &max_linear_accel_f,  sizeof(float));
    std::memcpy(&ctrl_frame_[5],  &max_angular_accel_f, sizeof(float));
    std::memcpy(&ctrl_frame_[9],  &max_linear_vel_f,    sizeof(float));
    std::memcpy(&ctrl_frame_[13], &max_angular_vel_f,   sizeof(float));
    std::memcpy(&ctrl_frame_[17], &vx,                  sizeof(float));
    std::memcpy(&ctrl_frame_[21], &vy,                  sizeof(float));
    std::memcpy(&ctrl_frame_[25], &vw,                  sizeof(float));
    std::memcpy(&ctrl_frame_[29], &feed_rpm,            sizeof(float));
    const uint16_t crc = CRC16_CCITT(ctrl_frame_, static_cast<uint16_t>(CTRL_FRAME_CRC_LEN));
    writeU16LE(&ctrl_frame_[CTRL_FRAME_CRC_OFFSET], crc);

    const ssize_t written = ::write(serial_fd_, ctrl_frame_, CTRL_FRAME_SIZE);
    if (written < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(this->get_logger(), "串口写入错误: %s", strerror(errno));
        closeSerial();
      }
    }
  }

  /**
   * @brief 非阻塞读取并解析反馈帧，解析成功后发布到话题
   */
  void readFeedback()
  {
    /* 一次性读取所有可用数据 */
    uint8_t tmp[256];
    const ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
    if (n <= 0) {
      return;
    }

    /* 追加到接收缓冲区 */
    rx_buffer_.insert(rx_buffer_.end(), tmp, tmp + n);

    /* 解析所有完整的反馈帧 */
    while (rx_buffer_.size() >= FB_FRAME_SIZE) {
      /* 查找帧头 0x55 并验证帧尾 */
      const size_t header_pos = findFeedbackHeader();
      if (header_pos == std::string::npos) {
        /* 未找到有效帧头，保留最后可能的部分数据 */
        if (rx_buffer_.size() > FB_FRAME_SIZE - 1) {
          rx_buffer_.erase(
            rx_buffer_.begin(),
            rx_buffer_.end() - static_cast<long>(FB_FRAME_SIZE - 1));
        }
        break;
      }

      /* 丢弃帧头之前的无效数据 */
      if (header_pos > 0) {
        rx_buffer_.erase(rx_buffer_.begin(),
                         rx_buffer_.begin() + static_cast<long>(header_pos));
      }

      /* 数据不足一帧则等待下次 */
      if (rx_buffer_.size() < FB_FRAME_SIZE) {
        break;
      }

      /* 校验帧尾和 CRC */
      if (rx_buffer_[FB_FRAME_SIZE - 1] == FB_FRAME_TAIL) {
        const uint16_t rx_crc = readU16LE(&rx_buffer_[FB_FRAME_CRC_OFFSET]);
        const uint16_t calc_crc =
          CRC16_CCITT(rx_buffer_.data(), static_cast<uint16_t>(FB_FRAME_CRC_LEN));
        if (rx_crc != calc_crc) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "反馈帧 CRC 校验失败: rx=0x%04X calc=0x%04X",
                               rx_crc, calc_crc);
          rx_buffer_.erase(rx_buffer_.begin(),
                           rx_buffer_.begin() + static_cast<long>(FB_FRAME_SIZE));
          continue;
        }

        /* 解析 12 个 float32: x, y, theta, vx, vy, vw, wheel1_v, wheel2_v, wheel3_v, wheel4_v, feed_rpm, gyro_z */
        float values[FB_FLOAT_COUNT];
        std::memcpy(values, &rx_buffer_[1], FB_FLOAT_COUNT * sizeof(float));

        for (size_t i = 0; i < FB_FLOAT_COUNT; ++i) {
          feedback_msg_.data[i] = values[i];
        }
        pub_feedback_->publish(feedback_msg_);
        gyro_z_msg_.data = values[11];
        pub_gyro_z_->publish(gyro_z_msg_);

        last_feedback_time_ = this->now();
        if (feedback_timeout_) {
          RCLCPP_INFO(this->get_logger(), "反馈帧恢复接收");
          feedback_timeout_ = false;
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "反馈: Pos(%.3f, %.3f, %.3f) Vel(%.3f, %.3f, %.3f) W(%.3f, %.3f, %.3f, %.3f) Feed(%.0f) GyroZ(%.3f)",
                     values[0], values[1], values[2],
                     values[3], values[4], values[5],
                     values[6], values[7], values[8], values[9], values[10], values[11]);
      }

      /* 移除已处理/无效的帧 */
      rx_buffer_.erase(rx_buffer_.begin(),
                       rx_buffer_.begin() + static_cast<long>(FB_FRAME_SIZE));
    }

    /* 防止缓冲区无限增长 */
    if (rx_buffer_.size() > 512) {
      rx_buffer_.erase(
        rx_buffer_.begin(),
        rx_buffer_.end() - static_cast<long>(FB_FRAME_SIZE));
    }
  }

  /**
   * @brief 在接收缓冲区中查找有效反馈帧头
   *
   * @details 同时验证帧头 0x55 和帧尾 0xAA 的位置关系，
   *          减少误帧头的影响。
   * @return 帧头起始位置，未找到返回 std::string::npos
   */
  size_t findFeedbackHeader() const
  {
    for (size_t i = 0; i + FB_FRAME_SIZE <= rx_buffer_.size(); ++i) {
      if (rx_buffer_[i] == FB_FRAME_HEADER &&
          rx_buffer_[i + FB_FRAME_SIZE - 1] == FB_FRAME_TAIL) {
        return i;
      }
    }
    return std::string::npos;
  }

  /**
   * @brief 反馈帧超时检测
   */
  void checkFeedbackTimeout()
  {
    if (!feedback_timeout_ &&
        (this->now() - last_feedback_time_).seconds() > 3.0) {
      RCLCPP_WARN(this->get_logger(), "MCU 反馈超时：3 秒内未收到有效反馈帧");
      feedback_timeout_ = true;
    }
  }

  /**
   * @brief 发送停止指令（连续 3 帧零速，确保 MCU 接收到）
   */
  void sendStopCommand()
  {
    if (serial_fd_ < 0) {
      return;
    }
    for (int i = 0; i < 3; ++i) {
      sendControlFrame(0.0f, 0.0f, 0.0f, 0.0f);
      usleep(5000);  /* 5ms 间隔 */
    }
    RCLCPP_INFO(this->get_logger(), "已发送停止指令");
  }

  // =========================================================================
  //  串口操作
  // =========================================================================

  /**
   * @brief 打开并配置串口 (8N1, 无流控, 非阻塞)
   * @return true 成功, false 失败
   */
  bool openSerial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s (%s)",
                   serial_port_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty{};

    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr 失败: %s", strerror(errno));
      closeSerial();
      return false;
    }

    /* 波特率 */
    const speed_t baud = baudToSpeed(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    /* 8N1: 8 数据位, 无校验, 1 停止位 */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    /* 无硬件流控 */
    tty.c_cflag &= ~CRTSCTS;

    /* 启用接收, 本地模式 */
    tty.c_cflag |= (CLOCAL | CREAD);

    /* 原始输入模式 */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* 禁用软件流控 & 特殊字符处理 */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                      INLCR | IGNCR | ICRNL);

    /* 原始输出模式 */
    tty.c_oflag &= ~OPOST;

    /* 完全非阻塞读取 */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    /* 清空缓冲区并应用配置 */
    tcflush(serial_fd_, TCIOFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr 失败: %s", strerror(errno));
      closeSerial();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "串口 %s 已打开 (波特率 %d)", serial_port_.c_str(), baud_rate_);
    return true;
  }

  /**
   * @brief 关闭串口
   */
  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  /**
   * @brief 将整数波特率转换为 termios speed_t 常量
   * @param baud 整数波特率
   * @return 对应的 speed_t 值
   */
  static speed_t baudToSpeed(int baud)
  {
    switch (baud) {
      case 9600:    return B9600;
      case 19200:   return B19200;
      case 38400:   return B38400;
      case 57600:   return B57600;
      case 115200:  return B115200;
      case 230400:  return B230400;
      case 460800:  return B460800;
      case 500000:  return B500000;
      case 576000:  return B576000;
      case 921600:  return B921600;
      case 1000000: return B1000000;
      default:      return B921600;
    }
  }

  // =========================================================================
  //  成员变量
  // =========================================================================

  /* ---- 参数 ---- */

  /** @brief 串口设备路径 */
  std::string serial_port_;
  /** @brief 波特率 */
  int baud_rate_{};
  /** @brief 最大线加速度 (m/s²) */
  double max_linear_accel_{};
  /** @brief 最大角加速度 (rad/s²) */
  double max_angular_accel_{};
  /** @brief 最大线速度 (m/s) */
  double max_linear_vel_{};
  /** @brief 最大角速度 (rad/s) */
  double max_angular_vel_{};
  /** @brief 最小线速度 (m/s)；当指令速度非零且小于此值时按此速度运动 */
  double min_linear_vel_{};
  /** @brief 摇杆死区（原始值单位） */
  int deadzone_{};
  /** @brief 发射/供弹控制输入源 */
  FireControlSource fire_control_source_{FireControlSource::HYBRID};
  /** @brief 键鼠优先窗口时长（秒） */
  double input_priority_timeout_{};

  /* ---- 串口 ---- */

  /** @brief 串口文件描述符 */
  int serial_fd_;
  /** @brief 接收缓冲区 */
  std::vector<uint8_t> rx_buffer_;
  /** @brief 重连计数器 */
  int reconnect_counter_ = 0;

  /* ---- 速度状态（受 vel_mutex_ 保护）---- */

  /** @brief 速度状态互斥锁 */
  std::mutex vel_mutex_;
  /** @brief 目标 x 速度 */
  float target_vx_ = 0.0f;
  /** @brief 目标 y 速度 */
  float target_vy_ = 0.0f;
  /** @brief 目标角速度 */
  float target_vw_ = 0.0f;
  /** @brief 供弹电机目标转速 */
  float feed_rpm_ = 0.0f;
  /** @brief 是否已收到过遥控数据 */
  bool channel_received_ = false;

  /* ---- 供弹速度控制状态 ---- */

  /** @brief Booster 是否处于高速模式 (由 fn_right 切换) */
  bool is_booster_high_speed_ = false;
  /** @brief fn_left 切换状态是否已初始化 */
  bool fn_left_toggle_initialized_ = false;
  /** @brief fn_left 上一次切换状态 */
  bool prev_fn_left_toggle_ = false;
  /** @brief fn_right 切换状态是否已初始化 */
  bool fn_right_toggle_initialized_ = false;
  /** @brief fn_right 上一次切换状态 */
  bool prev_fn_right_toggle_ = false;
  /** @brief 鼠标右键长按起始时间 */
  rclcpp::Time right_hold_start_time_;
  /** @brief 鼠标右键长按起始时间是否有效 */
  bool right_hold_start_valid_ = false;
  /** @brief 鼠标右键是否已激活1200档 */
  bool right_hold_active_1200_ = false;
  /** @brief 鼠标右键最近切换时间 */
  rclcpp::Time last_mouse_right_toggle_time_;
  /** @brief 鼠标右键切换时间是否有效 */
  bool mouse_right_toggle_time_valid_ = false;
  /** @brief 鼠标按键状态是否已初始化 */
  bool mouse_button_state_initialized_ = false;
  /** @brief 鼠标左键上次状态 */
  bool prev_mouse_left_pressed_ = false;
  /** @brief 鼠标右键上次状态 */
  bool prev_mouse_right_pressed_ = false;
  /** @brief WASD 键盘控制是否生效 */
  bool keyboard_wasd_active_ = false;

  /* ---- 时间戳 ---- */

  /** @brief 上次收到通道数据的时间 */
  rclcpp::Time last_channel_time_;
  /** @brief 上次收到反馈帧的时间 */
  rclcpp::Time last_feedback_time_;
  /** @brief 反馈是否超时 */
  bool feedback_timeout_ = false;

  /* ---- 预分配缓冲区 ---- */

  /** @brief 预分配的控制帧缓冲区 */
  uint8_t ctrl_frame_[CTRL_FRAME_SIZE]{};
  /** @brief 预分配的反馈消息 */
  std_msgs::msg::Float32MultiArray feedback_msg_;
  /** @brief 预分配的 gyro_z 消息 */
  std_msgs::msg::Float32 gyro_z_msg_;

  /* ---- ROS 对象 ---- */

  /** @brief 200Hz 控制定时器 */
  rclcpp::TimerBase::SharedPtr control_timer_;
  /** @brief 遥控器通道订阅者 */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_channels_;
  /** @brief 键盘按键订阅者 */
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr sub_keyboard_;
  /** @brief 鼠标数据订阅者 */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_mouse_;
  /** @brief 遥控器按键切换状态订阅者 (供弹控制) */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_key_toggles_;
  /** @brief 反馈数据发布者 */
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_feedback_;
  /** @brief gyro_z 发布者 */
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gyro_z_;
};

/* ========================================================================= */
/*  Main                                                                     */
/* ========================================================================= */

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisControlNode>());
  rclcpp::shutdown();
  return 0;
}
