/**
 * @file vt_remote_node.cpp
 * @brief VTM 图传遥控器节点
 *
 * @details
 * 通过 UART 接收 VTM Receiver 输出的遥控器数据帧（21 字节，14ms 周期），
 * 解析后发布到 ROS2 话题供其他节点使用。
 *
 * UART 参数：921600 baud, 8N1, 无流控
 *
 * 发布话题：
 *   - /vt_remote/channels  (std_msgs/Int16MultiArray) : [ch0, ch1, ch2, ch3, wheel]
 *   - /vt_remote/mouse     (std_msgs/Int16MultiArray) : [mouse_x, mouse_y, mouse_z, left, right, middle]
 *   - /vt_remote/keyboard  (std_msgs/UInt16)          : 16位键盘按键状态（位映射见下）
 *   - /vt_remote/keyboard_toggles (std_msgs/UInt16)   : 16位键盘按键切换状态（按下上升沿翻转）
 *   - /vt_remote/keyboard_readable (std_msgs/String)  : 可读键盘状态字符串
 *   - /vt_remote/mouse_toggles (std_msgs/Int16MultiArray) : [left_toggle, right_toggle, middle_toggle]
 *   - /vt_remote/switches     (std_msgs/Int16MultiArray) : [mode, pause, fn_left, fn_right, trigger]
 *   - /vt_remote/key_toggles  (std_msgs/Int16MultiArray) : [pause, fn_left, fn_right, trigger]
 *
 * 键盘位映射（0=未按下，1=按下）：
 *   - bit0: W      bit4: Shift  bit8:  R   bit12: X
 *   - bit1: S      bit5: Ctrl   bit9:  F   bit13: C
 *   - bit2: A      bit6: Q      bit10: G   bit14: V
 *   - bit3: D      bit7: E      bit11: Z   bit15: B
 *
 * 参数：
 *   - serial_port (string) : 串口设备路径，默认 "/dev/ttyUSB1"
 *   - baud_rate   (int)    : 波特率，默认 921600
 *   - mouse_toggle_debounce (double) : 鼠标按键切换去抖时间（秒），默认 0.2
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

#include <cerrno>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "vt_remote_control/vt_data_frame.hpp"

using namespace vt_remote_control;

/**
 * @class VtRemoteNode
 * @brief 图传遥控器 ROS2 节点
 *
 * 负责从串口读取 VTM Receiver 数据帧，校验解析后发布到话题。
 */
class VtRemoteNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化参数、发布者、串口和定时器
   */
  VtRemoteNode()
  : Node("vt_remote_node"), serial_fd_(-1)
  {
    /* 声明 ROS 参数 */
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    this->declare_parameter<int>("baud_rate", 921600);
    this->declare_parameter<double>("mouse_toggle_debounce", 0.2);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    mouse_toggle_debounce_ = this->get_parameter("mouse_toggle_debounce").as_double();

    /* 创建发布者 */
    pub_channels_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/channels", 10);
    pub_mouse_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse", 10);
    pub_mouse_toggles_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse_toggles", 10);
    pub_keyboard_ = this->create_publisher<std_msgs::msg::UInt16>(
      "/vt_remote/keyboard", 10);
    pub_keyboard_toggles_ = this->create_publisher<std_msgs::msg::UInt16>(
      "/vt_remote/keyboard_toggles", 10);
    pub_keyboard_readable_ = this->create_publisher<std_msgs::msg::String>(
      "/vt_remote/keyboard_readable", 10);
    pub_switches_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/switches", 10);
    pub_key_toggles_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/key_toggles", 10);

    /* 打开串口 */
    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s，节点将尝试重连...",
                   serial_port_.c_str());
    }

    /* 创建定时器：5ms 检查一次串口数据（帧周期 14ms，保证不丢帧） */
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&VtRemoteNode::timerCallback, this));

    /* 初始化超时检测 */
    last_frame_time_ = this->now();
    is_timeout_ = false;

    RCLCPP_INFO(this->get_logger(),
                "VtRemoteNode 已启动 | 串口: %s | 波特率: %d",
                serial_port_.c_str(), baud_rate_);
  }

  /**
   * @brief 析构函数，关闭串口
   */
  ~VtRemoteNode() override
  {
    closeSerial();
  }

private:
  // =========================================================================
  //  串口操作
  // =========================================================================

  /**
   * @brief 打开并配置串口
   * @return true 成功，false 失败
   */
  bool openSerial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s (%s)",
                   serial_port_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr 失败: %s", strerror(errno));
      closeSerial();
      return false;
    }

    /* 设置波特率 */
    speed_t baud = baudToSpeed(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    /* 8N1: 8 数据位, 无校验, 1 停止位 */
    tty.c_cflag &= ~PARENB;   /* 无校验 */
    tty.c_cflag &= ~CSTOPB;   /* 1 停止位 */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       /* 8 数据位 */

    /* 无流控 */
    tty.c_cflag &= ~CRTSCTS;

    /* 启用接收, 本地模式 */
    tty.c_cflag |= (CLOCAL | CREAD);

    /* 原始输入模式 */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* 禁用软件流控和特殊字符处理 */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /* 原始输出模式 */
    tty.c_oflag &= ~OPOST;

    /* 读取设置：最少 1 字节，超时 100ms */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    /* 刷新并应用设置 */
    tcflush(serial_fd_, TCIFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr 失败: %s", strerror(errno));
      closeSerial();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "串口 %s 已打开", serial_port_.c_str());
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
  //  数据接收与解析
  // =========================================================================

  /**
   * @brief 定时器回调：从串口读取数据并尝试解析帧
   */
  void timerCallback()
  {
    /* 超时检测：5 秒内未收到有效帧则报警（放在最前面，确保始终执行） */
    if (!is_timeout_ && serial_fd_ >= 0 &&
        (this->now() - last_frame_time_).seconds() > 5.0) {
      RCLCPP_WARN(this->get_logger(), "连接超时：5 秒内未收到有效数据帧");
      is_timeout_ = true;
    }

    /* 如果串口未打开，尝试重连 */
    if (serial_fd_ < 0) {
      reconnect_counter_++;
      if (reconnect_counter_ >= 200) {  /* 约每 1 秒尝试一次 */
        reconnect_counter_ = 0;
        RCLCPP_WARN(this->get_logger(), "尝试重新打开串口 %s ...",
                    serial_port_.c_str());
        openSerial();
      }
      return;
    }

    /* 从串口读取数据到环形缓冲区 */
    uint8_t tmp_buf[256];
    ssize_t n = ::read(serial_fd_, tmp_buf, sizeof(tmp_buf));

    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", strerror(errno));
        closeSerial();
      }
      return;
    }

    if (n == 0) {
      return;
    }

    /* 追加到接收缓冲区 */
    for (ssize_t i = 0; i < n; ++i) {
      rx_buffer_.push_back(tmp_buf[i]);
    }

    /* 在缓冲区中搜索并解析完整帧 */
    processBuffer();
  }

  /**
   * @brief 处理接收缓冲区，搜索帧头并解析完整帧
   *
   * @details 使用状态机在缓冲区中查找帧头 0xAF 0x53，
   *          找到后等待凑够 21 字节，然后验证 CRC 并解析。
   */
  void processBuffer()
  {
    while (rx_buffer_.size() >= FRAME_LENGTH) {
      /* 查找帧头 */
      size_t header_pos = findFrameHeader();
      if (header_pos == std::string::npos) {
        /* 没找到帧头，只保留最后 1 字节（可能是 0xAF 的开头） */
        if (rx_buffer_.size() > 1) {
          rx_buffer_.erase(rx_buffer_.begin(),
                           rx_buffer_.begin() + static_cast<long>(rx_buffer_.size() - 1));
        }
        break;
      }

      /* 丢弃帧头之前的垃圾数据 */
      if (header_pos > 0) {
        rx_buffer_.erase(rx_buffer_.begin(),
                         rx_buffer_.begin() + static_cast<long>(header_pos));
      }

      /* 检查剩余数据是否够一帧 */
      if (rx_buffer_.size() < FRAME_LENGTH) {
        break;
      }

      /* 尝试解析 */
      RemoteData data{};
      if (parseFrame(rx_buffer_.data(), data)) {
        publishData(data);
        frame_error_count_ = 0;
        last_frame_time_ = this->now();
        if (is_timeout_) {
          RCLCPP_INFO(this->get_logger(), "数据帧恢复接收");
          is_timeout_ = false;
        }
      } else {
        frame_error_count_++;
        if (frame_error_count_ % 100 == 1) {
          RCLCPP_WARN(this->get_logger(),
                      "帧解析失败（CRC 错误），累计: %lu", frame_error_count_);
        }
      }

      /* 移除已处理的帧 */
      rx_buffer_.erase(rx_buffer_.begin(),
                       rx_buffer_.begin() + FRAME_LENGTH);
    }

    /* 防止缓冲区无限增长 */
    if (rx_buffer_.size() > 1024) {
      rx_buffer_.erase(rx_buffer_.begin(),
                       rx_buffer_.begin() + static_cast<long>(rx_buffer_.size() - FRAME_LENGTH));
    }
  }

  /**
   * @brief 在接收缓冲区中查找帧头 (0xAF, 0x53)
   * @return 帧头起始位置，未找到返回 string::npos
   */
  size_t findFrameHeader() const
  {
    if (rx_buffer_.size() < 2) {
      return std::string::npos;
    }
    for (size_t i = 0; i <= rx_buffer_.size() - 2; ++i) {
      if (rx_buffer_[i] == FRAME_HEADER_1 && rx_buffer_[i + 1] == FRAME_HEADER_2) {
        return i;
      }
    }
    return std::string::npos;
  }

  // =========================================================================
  //  话题发布
  // =========================================================================

  /**
   * @brief 将解析后的遥控器数据发布到各话题
   * @param data 已解析的遥控器数据
   */
  void publishData(const RemoteData &data)
  {
    /* 通道数据: [ch0, ch1, ch2, ch3, wheel] */
    auto channels_msg = std_msgs::msg::Int16MultiArray();
    channels_msg.data = {
      static_cast<int16_t>(data.ch0),
      static_cast<int16_t>(data.ch1),
      static_cast<int16_t>(data.ch2),
      static_cast<int16_t>(data.ch3),
      static_cast<int16_t>(data.wheel)
    };
    pub_channels_->publish(channels_msg);

    /* 鼠标数据: [mouse_x, mouse_y, mouse_z, left, right, middle] */
    auto mouse_msg = std_msgs::msg::Int16MultiArray();
    mouse_msg.data = {
      data.mouse_x,
      data.mouse_y,
      data.mouse_z,
      static_cast<int16_t>(data.mouse_left),
      static_cast<int16_t>(data.mouse_right),
      static_cast<int16_t>(data.mouse_middle)
    };
    pub_mouse_->publish(mouse_msg);

    /* 鼠标按键切换状态: [left_toggle, right_toggle, middle_toggle] */
    updateMouseToggleStates(data);
    auto mouse_toggles_msg = std_msgs::msg::Int16MultiArray();
    mouse_toggles_msg.data = {
      static_cast<int16_t>(mouse_left_toggle_state_),
      static_cast<int16_t>(mouse_right_toggle_state_),
      static_cast<int16_t>(mouse_middle_toggle_state_)
    };
    pub_mouse_toggles_->publish(mouse_toggles_msg);

    /**
     * @brief 键盘数据（16位位图）
     *
     * 位定义：
     * bit0~bit15 = [W, S, A, D, Shift, Ctrl, Q, E, R, F, G, Z, X, C, V, B]
     * 0 表示未按下，1 表示按下。
     */
    auto keyboard_msg = std_msgs::msg::UInt16();
    keyboard_msg.data = data.keyboard;
    pub_keyboard_->publish(keyboard_msg);

    /* 键盘切换状态（16位位图，上升沿翻转） */
    updateKeyboardToggleState(data.keyboard);
    auto keyboard_toggles_msg = std_msgs::msg::UInt16();
    keyboard_toggles_msg.data = keyboard_toggle_state_;
    pub_keyboard_toggles_->publish(keyboard_toggles_msg);

    /**
     * @brief 可读键盘状态字符串
     *
     * 键顺序：
     * W, S, A, D, Shift, Ctrl, Q, E, R, F, G, Z, X, C, V, B
     */
    auto keyboard_readable_msg = std_msgs::msg::String();
    keyboard_readable_msg.data = buildKeyboardReadableString(data.keyboard);
    pub_keyboard_readable_->publish(keyboard_readable_msg);

    /* 开关与按键: [mode, pause, fn_left, fn_right, trigger] */
    auto switches_msg = std_msgs::msg::Int16MultiArray();
    switches_msg.data = {
      static_cast<int16_t>(data.mode_switch),
      static_cast<int16_t>(data.pause),
      static_cast<int16_t>(data.fn_left),
      static_cast<int16_t>(data.fn_right),
      static_cast<int16_t>(data.trigger)
    };
    pub_switches_->publish(switches_msg);

    /* 按键切换状态: [pause, fn_left, fn_right, trigger] */
    updateKeyToggleStates(data);
    auto key_toggles_msg = std_msgs::msg::Int16MultiArray();
    key_toggles_msg.data = {
      static_cast<int16_t>(pause_toggle_state_),
      static_cast<int16_t>(fn_left_toggle_state_),
      static_cast<int16_t>(fn_right_toggle_state_),
      static_cast<int16_t>(trigger_toggle_state_)
    };
    pub_key_toggles_->publish(key_toggles_msg);

    /* 调试日志（DEBUG 级别，默认不输出） */
    RCLCPP_DEBUG(this->get_logger(),
                 "CH:[%4u %4u %4u %4u] W:%4u M:%s T:%d P:%d | "
                 "Mouse:[%6d %6d %6d L:%d R:%d M:%d] KB:0x%04X",
                 data.ch0, data.ch1, data.ch2, data.ch3,
                 data.wheel, data.getModeName().c_str(),
                 data.trigger, data.pause,
                 data.mouse_x, data.mouse_y, data.mouse_z,
                 data.mouse_left, data.mouse_right, data.mouse_middle,
                 data.keyboard);
  }

  /**
   * @brief 更新四个按键的按下切换状态（上升沿翻转）
   * @param data 已解析的遥控器数据
   */
  void updateKeyToggleStates(const RemoteData &data)
  {
    const bool pause_pressed = (data.pause != 0);
    const bool fn_left_pressed = (data.fn_left != 0);
    const bool fn_right_pressed = (data.fn_right != 0);
    const bool trigger_pressed = (data.trigger != 0);

    if (!key_toggle_initialized_) {
      prev_pause_pressed_ = pause_pressed;
      prev_fn_left_pressed_ = fn_left_pressed;
      prev_fn_right_pressed_ = fn_right_pressed;
      prev_trigger_pressed_ = trigger_pressed;
      key_toggle_initialized_ = true;
      return;
    }

    toggleOnRisingEdge(pause_pressed, prev_pause_pressed_, pause_toggle_state_);
    toggleOnRisingEdge(fn_left_pressed, prev_fn_left_pressed_, fn_left_toggle_state_);
    toggleOnRisingEdge(fn_right_pressed, prev_fn_right_pressed_, fn_right_toggle_state_);
    toggleOnRisingEdge(trigger_pressed, prev_trigger_pressed_, trigger_toggle_state_);
  }

  /**
   * @brief 上升沿检测并翻转目标状态
   * @param current_pressed 当前是否按下
   * @param prev_pressed 上次是否按下（函数内会更新）
   * @param toggle_state 按键切换状态（函数内可能翻转）
   */
  static void toggleOnRisingEdge(bool current_pressed, bool &prev_pressed, bool &toggle_state)
  {
    if (current_pressed && !prev_pressed) {
      toggle_state = !toggle_state;
    }
    prev_pressed = current_pressed;
  }

  /**
   * @brief 上升沿检测并翻转目标状态（带最小间隔去抖）
   * @param current_pressed 当前是否按下
   * @param prev_pressed 上次是否按下（函数内会更新）
   * @param toggle_state 按键切换状态（函数内可能翻转）
   * @param now 当前时间
   * @param last_toggle_time 上次翻转时间
   * @param has_last_toggle_time 上次翻转时间是否有效
   * @param debounce_seconds 去抖时间（秒）
   */
  static void toggleOnRisingEdgeWithDebounce(
    bool current_pressed,
    bool &prev_pressed,
    bool &toggle_state,
    const rclcpp::Time &now,
    rclcpp::Time &last_toggle_time,
    bool &has_last_toggle_time,
    double debounce_seconds)
  {
    if (current_pressed && !prev_pressed) {
      const bool debounce_passed =
        !has_last_toggle_time || (now - last_toggle_time).seconds() >= debounce_seconds;
      if (debounce_passed) {
        toggle_state = !toggle_state;
        last_toggle_time = now;
        has_last_toggle_time = true;
      }
    }
    prev_pressed = current_pressed;
  }

  /**
   * @brief 更新鼠标三键的按下切换状态（上升沿翻转）
   * @param data 已解析的遥控器数据
   */
  void updateMouseToggleStates(const RemoteData &data)
  {
    const bool left_pressed = (data.mouse_left != 0);
    const bool right_pressed = (data.mouse_right != 0);
    const bool middle_pressed = (data.mouse_middle != 0);
    const auto now = this->now();

    if (!mouse_toggle_initialized_) {
      prev_mouse_left_pressed_ = left_pressed;
      prev_mouse_right_pressed_ = right_pressed;
      prev_mouse_middle_pressed_ = middle_pressed;
      mouse_toggle_initialized_ = true;
      return;
    }

    toggleOnRisingEdgeWithDebounce(
      left_pressed, prev_mouse_left_pressed_, mouse_left_toggle_state_,
      now, last_mouse_left_toggle_time_, has_mouse_left_toggle_time_, mouse_toggle_debounce_);
    toggleOnRisingEdgeWithDebounce(
      right_pressed, prev_mouse_right_pressed_, mouse_right_toggle_state_,
      now, last_mouse_right_toggle_time_, has_mouse_right_toggle_time_, mouse_toggle_debounce_);
    toggleOnRisingEdgeWithDebounce(
      middle_pressed, prev_mouse_middle_pressed_, mouse_middle_toggle_state_,
      now, last_mouse_middle_toggle_time_, has_mouse_middle_toggle_time_, mouse_toggle_debounce_);
  }

  /**
   * @brief 更新16位键盘按键切换状态（逐位上升沿翻转）
   * @param keyboard 当前键盘按键位图
   */
  void updateKeyboardToggleState(uint16_t keyboard)
  {
    if (!keyboard_toggle_initialized_) {
      prev_keyboard_pressed_ = keyboard;
      keyboard_toggle_initialized_ = true;
      return;
    }

    const uint16_t rising_edges = static_cast<uint16_t>(keyboard & (~prev_keyboard_pressed_));
    keyboard_toggle_state_ = static_cast<uint16_t>(keyboard_toggle_state_ ^ rising_edges);
    prev_keyboard_pressed_ = keyboard;
  }

  /**
   * @brief 构建可读键盘状态字符串
   * @param keyboard 16位按键位图
   * @return 格式化后的按键状态
   */
  static std::string buildKeyboardReadableString(uint16_t keyboard)
  {
    const auto keyState = [keyboard](KeyboardKey key) -> int {
      return (keyboard & static_cast<uint16_t>(key)) ? 1 : 0;
    };

    return
      "W:" + std::to_string(keyState(KEY_W)) +
      " S:" + std::to_string(keyState(KEY_S)) +
      " A:" + std::to_string(keyState(KEY_A)) +
      " D:" + std::to_string(keyState(KEY_D)) +
      " Shift:" + std::to_string(keyState(KEY_SHIFT)) +
      " Ctrl:" + std::to_string(keyState(KEY_CTRL)) +
      " Q:" + std::to_string(keyState(KEY_Q)) +
      " E:" + std::to_string(keyState(KEY_E)) +
      " R:" + std::to_string(keyState(KEY_R)) +
      " F:" + std::to_string(keyState(KEY_F)) +
      " G:" + std::to_string(keyState(KEY_G)) +
      " Z:" + std::to_string(keyState(KEY_Z)) +
      " X:" + std::to_string(keyState(KEY_X)) +
      " C:" + std::to_string(keyState(KEY_C)) +
      " V:" + std::to_string(keyState(KEY_V)) +
      " B:" + std::to_string(keyState(KEY_B));
  }

  // =========================================================================
  //  成员变量
  // =========================================================================

  /** @brief 串口设备路径 */
  std::string serial_port_;

  /** @brief 波特率 */
  int baud_rate_;
  /** @brief 鼠标按键切换去抖时间（秒） */
  double mouse_toggle_debounce_ = 0.2;

  /** @brief 串口文件描述符 */
  int serial_fd_;

  /** @brief 接收数据缓冲区 */
  std::vector<uint8_t> rx_buffer_;

  /** @brief 重连计数器 */
  int reconnect_counter_ = 0;

  /** @brief 帧错误计数 */
  size_t frame_error_count_ = 0;

  /** @brief 上次成功接收数据帧的时间 */
  rclcpp::Time last_frame_time_;

  /** @brief 是否处于超时状态 */
  bool is_timeout_ = false;

  /** @brief 定时器 */
  rclcpp::TimerBase::SharedPtr timer_;

  /** @brief 通道数据发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_channels_;

  /** @brief 鼠标数据发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_mouse_;

  /** @brief 鼠标按键切换状态发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_mouse_toggles_;

  /** @brief 键盘数据发布者 */
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_keyboard_;

  /** @brief 键盘切换状态发布者 */
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_keyboard_toggles_;

  /** @brief 可读键盘状态发布者 */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_keyboard_readable_;

  /** @brief 开关按键发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_switches_;

  /** @brief 按键切换状态发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_key_toggles_;

  /** @brief 按键切换状态是否已完成首次初始化 */
  bool key_toggle_initialized_ = false;
  /** @brief pause 上次按下状态 */
  bool prev_pause_pressed_ = false;
  /** @brief fn_left 上次按下状态 */
  bool prev_fn_left_pressed_ = false;
  /** @brief fn_right 上次按下状态 */
  bool prev_fn_right_pressed_ = false;
  /** @brief trigger 上次按下状态 */
  bool prev_trigger_pressed_ = false;

  /** @brief pause 切换状态 */
  bool pause_toggle_state_ = false;
  /** @brief fn_left 切换状态 */
  bool fn_left_toggle_state_ = false;
  /** @brief fn_right 切换状态 */
  bool fn_right_toggle_state_ = false;
  /** @brief trigger 切换状态 */
  bool trigger_toggle_state_ = false;

  /** @brief 键盘切换状态是否已完成首次初始化 */
  bool keyboard_toggle_initialized_ = false;
  /** @brief 键盘上一次按下状态位图 */
  uint16_t prev_keyboard_pressed_ = 0;
  /** @brief 键盘切换状态位图 */
  uint16_t keyboard_toggle_state_ = 0;

  /** @brief 鼠标按键切换状态是否已完成首次初始化 */
  bool mouse_toggle_initialized_ = false;
  /** @brief 鼠标左键上次按下状态 */
  bool prev_mouse_left_pressed_ = false;
  /** @brief 鼠标右键上次按下状态 */
  bool prev_mouse_right_pressed_ = false;
  /** @brief 鼠标中键上次按下状态 */
  bool prev_mouse_middle_pressed_ = false;
  /** @brief 鼠标左键切换状态 */
  bool mouse_left_toggle_state_ = false;
  /** @brief 鼠标右键切换状态 */
  bool mouse_right_toggle_state_ = false;
  /** @brief 鼠标中键切换状态 */
  bool mouse_middle_toggle_state_ = false;
  /** @brief 鼠标左键最近切换时间 */
  rclcpp::Time last_mouse_left_toggle_time_;
  /** @brief 鼠标右键最近切换时间 */
  rclcpp::Time last_mouse_right_toggle_time_;
  /** @brief 鼠标中键最近切换时间 */
  rclcpp::Time last_mouse_middle_toggle_time_;
  /** @brief 鼠标左键最近切换时间是否有效 */
  bool has_mouse_left_toggle_time_ = false;
  /** @brief 鼠标右键最近切换时间是否有效 */
  bool has_mouse_right_toggle_time_ = false;
  /** @brief 鼠标中键最近切换时间是否有效 */
  bool has_mouse_middle_toggle_time_ = false;
};

// ===========================================================================
//  Main
// ===========================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VtRemoteNode>());
  rclcpp::shutdown();
  return 0;
}
