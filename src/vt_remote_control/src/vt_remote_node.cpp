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
 *   - /vt_remote/keyboard  (std_msgs/UInt16)          : 16位键盘按键状态
 *   - /vt_remote/switches  (std_msgs/Int16MultiArray) : [mode, pause, fn_left, fn_right, trigger]
 *
 * 参数：
 *   - serial_port (string) : 串口设备路径，默认 "/dev/ttyTHS1"
 *   - baud_rate   (int)    : 波特率，默认 921600
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
    this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
    this->declare_parameter<int>("baud_rate", 921600);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    /* 创建发布者 */
    pub_channels_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/channels", 10);
    pub_mouse_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse", 10);
    pub_keyboard_ = this->create_publisher<std_msgs::msg::UInt16>(
      "/vt_remote/keyboard", 10);
    pub_switches_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/switches", 10);

    /* 打开串口 */
    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s，节点将尝试重连...",
                   serial_port_.c_str());
    }

    /* 创建定时器：5ms 检查一次串口数据（帧周期 14ms，保证不丢帧） */
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&VtRemoteNode::timerCallback, this));

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

    /* 键盘数据 */
    auto keyboard_msg = std_msgs::msg::UInt16();
    keyboard_msg.data = data.keyboard;
    pub_keyboard_->publish(keyboard_msg);

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

  // =========================================================================
  //  成员变量
  // =========================================================================

  /** @brief 串口设备路径 */
  std::string serial_port_;

  /** @brief 波特率 */
  int baud_rate_;

  /** @brief 串口文件描述符 */
  int serial_fd_;

  /** @brief 接收数据缓冲区 */
  std::vector<uint8_t> rx_buffer_;

  /** @brief 重连计数器 */
  int reconnect_counter_ = 0;

  /** @brief 帧错误计数 */
  size_t frame_error_count_ = 0;

  /** @brief 定时器 */
  rclcpp::TimerBase::SharedPtr timer_;

  /** @brief 通道数据发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_channels_;

  /** @brief 鼠标数据发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_mouse_;

  /** @brief 键盘数据发布者 */
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_keyboard_;

  /** @brief 开关按键发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_switches_;
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
