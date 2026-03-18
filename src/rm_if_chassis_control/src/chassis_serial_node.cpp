/**
 * @file chassis_serial_node.cpp
 * @brief 底盘串口通讯节点
 *
 * @details
 * 负责：
 * 1. 订阅 /chassis/command (std_msgs/Float32MultiArray)，缓存最新的控制指令
 * 2. 以 200Hz 频率通过 RS485 串口发送控制帧到 STM32 MCU
 * 3. 异步接收并发布 MCU 反馈数据到 /chassis/feedback 和 /chassis/gyro_z
 *
 * 控制帧 PC→MCU (35B):
 *   [0xAA] + max_linear_accel(f32) + max_angular_accel(f32)
 *          + max_linear_vel(f32)   + max_angular_vel(f32)
 *          + vx(f32) + vy(f32) + vw(f32) + feed_rpm(f32) + crc16(u16)
 *
 * 反馈帧 MCU→PC (52B):
 *   [0x55] + x(f32) + y(f32) + θ(f32) + vx(f32) + vy(f32) + vw(f32)
 *          + wheel1_v(f32) + wheel2_v(f32) + wheel3_v(f32) + wheel4_v(f32)
 *          + feed_rpm(f32) + gyro_z(f32) + crc16(u16) + [0xAA]
 *
 * 订阅话题:
 *   - /chassis/command (std_msgs/Float32MultiArray):
 *     [max_linear_accel, max_angular_accel, max_linear_vel, max_angular_vel, vx, vy, vw, feed_rpm]
 *
 * 发布话题:
 *   - /chassis/feedback (std_msgs/Float32MultiArray)
 *   - /chassis/gyro_z (std_msgs/Float32)
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

/* ========================================================================= */
/*  协议常量                                                                  */
/* ========================================================================= */

static constexpr uint8_t CTRL_FRAME_HEADER = 0xAA;
static constexpr size_t CTRL_FRAME_CRC_OFFSET = 33;
static constexpr size_t CTRL_FRAME_CRC_LEN = 33;
static constexpr size_t CTRL_FRAME_SIZE = 35;

static constexpr uint8_t FB_FRAME_HEADER = 0x55;
static constexpr uint8_t FB_FRAME_TAIL = 0xAA;
static constexpr size_t FB_FRAME_CRC_OFFSET = 49;
static constexpr size_t FB_FRAME_CRC_LEN = 49;
static constexpr size_t FB_FRAME_SIZE = 52;
static constexpr size_t FB_FLOAT_COUNT = 12;

/**
 * @brief CRC16-CCITT 计算
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

static inline void writeU16LE(uint8_t * dst, uint16_t value)
{
  dst[0] = static_cast<uint8_t>(value & 0xFFu);
  dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

static inline uint16_t readU16LE(const uint8_t * src)
{
  return static_cast<uint16_t>(src[0]) |
         (static_cast<uint16_t>(src[1]) << 8);
}

/**
 * @class ChassisSerialNode
 * @brief 底盘串口通讯 ROS2 节点
 */
class ChassisSerialNode : public rclcpp::Node
{
public:
  ChassisSerialNode()
  : Node("chassis_serial_node"), serial_fd_(-1)
  {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 460800);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_   = this->get_parameter("baud_rate").as_int();

    /* 订阅控制指令 */
    sub_command_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/chassis/command", 10,
      std::bind(&ChassisSerialNode::commandCallback, this, std::placeholders::_1));

    /* 反馈发布者 */
    pub_feedback_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/chassis/feedback", 10);
    pub_gyro_z_ = this->create_publisher<std_msgs::msg::Float32>(
      "/chassis/gyro_z", 10);

    /* 打开串口 */
    if (!openSerial()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s，节点将尝试重连...",
                   serial_port_.c_str());
    }

    /* 预分配 */
    ctrl_frame_[0] = CTRL_FRAME_HEADER;
    feedback_msg_.data.resize(FB_FLOAT_COUNT, 0.0f);
    rx_buffer_.reserve(256);

    /* 初始化命令数据 */
    cmd_data_.resize(8, 0.0f);
    last_command_time_ = this->now();
    last_feedback_time_ = this->now();

    /* 200Hz 控制定时器 (5ms) */
    control_timer_ = this->create_wall_timer(
      std::chrono::microseconds(5000),
      std::bind(&ChassisSerialNode::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "ChassisSerialNode 已启动 | 串口: %s | 波特率: %d | 频率: 200Hz",
                serial_port_.c_str(), baud_rate_);
  }

  ~ChassisSerialNode() override
  {
    sendStopCommand();
    closeSerial();
  }

private:
  void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 8) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "指令数据不足 8 个 float");
      return;
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (size_t i = 0; i < 8; ++i) {
      cmd_data_[i] = msg->data[i];
    }
    last_command_time_ = this->now();
  }

  void controlTimerCallback()
  {
    if (serial_fd_ < 0) {
      reconnect_counter_++;
      if (reconnect_counter_ >= 200) {
        reconnect_counter_ = 0;
        RCLCPP_WARN(this->get_logger(), "尝试重新打开串口 %s ...",
                    serial_port_.c_str());
        openSerial();
      }
      return;
    }

    float current_cmd[8];
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      // 安全保护：如果 0.5 秒没有收到命令，将速度清零（保持加速度限制）
      if ((this->now() - last_command_time_).seconds() > 0.5) {
        current_cmd[0] = cmd_data_[0];
        current_cmd[1] = cmd_data_[1];
        current_cmd[2] = cmd_data_[2];
        current_cmd[3] = cmd_data_[3];
        current_cmd[4] = 0.0f; // vx
        current_cmd[5] = 0.0f; // vy
        current_cmd[6] = 0.0f; // vw
        current_cmd[7] = 0.0f; // feed_rpm
      } else {
        std::memcpy(current_cmd, cmd_data_.data(), 8 * sizeof(float));
      }
    }

    sendControlFrame(current_cmd);
    readFeedback();
    checkFeedbackTimeout();
  }

  void sendControlFrame(const float * cmd)
  {
    std::memcpy(&ctrl_frame_[1],  &cmd[0], sizeof(float)); // max_linear_accel
    std::memcpy(&ctrl_frame_[5],  &cmd[1], sizeof(float)); // max_angular_accel
    std::memcpy(&ctrl_frame_[9],  &cmd[2], sizeof(float)); // max_linear_vel
    std::memcpy(&ctrl_frame_[13], &cmd[3], sizeof(float)); // max_angular_vel
    std::memcpy(&ctrl_frame_[17], &cmd[4], sizeof(float)); // vx
    std::memcpy(&ctrl_frame_[21], &cmd[5], sizeof(float)); // vy
    std::memcpy(&ctrl_frame_[25], &cmd[6], sizeof(float)); // vw
    std::memcpy(&ctrl_frame_[29], &cmd[7], sizeof(float)); // feed_rpm

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

  void readFeedback()
  {
    uint8_t tmp[256];
    const ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
    if (n <= 0) return;

    rx_buffer_.insert(rx_buffer_.end(), tmp, tmp + n);

    while (rx_buffer_.size() >= FB_FRAME_SIZE) {
      const size_t header_pos = findFeedbackHeader();
      if (header_pos == std::string::npos) {
        if (rx_buffer_.size() > FB_FRAME_SIZE - 1) {
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - static_cast<long>(FB_FRAME_SIZE - 1));
        }
        break;
      }

      if (header_pos > 0) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(header_pos));
      }

      if (rx_buffer_.size() < FB_FRAME_SIZE) break;

      if (rx_buffer_[FB_FRAME_SIZE - 1] == FB_FRAME_TAIL) {
        const uint16_t rx_crc = readU16LE(&rx_buffer_[FB_FRAME_CRC_OFFSET]);
        const uint16_t calc_crc = CRC16_CCITT(rx_buffer_.data(), static_cast<uint16_t>(FB_FRAME_CRC_LEN));
        
        if (rx_crc != calc_crc) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "反馈帧 CRC 校验失败: rx=0x%04X calc=0x%04X", rx_crc, calc_crc);
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(FB_FRAME_SIZE));
          continue;
        }

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
      }

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<long>(FB_FRAME_SIZE));
    }

    if (rx_buffer_.size() > 512) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - static_cast<long>(FB_FRAME_SIZE));
    }
  }

  size_t findFeedbackHeader() const
  {
    for (size_t i = 0; i + FB_FRAME_SIZE <= rx_buffer_.size(); ++i) {
      if (rx_buffer_[i] == FB_FRAME_HEADER && rx_buffer_[i + FB_FRAME_SIZE - 1] == FB_FRAME_TAIL) {
        return i;
      }
    }
    return std::string::npos;
  }

  void checkFeedbackTimeout()
  {
    if (!feedback_timeout_ && (this->now() - last_feedback_time_).seconds() > 3.0) {
      RCLCPP_WARN(this->get_logger(), "MCU 反馈超时：3 秒内未收到有效反馈帧");
      feedback_timeout_ = true;
    }
  }

  void sendStopCommand()
  {
    if (serial_fd_ < 0) return;
    float stop_cmd[8] = {0};
    for (int i = 0; i < 3; ++i) {
      sendControlFrame(stop_cmd);
      usleep(5000);
    }
    RCLCPP_INFO(this->get_logger(), "已发送停止指令");
  }

  bool openSerial()
  {
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "打开串口失败: %s (%s)", serial_port_.c_str(), strerror(errno));
      return false;
    }

    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      closeSerial();
      return false;
    }

    const speed_t baud = baudToSpeed(baud_rate_);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(serial_fd_, TCIOFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      closeSerial();
      return false;
    }

    return true;
  }

  void closeSerial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  static speed_t baudToSpeed(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 115200: return B115200;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B921600;
    }
  }

  std::string serial_port_;
  int baud_rate_{};
  int serial_fd_;
  std::vector<uint8_t> rx_buffer_;
  int reconnect_counter_ = 0;

  std::mutex cmd_mutex_;
  std::vector<float> cmd_data_;
  rclcpp::Time last_command_time_;

  rclcpp::Time last_feedback_time_;
  bool feedback_timeout_ = false;

  uint8_t ctrl_frame_[CTRL_FRAME_SIZE]{};
  std_msgs::msg::Float32MultiArray feedback_msg_;
  std_msgs::msg::Float32 gyro_z_msg_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_command_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_feedback_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gyro_z_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisSerialNode>());
  rclcpp::shutdown();
  return 0;
}
