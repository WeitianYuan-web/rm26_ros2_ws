/**
 * @file ammo_booster_node.cpp
 * @brief 弹仓拨弹控制节点
 *
 * @details
 * 订阅遥控器鼠标话题 /vt_remote/mouse，使用鼠标左键按下上升沿切换
 * 发射电机低速 (500 RPM) 和高速 (4500 RPM) 两种模式。
 * 默认启动为低速模式。
 *
 * CAN 协议 (与 CtrBoard-H7 通信):
 *   - 发送: ID=0x100, Data[0..1] = int16 速度 (RPM, 大端序)
 *   - 接收: ID=0x101, Data[0..7] = 电机0速度, 电机0电流, 电机1速度, 电机1电流
 *
 * 参数:
 *   - can_interface (string) : CAN 设备名，默认 "can2"
 *   - low_speed_rpm  (int)   : 低速模式 RPM，默认 500
 *   - high_speed_rpm (int)   : 高速模式 RPM，默认 4500
 *   - send_interval_ms (int) : 速度指令发送周期 (ms)，默认 50
 *   - recv_interval_ms (int) : CAN 反馈读取周期 (ms)，默认 2
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>

/**
 * @brief mouse 数据索引定义
 *
 * /vt_remote/mouse 话题数据格式:
 *   [mouse_x, mouse_y, mouse_z, left, right, middle]
 */
enum MouseIndex
{
  MOUSE_LEFT = 3,   ///< 鼠标左键
  MOUSE_RIGHT = 4,  ///< 鼠标右键
  MOUSE_COUNT = 6   ///< 数据总数
};

/**
 * @brief key_toggles 数据索引定义
 *
 * /vt_remote/key_toggles 话题数据格式:
 *   [pause, fn_left, fn_right, trigger]
 */
enum KeyToggleIndex
{
  TG_PAUSE    = 0,  ///< 暂停按键切换状态
  TG_FN_LEFT  = 1,  ///< 自定义按键（左）切换状态
  TG_FN_RIGHT = 2,  ///< 自定义按键（右）切换状态
  TG_TRIGGER  = 3,  ///< 扳机切换状态
  TG_COUNT    = 4   ///< 数据总数
};

/**
 * @brief 发射控制输入源
 */
enum class FireControlSource
{
  MOUSE = 0,   ///< 仅鼠标
  REMOTE = 1,  ///< 仅遥控器按键
  HYBRID = 2   ///< 混合模式（鼠标优先窗口）
};

/** @brief CAN 协议常量 */
static constexpr uint32_t CAN_SPEED_CMD_ID  = 0x100;  ///< PC -> MCU: 速度指令
static constexpr uint32_t CAN_FEEDBACK_ID   = 0x101;  ///< MCU -> PC: 电机反馈
static constexpr uint32_t CAN_GYRO_Z_ID     = 0x102;  ///< MCU -> PC: 陀螺仪 Z 轴角速度
static constexpr size_t FEEDBACK_DATA_COUNT = 4;

/**
 * @class AmmoBoosterNode
 * @brief 弹仓拨弹控制节点类
 *
 * 负责：
 * - 订阅 /vt_remote/mouse 使用鼠标左键上升沿切换速度模式
 * - 通过 SocketCAN 向 CtrBoard-H7 发送速度指令
 * - 在低速/高速两种模式之间切换
 */
class AmmoBoosterNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数，初始化参数、CAN 接口、订阅者和定时器
   */
  AmmoBoosterNode()
  : Node("ammo_booster_node"),
    can_fd_(-1),
    is_high_speed_(false),
    current_speed_rpm_(0)
  {
    /* 声明 ROS 参数 */
    this->declare_parameter<std::string>("can_interface", "can2");
    this->declare_parameter<int>("low_speed_rpm", 500);
    this->declare_parameter<int>("high_speed_rpm", 4500);
    this->declare_parameter<int>("send_interval_ms", 50);
    this->declare_parameter<int>("recv_interval_ms", 2);
    this->declare_parameter<std::string>("fire_control_source", "hybrid");
    this->declare_parameter<double>("input_priority_timeout", 0.3);

    can_interface_    = this->get_parameter("can_interface").as_string();
    low_speed_rpm_    = this->get_parameter("low_speed_rpm").as_int();
    high_speed_rpm_   = this->get_parameter("high_speed_rpm").as_int();
    int send_interval = this->get_parameter("send_interval_ms").as_int();
    int recv_interval = this->get_parameter("recv_interval_ms").as_int();
    const std::string fire_control_source =
      this->get_parameter("fire_control_source").as_string();
    input_priority_timeout_ = this->get_parameter("input_priority_timeout").as_double();
    fire_control_source_ = parseFireControlSource(fire_control_source);

    /* 默认低速 */
    current_speed_rpm_ = low_speed_rpm_;

    /* 初始化 CAN 套接字 */
    if (!initCan()) {
      RCLCPP_ERROR(this->get_logger(),
                   "无法初始化 CAN 接口 %s，将持续重试...",
                   can_interface_.c_str());
    }

    /* 订阅遥控器鼠标话题 */
    sub_mouse_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/mouse", 10,
      std::bind(&AmmoBoosterNode::mouseCallback, this, std::placeholders::_1));
    sub_key_toggles_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/key_toggles", 10,
      std::bind(&AmmoBoosterNode::keyTogglesCallback, this, std::placeholders::_1));

    pub_feedback_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "/ammo_booster/feedback", 10);
    pub_gyro_z_ = this->create_publisher<std_msgs::msg::Float64>(
      "/gimbal/gyro_z", rclcpp::QoS(rclcpp::KeepLast(200)));
    feedback_msg_.data.resize(FEEDBACK_DATA_COUNT, 0);

    /* 创建速度指令发送定时器 */
    send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(send_interval),
      std::bind(&AmmoBoosterNode::sendTimerCallback, this));
    recv_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(recv_interval),
      std::bind(&AmmoBoosterNode::recvTimerCallback, this));

    /* 创建 CAN 重连定时器 (1 秒检查一次) */
    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AmmoBoosterNode::reconnectTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "AmmoBoosterNode 已启动 | CAN: %s | 低速: %d RPM | 高速: %d RPM | 发送周期: %d ms | 接收周期: %d ms",
                can_interface_.c_str(), low_speed_rpm_, high_speed_rpm_, send_interval, recv_interval);
    RCLCPP_INFO(this->get_logger(), "当前模式: 低速 (%d RPM)", low_speed_rpm_);
    RCLCPP_INFO(this->get_logger(),
                "发射控制输入源: %s",
                fireControlSourceToString(fire_control_source_));
  }

  /**
   * @brief 析构函数，停止电机并关闭 CAN
   */
  ~AmmoBoosterNode() override
  {
    /* 发送停止指令 */
    sendSpeedCommand(0);
    closeCan();
    RCLCPP_INFO(this->get_logger(), "AmmoBoosterNode 已关闭，电机已停止");
  }

private:
  // =========================================================================
  //  CAN 接口操作
  // =========================================================================

  /**
   * @brief 解析发射控制输入源参数
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
   * @brief 初始化 SocketCAN 接口
   * @return true 成功，false 失败
   */
  bool initCan()
  {
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "创建 CAN 套接字失败: %s", strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "CAN 接口 %s 不存在: %s", can_interface_.c_str(), strerror(errno));
      closeCan();
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "绑定 CAN 接口 %s 失败: %s", can_interface_.c_str(), strerror(errno));
      closeCan();
      return false;
    }

    /* 设置接收过滤：接收电机反馈与陀螺仪反馈帧 */
    struct can_filter rfilter[2];
    rfilter[0].can_id   = CAN_FEEDBACK_ID;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id   = CAN_GYRO_Z_ID;
    rfilter[1].can_mask = CAN_SFF_MASK;
    setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    RCLCPP_INFO(this->get_logger(), "CAN 接口 %s 已初始化", can_interface_.c_str());
    return true;
  }

  /**
   * @brief 关闭 CAN 套接字
   */
  void closeCan()
  {
    if (can_fd_ >= 0) {
      ::close(can_fd_);
      can_fd_ = -1;
    }
  }

  /**
   * @brief 发送速度指令到 MCU
   * @param speed_rpm 目标速度 (int16, RPM)
   * @return true 发送成功，false 失败
   */
  bool sendSpeedCommand(int speed_rpm)
  {
    if (can_fd_ < 0) {
      return false;
    }

    /* 限幅到 int16 范围 */
    speed_rpm = std::max(-32768, std::min(32767, speed_rpm));

    struct can_frame frame {};
    frame.can_id  = CAN_SPEED_CMD_ID;
    frame.can_dlc = 2;

    /* 大端序: 高字节在前 */
    int16_t spd = static_cast<int16_t>(speed_rpm);
    frame.data[0] = static_cast<uint8_t>((spd >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(spd & 0xFF);

    ssize_t nbytes = write(can_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "CAN 发送失败: %s", strerror(errno));
      return false;
    }
    return true;
  }

  // =========================================================================
  //  回调函数
  // =========================================================================

  /**
   * @brief 遥控器鼠标回调（左键状态控制发射速度）
   * @param msg mouse 数据 [mouse_x, mouse_y, mouse_z, left, right, middle]
   */
  void mouseCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (fire_control_source_ == FireControlSource::REMOTE) {
      return;
    }

    if (static_cast<int>(msg->data.size()) < MOUSE_COUNT) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "mouse 数据长度不足: %zu (期望 %d)",
                           msg->data.size(), MOUSE_COUNT);
      return;
    }

    const bool left_pressed = (msg->data[MOUSE_LEFT] != 0);
    const bool state_changed =
      !mouse_left_state_initialized_ || (left_pressed != prev_mouse_left_pressed_);
    if (state_changed) {
      last_mouse_left_toggle_time_ = this->now();
      mouse_left_toggle_time_valid_ = true;
    }

    if (state_changed) {
      is_high_speed_ = left_pressed;
      current_speed_rpm_ = is_high_speed_ ? high_speed_rpm_ : low_speed_rpm_;
      RCLCPP_INFO(this->get_logger(),
                  "鼠标左键状态变化 -> 切换至%s模式 (%d RPM)",
                  is_high_speed_ ? "高速" : "低速",
                  current_speed_rpm_);
    }
    mouse_left_state_initialized_ = true;
    prev_mouse_left_pressed_ = left_pressed;
  }

  /**
   * @brief 遥控器按键切换状态回调（保留原有 fn_right 控制逻辑）
   * @param msg key_toggles 数据 [pause, fn_left, fn_right, trigger]
   */
  void keyTogglesCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (fire_control_source_ == FireControlSource::MOUSE) {
      return;
    }

    if (static_cast<int>(msg->data.size()) < TG_COUNT) {
      return;
    }

    const bool mouse_priority_active =
      (fire_control_source_ == FireControlSource::HYBRID) &&
      mouse_left_toggle_time_valid_ &&
      (this->now() - last_mouse_left_toggle_time_).seconds() < input_priority_timeout_;
    if (mouse_priority_active) {
      return;
    }

    const bool fn_right_toggle = (msg->data[TG_FN_RIGHT] != 0);

    if (!fn_right_toggle_initialized_) {
      prev_fn_right_toggle_ = fn_right_toggle;
      fn_right_toggle_initialized_ = true;
      return;
    }

    if (fn_right_toggle != prev_fn_right_toggle_) {
      const bool new_high_speed = fn_right_toggle;
      is_high_speed_ = new_high_speed;
      current_speed_rpm_ = is_high_speed_ ? high_speed_rpm_ : low_speed_rpm_;
      RCLCPP_INFO(this->get_logger(),
                  "fn_right 切换 -> 切换至%s模式 (%d RPM)",
                  is_high_speed_ ? "高速" : "低速",
                  current_speed_rpm_);
    }
    prev_fn_right_toggle_ = fn_right_toggle;
  }

  /**
   * @brief 速度指令发送定时器回调
   *
   * 以固定周期向 CAN 总线发送当前目标速度。
   */
  void sendTimerCallback()
  {
    if (can_fd_ < 0) {
      return;
    }
    sendSpeedCommand(current_speed_rpm_);
  }

  /**
   * @brief CAN 接收与反馈发布定时器回调
   */
  void recvTimerCallback()
  {
    if (can_fd_ < 0) {
      return;
    }
    readAndPublishFeedback();
  }

  void readAndPublishFeedback()
  {
    if (can_fd_ < 0) {
      return;
    }

    struct can_frame frame {};
    while (true) {
      const ssize_t nbytes = recv(can_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "CAN 接收失败: %s", strerror(errno));
        break;
      }
      if (nbytes == 0) {
        break;
      }
      if (static_cast<size_t>(nbytes) != sizeof(frame)) {
        continue;
      }
      const uint32_t can_id = (frame.can_id & CAN_SFF_MASK);
      if (can_id == CAN_FEEDBACK_ID) {
        if (frame.can_dlc < 8) {
          continue;
        }
        const int16_t motor0_speed =
          static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]);
        const int16_t motor0_current =
          static_cast<int16_t>((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]);
        const int16_t motor1_speed =
          static_cast<int16_t>((static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
        const int16_t motor1_current =
          static_cast<int16_t>((static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7]);

        feedback_msg_.data[0] = motor0_speed;
        feedback_msg_.data[1] = motor0_current;
        feedback_msg_.data[2] = motor1_speed;
        feedback_msg_.data[3] = motor1_current;
        pub_feedback_->publish(feedback_msg_);
      } else if (can_id == CAN_GYRO_Z_ID) {
        if (frame.can_dlc < 2) {
          continue;
        }
        const int16_t raw =
          static_cast<int16_t>((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]);
        std_msgs::msg::Float64 gyro_msg;
        gyro_msg.data = static_cast<double>(raw) / 100.0;
        pub_gyro_z_->publish(gyro_msg);
      }
    }
  }

  /**
   * @brief CAN 重连定时器回调
   *
   * 若 CAN 套接字未打开，尝试重新初始化。
   */
  void reconnectTimerCallback()
  {
    if (can_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(),
                  "尝试重新初始化 CAN 接口 %s ...", can_interface_.c_str());
      initCan();
    }
  }

  // =========================================================================
  //  成员变量
  // =========================================================================

  /** @brief CAN 接口名称 (如 "can2") */
  std::string can_interface_;

  /** @brief CAN 套接字文件描述符 */
  int can_fd_;

  /** @brief 低速模式 RPM */
  int low_speed_rpm_;

  /** @brief 高速模式 RPM */
  int high_speed_rpm_;

  /** @brief 当前是否为高速模式 */
  bool is_high_speed_;
  /** @brief 发射控制输入源 */
  FireControlSource fire_control_source_{FireControlSource::HYBRID};
  /** @brief 键鼠优先窗口时长（秒） */
  double input_priority_timeout_{};
  /** @brief 鼠标左键最近切换时间 */
  rclcpp::Time last_mouse_left_toggle_time_;
  /** @brief 鼠标左键切换时间是否有效 */
  bool mouse_left_toggle_time_valid_ = false;
  /** @brief 鼠标左键上一次按下状态 */
  bool prev_mouse_left_pressed_ = false;
  /** @brief 鼠标左键状态是否已初始化 */
  bool mouse_left_state_initialized_ = false;
  /** @brief fn_right 切换状态是否已初始化 */
  bool fn_right_toggle_initialized_ = false;
  /** @brief fn_right 上一次切换状态 */
  bool prev_fn_right_toggle_ = false;

  /** @brief 当前目标速度 (RPM) */
  int current_speed_rpm_;

  /** @brief 遥控器鼠标订阅者 */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_mouse_;
  /** @brief 遥控器按键切换状态订阅者 */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_key_toggles_;
  /** @brief 电机反馈发布者 */
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_feedback_;
  /** @brief 云台陀螺仪 Z 轴角速度发布者 (rad/s) */
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_gyro_z_;

  /** @brief 速度指令发送定时器 */
  rclcpp::TimerBase::SharedPtr send_timer_;
  /** @brief CAN 反馈接收定时器 */
  rclcpp::TimerBase::SharedPtr recv_timer_;

  /** @brief CAN 重连定时器 */
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std_msgs::msg::Int16MultiArray feedback_msg_;
};

// ===========================================================================
//  Main
// ===========================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmmoBoosterNode>());
  rclcpp::shutdown();
  return 0;
}
