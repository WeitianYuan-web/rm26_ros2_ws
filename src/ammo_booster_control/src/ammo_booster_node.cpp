/**
 * @file ammo_booster_node.cpp
 * @brief 弹仓拨弹控制节点
 *
 * @details
 * 订阅遥控器开关话题 /vt_remote/switches，通过 fn_right 按键切换
 * 供弹电机的低速 (500 RPM) 和高速 (4500 RPM) 两种模式。
 * 每次按下 fn_right（上升沿检测），速度模式在低速与高速之间切换。
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
 */

#include <rclcpp/rclcpp.hpp>
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
 * @brief switches 数据索引定义
 *
 * /vt_remote/switches 话题数据格式:
 *   [mode, pause, fn_left, fn_right, trigger]
 */
enum SwitchIndex
{
  SW_MODE     = 0,  ///< 模式开关
  SW_PAUSE    = 1,  ///< 暂停按键
  SW_FN_LEFT  = 2,  ///< 自定义按键（左）
  SW_FN_RIGHT = 3,  ///< 自定义按键（右）
  SW_TRIGGER  = 4,  ///< 扳机
  SW_COUNT    = 5   ///< 数据总数
};

/** @brief CAN 协议常量 */
static constexpr uint32_t CAN_SPEED_CMD_ID  = 0x100;  ///< PC -> MCU: 速度指令
static constexpr uint32_t CAN_FEEDBACK_ID   = 0x101;  ///< MCU -> PC: 电机反馈

/**
 * @class AmmoBoosterNode
 * @brief 弹仓拨弹控制节点类
 *
 * 负责：
 * - 订阅 /vt_remote/switches 检测 fn_right 按键边沿
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
    prev_fn_right_(0),
    current_speed_rpm_(0)
  {
    /* 声明 ROS 参数 */
    this->declare_parameter<std::string>("can_interface", "can2");
    this->declare_parameter<int>("low_speed_rpm", 500);
    this->declare_parameter<int>("high_speed_rpm", 4500);
    this->declare_parameter<int>("send_interval_ms", 50);

    can_interface_    = this->get_parameter("can_interface").as_string();
    low_speed_rpm_    = this->get_parameter("low_speed_rpm").as_int();
    high_speed_rpm_   = this->get_parameter("high_speed_rpm").as_int();
    int send_interval = this->get_parameter("send_interval_ms").as_int();

    /* 默认低速 */
    current_speed_rpm_ = low_speed_rpm_;

    /* 初始化 CAN 套接字 */
    if (!initCan()) {
      RCLCPP_ERROR(this->get_logger(),
                   "无法初始化 CAN 接口 %s，将持续重试...",
                   can_interface_.c_str());
    }

    /* 订阅遥控器开关话题 */
    sub_switches_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/vt_remote/switches", 10,
      std::bind(&AmmoBoosterNode::switchesCallback, this, std::placeholders::_1));

    /* 创建速度指令发送定时器 */
    send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(send_interval),
      std::bind(&AmmoBoosterNode::sendTimerCallback, this));

    /* 创建 CAN 重连定时器 (1 秒检查一次) */
    reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&AmmoBoosterNode::reconnectTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "AmmoBoosterNode 已启动 | CAN: %s | 低速: %d RPM | 高速: %d RPM | 发送周期: %d ms",
                can_interface_.c_str(), low_speed_rpm_, high_speed_rpm_, send_interval);
    RCLCPP_INFO(this->get_logger(), "当前模式: 低速 (%d RPM)", low_speed_rpm_);
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

    /* 设置接收过滤：只接收反馈帧 ID=0x101 */
    struct can_filter rfilter[1];
    rfilter[0].can_id   = CAN_FEEDBACK_ID;
    rfilter[0].can_mask = CAN_SFF_MASK;
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
   * @brief 遥控器开关话题回调
   *
   * 检测 fn_right 的上升沿（0->1），每次按下切换速度模式。
   *
   * @param msg switches 数据 [mode, pause, fn_left, fn_right, trigger]
   */
  void switchesCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (static_cast<int>(msg->data.size()) < SW_COUNT) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "switches 数据长度不足: %zu (期望 %d)",
                           msg->data.size(), SW_COUNT);
      return;
    }

    int16_t fn_right = msg->data[SW_FN_RIGHT];

    /* 上升沿检测：前一次为 0，当前为 1 */
    if (fn_right == 1 && prev_fn_right_ == 0) {
      is_high_speed_ = !is_high_speed_;
      current_speed_rpm_ = is_high_speed_ ? high_speed_rpm_ : low_speed_rpm_;

      RCLCPP_INFO(this->get_logger(),
                  "fn_right 按下 -> 切换至%s模式 (%d RPM)",
                  is_high_speed_ ? "高速" : "低速",
                  current_speed_rpm_);
    }

    prev_fn_right_ = fn_right;
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

  /** @brief fn_right 上一次的状态 (用于边沿检测) */
  int16_t prev_fn_right_;

  /** @brief 当前目标速度 (RPM) */
  int current_speed_rpm_;

  /** @brief 遥控器开关订阅者 */
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_switches_;

  /** @brief 速度指令发送定时器 */
  rclcpp::TimerBase::SharedPtr send_timer_;

  /** @brief CAN 重连定时器 */
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
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
