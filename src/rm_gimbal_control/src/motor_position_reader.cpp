/**
 * @file motor_position_reader.cpp
 * @brief 电机位置读取节点，用于测试电机最大最小位置
 * @author Auto-generated
 * @date 2026
 */

#include "motor_ros2/motor_cfg.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>

/**
 * @brief 电机位置读取节点类
 */
class MotorPositionReaderNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   */
  MotorPositionReaderNode()
      : rclcpp::Node("motor_position_reader_node"),
        motor_initialized_(false) {

    // 声明参数
    this->declare_parameter<std::string>("can_interface", "can1");
    this->declare_parameter<int>("motor_id", 0x01);
    this->declare_parameter<int>("master_id", 0xFF);
    this->declare_parameter<int>("actuator_type", 5); // RS05
    this->declare_parameter<double>("publish_rate", 50.0); // 发布频率 Hz

    // 获取参数
    std::string can_interface = this->get_parameter("can_interface").as_string();
    int motor_id = this->get_parameter("motor_id").as_int();
    int master_id = this->get_parameter("master_id").as_int();
    int actuator_type = this->get_parameter("actuator_type").as_int();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    // 使用参数创建电机对象
    motor_ = std::make_unique<RobStrideMotor>(
        can_interface, 
        static_cast<uint8_t>(master_id), 
        static_cast<uint8_t>(motor_id), 
        actuator_type);

    RCLCPP_INFO(this->get_logger(), "CAN接口: %s", can_interface.c_str());
    RCLCPP_INFO(this->get_logger(), "电机ID: 0x%02X", motor_id);
    RCLCPP_INFO(this->get_logger(), "主机ID: 0x%02X", master_id);
    RCLCPP_INFO(this->get_logger(), "电机类型: RS%02d", actuator_type);
    RCLCPP_INFO(this->get_logger(), "发布频率: %.1f Hz", publish_rate);

    // 创建发布者
    position_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
        "motor/position", 10);
    
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "motor/joint_state", 10);

    // 初始化电机（使用异步方式，避免阻塞）
    init_motor();

    // 创建定时器定期读取和发布位置
    auto period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(
        period, std::bind(&MotorPositionReaderNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "电机位置读取节点已启动");
    RCLCPP_INFO(this->get_logger(), "请手动控制电机来测试最大最小位置");
  }

  /**
   * @brief 析构函数
   */
  ~MotorPositionReaderNode() {
    // 先停止定时器
    if (timer_) {
      timer_->cancel();
    }
    
    RCLCPP_INFO(this->get_logger(), "正在停止电机...");
    if (motor_initialized_ && motor_) {
      try {
        motor_->Disenable_Motor(0);
        RCLCPP_INFO(this->get_logger(), "电机已停止");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "停止电机时出错: %s", e.what());
      }
    }
  }

private:
  /**
   * @brief 初始化电机（异步方式，带超时）
   */
  void init_motor() {
    if (!motor_) {
      RCLCPP_ERROR(this->get_logger(), "电机对象未创建");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "正在初始化电机...");
    
    // 在单独线程中初始化，避免阻塞主线程
    std::thread init_thread([this]() {
      try {
        // 读取当前电机模式
        motor_->Get_RobStrite_Motor_parameter(0x7005);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 如果不是运控模式，切换到运控模式
        if (motor_->drw.run_mode.data != 0) {
          RCLCPP_INFO(this->get_logger(), "当前模式: %d, 切换到运控模式...", 
                      motor_->drw.run_mode.data);
          motor_->Disenable_Motor(0);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          
          motor_->Set_RobStrite_Motor_parameter(0x7005, 0, 'j'); // 设置为运控模式
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          
          motor_->Get_RobStrite_Motor_parameter(0x7005);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        motor_->enable_motor();
        motor_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "电机已使能（运控模式），kp=0 kd=0，可手动移动");
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), 
                    "电机初始化失败: %s. 节点将继续运行，但电机可能无法正常工作。", 
                    e.what());
        RCLCPP_WARN(this->get_logger(), 
                    "请检查: 1) CAN接口是否正确配置 2) 电机是否连接 3) 电机ID是否正确");
        motor_initialized_ = false;
      }
    });
    init_thread.detach(); // 分离线程，不等待完成
  }

  /**
   * @brief 定时器回调函数，读取并发布电机位置
   */
  void timer_callback() {
    if (!motor_initialized_) {
      // 如果电机未初始化，定期重试
      static int retry_counter = 0;
      if (++retry_counter >= 250) { // 每5秒重试一次（50Hz * 5s = 250）
        RCLCPP_WARN(this->get_logger(), "电机未初始化，正在重试...");
        init_motor();
        retry_counter = 0;
      }
      return;
    }

    if (!motor_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "电机对象未创建");
      return;
    }

    try {
      // 使用运控模式获取位置反馈（communication_type=0x02会更新position_）
      // kp=0, kd=0, torque=0 使电机保持透明状态，可手动移动
      // 这样会返回运控状态反馈，实时更新position_, velocity_, torque_
      motor_->send_motion_command(0.0f, motor_->position_, 0.0f, 0.0f, 0.0f);
      
      // 发布位置信息
      auto position_msg = std_msgs::msg::Float32();
      position_msg.data = motor_->position_;
      position_publisher_->publish(position_msg);

      // 发布关节状态
      auto joint_state_msg = sensor_msgs::msg::JointState();
      joint_state_msg.header.stamp = this->now();
      joint_state_msg.header.frame_id = "motor_link";
      joint_state_msg.name = {"motor_joint"};
      joint_state_msg.position = {motor_->position_};
      joint_state_msg.velocity = {motor_->velocity_};
      joint_state_msg.effort = {motor_->torque_};
      joint_state_publisher_->publish(joint_state_msg);

      // 定期打印位置信息（降低频率）
      static int counter = 0;
      if (++counter >= 50) { // 每50次打印一次（约1秒一次，如果50Hz）
        RCLCPP_INFO(this->get_logger(),
                    "位置: %.4f rad (%.2f deg), 速度: %.4f rad/s, 力矩: %.4f Nm, "
                    "温度: %.1f °C",
                    motor_->position_, motor_->position_ * 180.0 / M_PI,
                    motor_->velocity_, motor_->torque_, motor_->temperature_);
        counter = 0;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "读取电机位置时出错: %s", e.what());
      // 如果连续出错，标记为未初始化
      static int error_counter = 0;
      if (++error_counter >= 10) {
        RCLCPP_WARN(this->get_logger(), "连续读取失败，标记电机为未初始化状态");
        motor_initialized_ = false;
        error_counter = 0;
      }
    }
  }

  std::unique_ptr<RobStrideMotor> motor_;
  std::atomic<bool> motor_initialized_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * @brief 主函数
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotorPositionReaderNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
