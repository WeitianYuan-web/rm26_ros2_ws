/**
 * @file gimbal_control_node.cpp
 * @brief 云台控制节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class GimbalControlNode
 * @brief 云台控制节点类，负责云台 Pitch/Yaw 轴运动控制
 */
class GimbalControlNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  GimbalControlNode() : Node("gimbal_control_node")
  {
    RCLCPP_INFO(this->get_logger(), "GimbalControlNode 已启动");
  }

private:
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalControlNode>());
  rclcpp::shutdown();
  return 0;
}
