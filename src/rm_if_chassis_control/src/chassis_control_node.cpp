/**
 * @file chassis_control_node.cpp
 * @brief 底盘控制节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class ChassisControlNode
 * @brief 底盘控制节点类，负责底盘运动控制
 */
class ChassisControlNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  ChassisControlNode() : Node("chassis_control_node")
  {
    RCLCPP_INFO(this->get_logger(), "ChassisControlNode 已启动");
  }

private:
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisControlNode>());
  rclcpp::shutdown();
  return 0;
}
