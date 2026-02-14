/**
 * @file auto_aim_node.cpp
 * @brief 自动瞄准节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class AutoAimNode
 * @brief 自动瞄准节点类，负责目标检测与自动瞄准
 */
class AutoAimNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  AutoAimNode() : Node("auto_aim_node")
  {
    RCLCPP_INFO(this->get_logger(), "AutoAimNode 已启动");
  }

private:
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoAimNode>());
  rclcpp::shutdown();
  return 0;
}
