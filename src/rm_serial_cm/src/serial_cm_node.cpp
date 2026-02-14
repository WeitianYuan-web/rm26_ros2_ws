/**
 * @file serial_cm_node.cpp
 * @brief 串口通信节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class SerialCmNode
 * @brief 串口通信节点类，负责与下位机的串口数据收发
 */
class SerialCmNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  SerialCmNode() : Node("serial_cm_node")
  {
    RCLCPP_INFO(this->get_logger(), "SerialCmNode 已启动");
  }

private:
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialCmNode>());
  rclcpp::shutdown();
  return 0;
}
