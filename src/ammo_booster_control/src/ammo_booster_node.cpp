/**
 * @file ammo_booster_node.cpp
 * @brief 弹仓拨弹控制节点
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class AmmoBoosterNode
 * @brief 弹仓拨弹控制节点类，负责弹仓与拨弹机构控制
 */
class AmmoBoosterNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  AmmoBoosterNode() : Node("ammo_booster_node")
  {
    RCLCPP_INFO(this->get_logger(), "AmmoBoosterNode 已启动");
  }

private:
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmmoBoosterNode>());
  rclcpp::shutdown();
  return 0;
}
