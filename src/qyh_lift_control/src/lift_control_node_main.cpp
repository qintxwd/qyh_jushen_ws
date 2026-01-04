/*
 * 升降电机控制节点入口
 */

#include <rclcpp/rclcpp.hpp>
#include "qyh_lift_control/lift_control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<qyh_lift_control::LiftControlNode>();

  RCLCPP_INFO(node->get_logger(), "Lift Control Node is running...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
