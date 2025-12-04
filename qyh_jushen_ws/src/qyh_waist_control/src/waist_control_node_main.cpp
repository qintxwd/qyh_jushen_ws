/*
 * 腰部电机控制节点入口
 */

#include <rclcpp/rclcpp.hpp>
#include "qyh_waist_control/waist_control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<qyh_waist_control::WaistControlNode>();

  RCLCPP_INFO(node->get_logger(), "Waist Control Node is running...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
