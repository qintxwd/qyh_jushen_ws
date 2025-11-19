#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "qyh_standard_robot/standard_robot_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<qyh_standard_robot::StandardRobotNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
