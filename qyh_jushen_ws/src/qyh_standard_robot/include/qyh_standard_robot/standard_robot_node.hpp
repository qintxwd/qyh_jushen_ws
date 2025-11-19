#ifndef QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
#define QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qyh_standard_robot_msgs/msg/standard_robot_status.hpp>
#include <modbus/modbus.hpp>

namespace qyh_standard_robot
{

class StandardRobotNode : public rclcpp::Node
{
public:
  explicit StandardRobotNode(const rclcpp::NodeOptions & options);
  virtual ~StandardRobotNode();

private:
  void timer_callback();
  bool connect_modbus();
  void disconnect_modbus();
  bool read_robot_status();

  // ROS2 members
  rclcpp::Publisher<qyh_standard_robot_msgs::msg::StandardRobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  qyh_standard_robot_msgs::msg::StandardRobotStatus status_msg_;

  // Modbus members - use C++ API
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  bool is_connected_;

  // Parameters
  std::string modbus_ip_;
  int modbus_port_;
  int slave_id_;
  double publish_rate_;
};

}  // namespace qyh_standard_robot

#endif  // QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
