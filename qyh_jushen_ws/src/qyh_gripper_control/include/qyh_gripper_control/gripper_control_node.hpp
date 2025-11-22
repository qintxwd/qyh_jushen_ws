#ifndef QYH_GRIPPER_CONTROL__GRIPPER_CONTROL_NODE_HPP_
#define QYH_GRIPPER_CONTROL__GRIPPER_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <qyh_gripper_msgs/msg/gripper_state.hpp>
#include <qyh_gripper_msgs/srv/activate_gripper.hpp>
#include <qyh_gripper_msgs/srv/move_gripper.hpp>
#include <qyh_gripper_msgs/srv/get_gripper_state.hpp>
#include <modbus/modbus.hpp>  // 使用C++封装
#include <memory>
#include <string>

namespace qyh_gripper_control
{

class GripperControlNode : public rclcpp::Node
{
public:
  explicit GripperControlNode(const rclcpp::NodeOptions & options);
  ~GripperControlNode();

private:
  // Modbus communication - 使用C++封装
  std::unique_ptr<modbus::ModbusRTU> modbus_ctx_;
  std::string device_port_;
  int baudrate_;
  int slave_id_;
  bool is_connected_;

  // Gripper state
  qyh_gripper_msgs::msg::GripperState current_state_;
  bool is_activated_;

  // ROS2 interfaces
  rclcpp::Publisher<qyh_gripper_msgs::msg::GripperState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Service<qyh_gripper_msgs::srv::ActivateGripper>::SharedPtr srv_activate_;
  rclcpp::Service<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr srv_move_;
  rclcpp::Service<qyh_gripper_msgs::srv::GetGripperState>::SharedPtr srv_get_state_;

  // Methods
  bool connect_modbus();
  void disconnect_modbus();
  void timer_callback();
  bool read_gripper_state();
  bool activate_gripper();
  bool move_gripper(uint8_t position, uint8_t speed, uint8_t force);
  
  // Service callbacks
  void handle_activate(
    const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response);
    
  void handle_move(
    const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response);
    
  void handle_get_state(
    const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr request,
    qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response);

  std::string get_fault_message(uint8_t fault_code);
};

}  // namespace qyh_gripper_control

#endif  // QYH_GRIPPER_CONTROL__GRIPPER_CONTROL_NODE_HPP_
