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
#include <mutex>

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
  int left_slave_id_;
  int right_slave_id_;
  bool is_connected_;
  bool auto_activate_;

  // Gripper state
  qyh_gripper_msgs::msg::GripperState left_current_state_;
  qyh_gripper_msgs::msg::GripperState right_current_state_;
  bool left_is_activated_;
  bool right_is_activated_;

  // ROS2 interfaces
  rclcpp::Publisher<qyh_gripper_msgs::msg::GripperState>::SharedPtr left_state_pub_;
  rclcpp::Publisher<qyh_gripper_msgs::msg::GripperState>::SharedPtr right_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Service<qyh_gripper_msgs::srv::ActivateGripper>::SharedPtr left_srv_activate_;
  rclcpp::Service<qyh_gripper_msgs::srv::ActivateGripper>::SharedPtr right_srv_activate_;
  rclcpp::Service<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr left_srv_move_;
  rclcpp::Service<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr right_srv_move_;
  rclcpp::Service<qyh_gripper_msgs::srv::GetGripperState>::SharedPtr left_srv_get_state_;
  rclcpp::Service<qyh_gripper_msgs::srv::GetGripperState>::SharedPtr right_srv_get_state_;

  // Methods
  bool connect_modbus();
  void disconnect_modbus();
  void timer_callback();
  bool read_gripper_state(bool left);
  bool activate_gripper(bool left = true);
  bool move_gripper(bool left,uint8_t position, uint8_t speed, uint8_t force);
  
  // Service callbacks
  void handle_activate_left(
    const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response);

  void handle_activate_right(
    const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response);
    
  void handle_move_left(
    const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response);
  
  void handle_move_right(
    const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
    qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response);
    
  void handle_get_state_left(
    const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr request,
    qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response);

  void handle_get_state_right(
    const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr request,
    qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response);

  std::string get_fault_message(uint8_t fault_code);

  // Protects access to modbus_ctx_
  std::mutex modbus_mutex_;

};

}  // namespace qyh_gripper_control

#endif  // QYH_GRIPPER_CONTROL__GRIPPER_CONTROL_NODE_HPP_
