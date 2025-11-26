#include "qyh_teleoperation_controller/virtual_arm_follower.hpp"

namespace qyh_teleoperation_controller
{

VirtualArmFollower::VirtualArmFollower(
  const rclcpp::Node::SharedPtr& node,
  const std::string& arm_name)
: node_(node),
  arm_name_(arm_name)
{
  // Create publisher
  std::string topic_name = "/" + arm_name + "/virtual_joint_states";
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    topic_name, 10);
  
  // Initialize joint names (assuming 7 DOF arm)
  for (int i = 1; i <= 7; ++i) {
    joint_names_.push_back(arm_name_ + "_joint" + std::to_string(i));
  }
  
  current_state_ = JointState(joint_names_.size());
  
  RCLCPP_INFO(node_->get_logger(), 
    "VirtualArmFollower initialized for '%s'", arm_name_.c_str());
}

void VirtualArmFollower::updateState(const JointState& state)
{
  current_state_ = state;
  
  // Publish joint state
  auto msg = toJointStateMsg(state);
  joint_state_pub_->publish(msg);
}

sensor_msgs::msg::JointState VirtualArmFollower::toJointStateMsg(
  const JointState& state)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = state.timestamp;
  msg.name = joint_names_;
  msg.position = state.position;
  msg.velocity = state.velocity;
  
  return msg;
}

}  // namespace qyh_teleoperation_controller
