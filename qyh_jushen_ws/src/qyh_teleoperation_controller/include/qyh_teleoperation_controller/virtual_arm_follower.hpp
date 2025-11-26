#ifndef QYH_TELEOPERATION_CONTROLLER__VIRTUAL_ARM_FOLLOWER_HPP_
#define QYH_TELEOPERATION_CONTROLLER__VIRTUAL_ARM_FOLLOWER_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "qyh_teleoperation_controller/trajectory_smoother.hpp"

namespace qyh_teleoperation_controller
{

/**
 * @brief Virtual arm follower - publishes joint states for visualization
 */
class VirtualArmFollower
{
public:
  VirtualArmFollower(
    const rclcpp::Node::SharedPtr& node,
    const std::string& arm_name);
  
  /**
   * @brief Update virtual arm with new joint state
   */
  void updateState(const JointState& state);
  
  /**
   * @brief Get current virtual arm state
   */
  JointState getCurrentState() const { return current_state_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string arm_name_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  
  JointState current_state_;
  std::vector<std::string> joint_names_;
  
  /**
   * @brief Convert JointState to sensor_msgs::JointState
   */
  sensor_msgs::msg::JointState toJointStateMsg(const JointState& state);
};

}  // namespace qyh_teleoperation_controller

#endif  // QYH_TELEOPERATION_CONTROLLER__VIRTUAL_ARM_FOLLOWER_HPP_
