#ifndef QYH_TELEOPERATION_CONTROLLER__DIFFERENTIAL_IK_CONTROLLER_HPP_
#define QYH_TELEOPERATION_CONTROLLER__DIFFERENTIAL_IK_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "qyh_teleoperation_controller/trajectory_smoother.hpp"
#include "qyh_teleoperation_controller/safety_checker.hpp"

namespace qyh_teleoperation_controller
{

struct DifferentialIKParams
{
  double position_tolerance;     // m
  double orientation_tolerance;  // rad
  double damping_factor;         // For damped least squares
  double max_iterations;
  double dt;                     // Control period (seconds)
  
  DifferentialIKParams()
    : position_tolerance(0.001),
      orientation_tolerance(0.01),
      damping_factor(0.01),
      max_iterations(50),
      dt(0.008)  // 125Hz
  {}
};

class DifferentialIKController
{
public:
  DifferentialIKController(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModelPtr& robot_model,
    const std::string& planning_group);
  
  /**
   * @brief Compute joint velocities to reach target pose
   * @param target_pose Target end-effector pose
   * @param current_state Current joint state
   * @return Smoothed and safe joint command
   */
  JointState computeJointCommand(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const JointState& current_state);
  
  /**
   * @brief Set parameters
   */
  void setParams(const DifferentialIKParams& params);
  DifferentialIKParams getParams() const { return params_; }
  
  /**
   * @brief Set smoother limits
   */
  void setSmootherLimits(const SmootherLimits& limits);
  
  /**
   * @brief Reset controller state
   */
  void reset();
  
  /**
   * @brief Get last safety check result
   */
  SafetyCheckResult getLastSafetyCheck() const { return last_safety_check_; }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  
  std::string planning_group_;
  const moveit::core::JointModelGroup* joint_model_group_;
  std::string end_effector_link_;
  
  DifferentialIKParams params_;
  
  std::unique_ptr<TrajectorySmoother> smoother_;
  std::unique_ptr<SafetyChecker> safety_checker_;
  
  SafetyCheckResult last_safety_check_;
  
  /**
   * @brief Compute pose delta (twist)
   */
  Eigen::Matrix<double, 6, 1> computePoseDelta(
    const Eigen::Isometry3d& target,
    const Eigen::Isometry3d& current);
  
  /**
   * @brief Compute joint velocities using damped least squares
   */
  std::vector<double> computeJointVelocities(
    const Eigen::Matrix<double, 6, 1>& pose_delta);
  
  /**
   * @brief Convert geometry_msgs Pose to Eigen Isometry3d
   */
  static Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose);
};

}  // namespace qyh_teleoperation_controller

#endif  // QYH_TELEOPERATION_CONTROLLER__DIFFERENTIAL_IK_CONTROLLER_HPP_
