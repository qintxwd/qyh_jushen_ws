#include "qyh_teleoperation_controller/differential_ik_controller.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>
#include <algorithm>

namespace qyh_teleoperation_controller
{

DifferentialIKController::DifferentialIKController(
  const rclcpp::Node::SharedPtr& node,
  const moveit::core::RobotModelPtr& robot_model,
  const std::string& planning_group)
: node_(node),
  robot_model_(robot_model),
  planning_group_(planning_group)
{
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
  if (!joint_model_group_) {
    throw std::runtime_error("Invalid planning group: " + planning_group_);
  }
  
  // Get end effector link
  const auto& link_names = joint_model_group_->getLinkModelNames();
  if (!link_names.empty()) {
    end_effector_link_ = link_names.back();
  } else {
    throw std::runtime_error("No links in planning group");
  }
  
  // Initialize smoother
  size_t num_joints = joint_model_group_->getVariableCount();
  smoother_ = std::make_unique<TrajectorySmoother>(num_joints);
  
  // Initialize safety checker
  safety_checker_ = std::make_unique<SafetyChecker>(robot_model_, planning_group_);
  
  RCLCPP_INFO(node_->get_logger(), 
    "DifferentialIKController initialized for group '%s' with %zu joints",
    planning_group_.c_str(), num_joints);
}

JointState DifferentialIKController::computeJointCommand(
  const geometry_msgs::msg::PoseStamped& target_pose,
  const JointState& current_state)
{
  // Update robot state with current joint positions
  robot_state_->setJointGroupPositions(joint_model_group_, current_state.position);
  robot_state_->update();
  
  // Get current end-effector pose
  const Eigen::Isometry3d& current_ee_pose = 
    robot_state_->getGlobalLinkTransform(end_effector_link_);
  
  // Convert target pose to Eigen
  Eigen::Isometry3d target_ee_pose = poseToIsometry(target_pose.pose);
  
  // Compute pose delta (twist)
  Eigen::Matrix<double, 6, 1> pose_delta = 
    computePoseDelta(target_ee_pose, current_ee_pose);
  
  // Compute joint position delta (NOT velocities anymore)
  std::vector<double> joint_delta = computeJointVelocities(pose_delta);
  
  // Create target joint state
  JointState target_state(current_state.position.size());
  target_state.timestamp = rclcpp::Time(target_pose.header.stamp);
  
  // 直接加上关节位置差（不再乘以 dt）
  for (size_t i = 0; i < current_state.position.size(); ++i) {
    target_state.position[i] = current_state.position[i] + joint_delta[i];
    target_state.velocity[i] = joint_delta[i] / params_.dt;  // 估算速度用于平滑
  }
  
  // Apply trajectory smoothing
  JointState smoothed_state = smoother_->smoothTrajectory(
    target_state, current_state, params_.dt);
  
  // Safety check
  last_safety_check_ = safety_checker_->checkSafety(smoothed_state, current_state);
  
  if (!last_safety_check_.isSafe()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Safety check failed: returning current state");
    
    // Return current state if unsafe
    JointState safe_state = current_state;
    safe_state.velocity.assign(current_state.position.size(), 0.0);
    return safe_state;
  }
  
  return smoothed_state;
}

void DifferentialIKController::setParams(const DifferentialIKParams& params)
{
  params_ = params;
}

void DifferentialIKController::setSmootherLimits(const SmootherLimits& limits)
{
  if (smoother_) {
    smoother_->setLimits(limits);
  }
}

void DifferentialIKController::reset()
{
  if (smoother_) {
    smoother_->reset();
  }
}

Eigen::Matrix<double, 6, 1> DifferentialIKController::computePoseDelta(
  const Eigen::Isometry3d& target,
  const Eigen::Isometry3d& current)
{
  Eigen::Matrix<double, 6, 1> delta;
  
  // Position delta
  delta.head<3>() = target.translation() - current.translation();
  
  // Orientation delta (axis-angle representation)
  Eigen::Quaterniond q_target(target.rotation());
  Eigen::Quaterniond q_current(current.rotation());
  Eigen::Quaterniond q_delta = q_current.inverse() * q_target;
  
  // Convert quaternion to axis-angle
  Eigen::AngleAxisd aa_delta(q_delta);
  delta.tail<3>() = aa_delta.angle() * aa_delta.axis();
  
  return delta;
}

std::vector<double> DifferentialIKController::computeJointVelocities(
  const Eigen::Matrix<double, 6, 1>& pose_delta)
{
  // Get Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  
  if (!robot_state_->getJacobian(joint_model_group_,
                                  robot_state_->getLinkModel(end_effector_link_),
                                  reference_point_position,
                                  jacobian)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to compute Jacobian");
    return std::vector<double>(joint_model_group_->getVariableCount(), 0.0);
  }
  
  // 直接使用位置差（不除以dt转换为速度）
  // 这样关节变化量 = J^+ * pose_delta，而不是 J^+ * velocity * dt
  Eigen::Matrix<double, 6, 1> scaled_delta = pose_delta;
  
  // 限制位置误差幅度，防止大跳变
  const double max_pos_delta = 0.01;  // 10mm
  const double max_rot_delta = 0.05;  // ~3度
  
  for (int i = 0; i < 3; ++i) {
    scaled_delta(i) = std::clamp(scaled_delta(i), -max_pos_delta, max_pos_delta);
  }
  for (int i = 3; i < 6; ++i) {
    scaled_delta(i) = std::clamp(scaled_delta(i), -max_rot_delta, max_rot_delta);
  }
  
  // Damped Least Squares (DLS) solution: dq = J^T * (J*J^T + lambda^2*I)^-1 * dx
  // 使用更大的阻尼因子以提高稳定性
  size_t m = jacobian.rows();  // 6 (task space dimension)
  size_t n = jacobian.cols();  // num_joints
  
  // 增大阻尼因子，防止奇异点附近的大跳变
  double effective_damping = std::max(params_.damping_factor, 0.1);
  
  Eigen::MatrixXd J_JT = jacobian * jacobian.transpose();
  Eigen::MatrixXd damping = effective_damping * effective_damping * 
                            Eigen::MatrixXd::Identity(m, m);
  
  Eigen::MatrixXd JJT_damped = J_JT + damping;
  
  // 解出关节位置变化量（不是速度）
  Eigen::VectorXd joint_delta = 
    jacobian.transpose() * JJT_damped.ldlt().solve(scaled_delta);
  
  // 限制每个关节的最大变化量
  const double max_joint_delta = 0.05;  // ~3度
  for (size_t i = 0; i < n; ++i) {
    joint_delta(i) = std::clamp(joint_delta(i), -max_joint_delta, max_joint_delta);
  }
  
  // 返回关节变化量（作为"速度"返回，但实际是位置差）
  std::vector<double> joint_velocities(n);
  for (size_t i = 0; i < n; ++i) {
    joint_velocities[i] = joint_delta(i);
  }
  
  return joint_velocities;
}

Eigen::Isometry3d DifferentialIKController::poseToIsometry(
  const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  
  // Translation
  isometry.translation() = Eigen::Vector3d(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  
  // Rotation (from quaternion)
  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  isometry.linear() = q.toRotationMatrix();
  
  return isometry;
}

}  // namespace qyh_teleoperation_controller
