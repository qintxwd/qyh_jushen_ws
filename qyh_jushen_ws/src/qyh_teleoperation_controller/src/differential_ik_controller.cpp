#include "qyh_teleoperation_controller/differential_ik_controller.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>

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
  
  // Compute joint velocities
  std::vector<double> joint_velocities = computeJointVelocities(pose_delta);
  
  // Create target joint state
  JointState target_state(current_state.position.size());
  target_state.timestamp = rclcpp::Time(target_pose.header.stamp);
  
  // Integrate velocities to get target positions
  for (size_t i = 0; i < current_state.position.size(); ++i) {
    target_state.position[i] = current_state.position[i] + 
                                joint_velocities[i] * params_.dt;
    target_state.velocity[i] = joint_velocities[i];
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
  
  // Compute desired end-effector velocity (twist)
  Eigen::Matrix<double, 6, 1> ee_velocity = pose_delta / params_.dt;
  
  // Damped Least Squares (DLS) solution: dq = J^T * (J*J^T + lambda^2*I)^-1 * dx
  // This is more stable near singularities than pure pseudoinverse
  size_t m = jacobian.rows();  // 6 (task space dimension)
  size_t n = jacobian.cols();  // num_joints
  
  Eigen::MatrixXd J_JT = jacobian * jacobian.transpose();
  Eigen::MatrixXd damping = params_.damping_factor * params_.damping_factor * 
                            Eigen::MatrixXd::Identity(m, m);
  
  Eigen::MatrixXd JJT_damped = J_JT + damping;
  
  // Solve for joint velocities
  Eigen::VectorXd joint_velocities_eigen = 
    jacobian.transpose() * JJT_damped.ldlt().solve(ee_velocity);
  
  // Convert to std::vector
  std::vector<double> joint_velocities(n);
  for (size_t i = 0; i < n; ++i) {
    joint_velocities[i] = joint_velocities_eigen(i);
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
