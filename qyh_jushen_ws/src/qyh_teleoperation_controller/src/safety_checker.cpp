#include "qyh_teleoperation_controller/safety_checker.hpp"
#include <cmath>
#include <sstream>

namespace qyh_teleoperation_controller
{

SafetyChecker::SafetyChecker(
  const moveit::core::RobotModelPtr& robot_model,
  const std::string& planning_group)
: robot_model_(robot_model),
  planning_group_(planning_group),
  joint_limit_margin_(0.05),      // 0.05 rad margin
  velocity_limit_scale_(0.8),     // Use 80% of max velocity
  collision_check_distance_(0.02), // 2cm
  singularity_threshold_(0.05)
{
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
  
  if (!joint_model_group_) {
    throw std::runtime_error("Invalid planning group: " + planning_group_);
  }
}

SafetyCheckResult SafetyChecker::checkSafety(
  const JointState& planned_state,
  const JointState& current_state)
{
  SafetyCheckResult result;
  
  // Check joint limits
  if (!checkJointLimits(planned_state.position, result.errors)) {
    result.status = SafetyStatus::JOINT_LIMIT_VIOLATION;
    return result;
  }
  
  // Check velocity limits
  if (!planned_state.velocity.empty() && 
      !checkVelocityLimits(planned_state.velocity, result.errors)) {
    result.status = SafetyStatus::VELOCITY_LIMIT_VIOLATION;
    return result;
  }
  
  // Check collisions if planning scene is available
  if (planning_scene_ && 
      !checkCollisions(planned_state.position, result.min_collision_distance)) {
    result.status = SafetyStatus::COLLISION_DETECTED;
    result.errors.push_back("Collision detected");
    return result;
  }
  
  // Check singularities
  if (!checkSingularity(planned_state.position, result.distance_to_singularity)) {
    result.status = SafetyStatus::SINGULARITY_NEAR;
    result.warnings.push_back("Near singularity");
    // Don't return here - singularity is a warning, not an error
  }
  
  result.status = SafetyStatus::SAFE;
  return result;
}

bool SafetyChecker::checkJointLimits(
  const std::vector<double>& joint_positions,
  std::vector<std::string>& errors)
{
  if (joint_positions.size() != joint_model_group_->getVariableCount()) {
    errors.push_back("Invalid joint position vector size");
    return false;
  }
  
  const auto& joint_names = joint_model_group_->getVariableNames();
  bool within_limits = true;
  
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    const auto* joint_model = robot_model_->getJointModel(joint_names[i]);
    if (!joint_model) continue;
    
    const auto& bounds = joint_model->getVariableBounds(joint_names[i]);
    
    if (bounds.position_bounded_) {
      double lower = bounds.min_position_ + joint_limit_margin_;
      double upper = bounds.max_position_ - joint_limit_margin_;
      
      if (joint_positions[i] < lower || joint_positions[i] > upper) {
        std::stringstream ss;
        ss << "Joint " << joint_names[i] << " out of bounds: "
           << joint_positions[i] << " (limits: [" << lower << ", " << upper << "])";
        errors.push_back(ss.str());
        within_limits = false;
      }
    }
  }
  
  return within_limits;
}

bool SafetyChecker::checkVelocityLimits(
  const std::vector<double>& joint_velocities,
  std::vector<std::string>& errors)
{
  if (joint_velocities.size() != joint_model_group_->getVariableCount()) {
    errors.push_back("Invalid joint velocity vector size");
    return false;
  }
  
  const auto& joint_names = joint_model_group_->getVariableNames();
  bool within_limits = true;
  
  for (size_t i = 0; i < joint_velocities.size(); ++i) {
    const auto* joint_model = robot_model_->getJointModel(joint_names[i]);
    if (!joint_model) continue;
    
    const auto& bounds = joint_model->getVariableBounds(joint_names[i]);
    
    if (bounds.velocity_bounded_) {
      double max_vel = bounds.max_velocity_ * velocity_limit_scale_;
      
      if (std::abs(joint_velocities[i]) > max_vel) {
        std::stringstream ss;
        ss << "Joint " << joint_names[i] << " velocity exceeded: "
           << joint_velocities[i] << " (limit: " << max_vel << ")";
        errors.push_back(ss.str());
        within_limits = false;
      }
    }
  }
  
  return within_limits;
}

bool SafetyChecker::checkCollisions(
  const std::vector<double>& joint_positions,
  double& min_distance)
{
  if (!planning_scene_) {
    min_distance = 1.0;
    return true;  // No collision checking without planning scene
  }
  
  // Set robot state
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  robot_state_->update();
  
  // Check for collisions
  collision_detection::CollisionRequest collision_request;
  collision_request.distance = true;
  collision_request.group_name = planning_group_;
  
  collision_detection::CollisionResult collision_result;
  planning_scene_->checkCollision(collision_request, collision_result, *robot_state_);
  
  if (collision_result.distance > 0.0) {
    min_distance = collision_result.distance;
  } else {
    min_distance = 0.0;
  }
  
  // Return false if in collision or too close
  return !collision_result.collision && 
         collision_result.distance > collision_check_distance_;
}

bool SafetyChecker::checkSingularity(
  const std::vector<double>& joint_positions,
  double& distance)
{
  distance = computeManipulability(joint_positions);
  return distance > singularity_threshold_;
}

void SafetyChecker::setPlanningScene(
  const planning_scene::PlanningScenePtr& planning_scene)
{
  planning_scene_ = planning_scene;
}

double SafetyChecker::computeManipulability(
  const std::vector<double>& joint_positions)
{
  // Set robot state
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  robot_state_->update();
  
  // Get Jacobian
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  
  if (!robot_state_->getJacobian(joint_model_group_, 
                                  robot_state_->getLinkModel(
                                    joint_model_group_->getLinkModelNames().back()),
                                  reference_point_position,
                                  jacobian)) {
    return 0.0;
  }
  
  // Compute manipulability measure: sqrt(det(J * J^T))
  // For simplicity, use the minimum singular value as a proxy
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  auto singular_values = svd.singularValues();
  
  if (singular_values.size() > 0) {
    return singular_values.minCoeff();
  }
  
  return 0.0;
}

}  // namespace qyh_teleoperation_controller
