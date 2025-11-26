#ifndef QYH_TELEOPERATION_CONTROLLER__SAFETY_CHECKER_HPP_
#define QYH_TELEOPERATION_CONTROLLER__SAFETY_CHECKER_HPP_

#include <vector>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"
#include "qyh_teleoperation_controller/trajectory_smoother.hpp"

namespace qyh_teleoperation_controller
{

enum class SafetyStatus
{
  SAFE,
  JOINT_LIMIT_VIOLATION,
  VELOCITY_LIMIT_VIOLATION,
  COLLISION_DETECTED,
  SINGULARITY_NEAR,
  UNKNOWN_ERROR
};

struct SafetyCheckResult
{
  SafetyStatus status;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  double distance_to_singularity;
  double min_collision_distance;
  
  SafetyCheckResult()
    : status(SafetyStatus::SAFE),
      distance_to_singularity(1.0),
      min_collision_distance(1.0)
  {}
  
  bool isSafe() const {
    return status == SafetyStatus::SAFE;
  }
};

class SafetyChecker
{
public:
  SafetyChecker(
    const moveit::core::RobotModelPtr& robot_model,
    const std::string& planning_group);
  
  /**
   * @brief Check if planned state is safe
   * @param planned_state Planned joint state
   * @param current_state Current robot state
   * @return Safety check result
   */
  SafetyCheckResult checkSafety(
    const JointState& planned_state,
    const JointState& current_state);
  
  /**
   * @brief Check joint limits
   */
  bool checkJointLimits(
    const std::vector<double>& joint_positions,
    std::vector<std::string>& errors);
  
  /**
   * @brief Check velocity limits
   */
  bool checkVelocityLimits(
    const std::vector<double>& joint_velocities,
    std::vector<std::string>& errors);
  
  /**
   * @brief Check for collisions
   */
  bool checkCollisions(
    const std::vector<double>& joint_positions,
    double& min_distance);
  
  /**
   * @brief Check proximity to singularities
   */
  bool checkSingularity(
    const std::vector<double>& joint_positions,
    double& distance);
  
  /**
   * @brief Set planning scene for collision checking
   */
  void setPlanningScene(const planning_scene::PlanningScenePtr& planning_scene);
  
  /**
   * @brief Set safety margins
   */
  void setJointLimitMargin(double margin) { joint_limit_margin_ = margin; }
  void setVelocityLimitScale(double scale) { velocity_limit_scale_ = scale; }
  void setCollisionDistance(double distance) { collision_check_distance_ = distance; }
  void setSingularityThreshold(double threshold) { singularity_threshold_ = threshold; }

private:
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  
  std::string planning_group_;
  const moveit::core::JointModelGroup* joint_model_group_;
  
  // Safety parameters
  double joint_limit_margin_;        // rad
  double velocity_limit_scale_;      // 0.0-1.0
  double collision_check_distance_;  // m
  double singularity_threshold_;     // Manipulability threshold
  
  /**
   * @brief Compute manipulability measure
   */
  double computeManipulability(const std::vector<double>& joint_positions);
};

}  // namespace qyh_teleoperation_controller

#endif  // QYH_TELEOPERATION_CONTROLLER__SAFETY_CHECKER_HPP_
