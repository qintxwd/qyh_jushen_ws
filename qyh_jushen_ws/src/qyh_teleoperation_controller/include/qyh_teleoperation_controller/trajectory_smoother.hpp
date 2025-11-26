#ifndef QYH_TELEOPERATION_CONTROLLER__TRAJECTORY_SMOOTHER_HPP_
#define QYH_TELEOPERATION_CONTROLLER__TRAJECTORY_SMOOTHER_HPP_

#include <vector>
#include <deque>
#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace qyh_teleoperation_controller
{

struct JointState
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  rclcpp::Time timestamp;
  
  JointState() = default;
  JointState(size_t num_joints) 
    : position(num_joints, 0.0),
      velocity(num_joints, 0.0),
      acceleration(num_joints, 0.0)
  {}
};

struct SmootherLimits
{
  double max_velocity;      // rad/s
  double max_acceleration;  // rad/s²
  double max_jerk;          // rad/s³
  double low_pass_cutoff;   // Hz
  
  SmootherLimits()
    : max_velocity(1.0),
      max_acceleration(0.5),
      max_jerk(5.0),
      low_pass_cutoff(10.0)
  {}
};

class TrajectorySmoother
{
public:
  TrajectorySmoother(
    size_t num_joints,
    const SmootherLimits& limits = SmootherLimits());
  
  /**
   * @brief Smooth a target joint state
   * @param target Target joint state from IK solver
   * @param current Current joint state
   * @param dt Time step (seconds)
   * @return Smoothed joint state
   */
  JointState smoothTrajectory(
    const JointState& target,
    const JointState& current,
    double dt);
  
  /**
   * @brief Reset internal state (clear history)
   */
  void reset();
  
  /**
   * @brief Update smoother limits
   */
  void setLimits(const SmootherLimits& limits);
  
  /**
   * @brief Get current limits
   */
  SmootherLimits getLimits() const { return limits_; }

private:
  size_t num_joints_;
  SmootherLimits limits_;
  
  // History for jerk limiting
  std::deque<JointState> history_;
  static constexpr size_t MAX_HISTORY_SIZE = 5;
  
  // Low-pass filter state
  JointState filtered_state_;
  bool filter_initialized_;
  
  /**
   * @brief Clamp velocity to limits
   */
  void clampVelocity(JointState& state);
  
  /**
   * @brief Clamp acceleration based on previous state
   */
  void clampAcceleration(JointState& state, const JointState& prev, double dt);
  
  /**
   * @brief Clamp jerk based on acceleration history
   */
  void clampJerk(JointState& state, double dt);
  
  /**
   * @brief Apply low-pass filter
   */
  void applyLowPassFilter(JointState& state, double dt);
  
  /**
   * @brief Compute velocity from position difference
   */
  static std::vector<double> computeVelocity(
    const std::vector<double>& pos1,
    const std::vector<double>& pos2,
    double dt);
  
  /**
   * @brief Compute acceleration from velocity difference
   */
  static std::vector<double> computeAcceleration(
    const std::vector<double>& vel1,
    const std::vector<double>& vel2,
    double dt);
};

}  // namespace qyh_teleoperation_controller

#endif  // QYH_TELEOPERATION_CONTROLLER__TRAJECTORY_SMOOTHER_HPP_
