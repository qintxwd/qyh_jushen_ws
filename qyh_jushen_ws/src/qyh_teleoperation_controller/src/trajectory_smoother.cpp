#include "qyh_teleoperation_controller/trajectory_smoother.hpp"
#include <algorithm>
#include <cmath>

namespace qyh_teleoperation_controller
{

TrajectorySmoother::TrajectorySmoother(
  size_t num_joints,
  const SmootherLimits& limits)
: num_joints_(num_joints),
  limits_(limits),
  filtered_state_(num_joints),
  filter_initialized_(false)
{
}

JointState TrajectorySmoother::smoothTrajectory(
  const JointState& target,
  const JointState& current,
  double dt)
{
  if (dt <= 0.0) {
    return current;
  }
  
  // Start with target state
  JointState smoothed = target;
  smoothed.timestamp = target.timestamp;
  
  // Compute velocity if not provided
  if (smoothed.velocity.empty() || 
      std::all_of(smoothed.velocity.begin(), smoothed.velocity.end(), 
                  [](double v) { return v == 0.0; })) {
    smoothed.velocity = computeVelocity(target.position, current.position, dt);
  }
  
  // 1. Velocity limiting
  clampVelocity(smoothed);
  
  // 2. Acceleration limiting
  if (!history_.empty()) {
    clampAcceleration(smoothed, history_.back(), dt);
  }
  
  // 3. Jerk limiting
  if (history_.size() >= 2) {
    clampJerk(smoothed, dt);
  }
  
  // 4. Low-pass filtering
  applyLowPassFilter(smoothed, dt);
  
  // Add to history
  history_.push_back(smoothed);
  if (history_.size() > MAX_HISTORY_SIZE) {
    history_.pop_front();
  }
  
  return smoothed;
}

void TrajectorySmoother::reset()
{
  history_.clear();
  filter_initialized_ = false;
  filtered_state_ = JointState(num_joints_);
}

void TrajectorySmoother::setLimits(const SmootherLimits& limits)
{
  limits_ = limits;
}

void TrajectorySmoother::clampVelocity(JointState& state)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    if (i < state.velocity.size()) {
      state.velocity[i] = std::clamp(
        state.velocity[i],
        -limits_.max_velocity,
        limits_.max_velocity);
    }
  }
}

void TrajectorySmoother::clampAcceleration(
  JointState& state,
  const JointState& prev,
  double dt)
{
  if (state.velocity.size() != prev.velocity.size()) {
    return;
  }
  
  for (size_t i = 0; i < num_joints_; ++i) {
    if (i < state.velocity.size() && i < prev.velocity.size()) {
      double desired_acc = (state.velocity[i] - prev.velocity[i]) / dt;
      double clamped_acc = std::clamp(
        desired_acc,
        -limits_.max_acceleration,
        limits_.max_acceleration);
      
      // Apply clamped acceleration
      state.velocity[i] = prev.velocity[i] + clamped_acc * dt;
      
      // Store acceleration
      if (i >= state.acceleration.size()) {
        state.acceleration.resize(num_joints_, 0.0);
      }
      state.acceleration[i] = clamped_acc;
    }
  }
}

void TrajectorySmoother::clampJerk(JointState& state, double dt)
{
  if (history_.size() < 2) {
    return;
  }
  
  const auto& prev1 = history_[history_.size() - 1];
  const auto& prev2 = history_[history_.size() - 2];
  
  for (size_t i = 0; i < num_joints_; ++i) {
    if (i < state.acceleration.size() && 
        i < prev1.acceleration.size() && 
        i < prev2.acceleration.size()) {
      
      double desired_jerk = (state.acceleration[i] - prev1.acceleration[i]) / dt;
      double clamped_jerk = std::clamp(
        desired_jerk,
        -limits_.max_jerk,
        limits_.max_jerk);
      
      // Apply clamped jerk
      state.acceleration[i] = prev1.acceleration[i] + clamped_jerk * dt;
      
      // Recalculate velocity from clamped acceleration
      if (i < state.velocity.size() && i < prev1.velocity.size()) {
        state.velocity[i] = prev1.velocity[i] + state.acceleration[i] * dt;
      }
    }
  }
}

void TrajectorySmoother::applyLowPassFilter(JointState& state, double dt)
{
  if (!filter_initialized_) {
    filtered_state_ = state;
    filter_initialized_ = true;
    return;
  }
  
  // First-order low-pass filter: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
  // alpha = dt / (dt + RC), where RC = 1 / (2 * pi * cutoff_freq)
  double RC = 1.0 / (2.0 * M_PI * limits_.low_pass_cutoff);
  double alpha = dt / (dt + RC);
  
  for (size_t i = 0; i < num_joints_; ++i) {
    // Filter position
    if (i < state.position.size() && i < filtered_state_.position.size()) {
      filtered_state_.position[i] = 
        alpha * state.position[i] + (1.0 - alpha) * filtered_state_.position[i];
      state.position[i] = filtered_state_.position[i];
    }
    
    // Filter velocity
    if (i < state.velocity.size() && i < filtered_state_.velocity.size()) {
      filtered_state_.velocity[i] = 
        alpha * state.velocity[i] + (1.0 - alpha) * filtered_state_.velocity[i];
      state.velocity[i] = filtered_state_.velocity[i];
    }
  }
}

std::vector<double> TrajectorySmoother::computeVelocity(
  const std::vector<double>& pos1,
  const std::vector<double>& pos2,
  double dt)
{
  std::vector<double> velocity(pos1.size(), 0.0);
  
  if (dt > 0.0 && pos1.size() == pos2.size()) {
    for (size_t i = 0; i < pos1.size(); ++i) {
      velocity[i] = (pos1[i] - pos2[i]) / dt;
    }
  }
  
  return velocity;
}

std::vector<double> TrajectorySmoother::computeAcceleration(
  const std::vector<double>& vel1,
  const std::vector<double>& vel2,
  double dt)
{
  std::vector<double> acceleration(vel1.size(), 0.0);
  
  if (dt > 0.0 && vel1.size() == vel2.size()) {
    for (size_t i = 0; i < vel1.size(); ++i) {
      acceleration[i] = (vel1[i] - vel2[i]) / dt;
    }
  }
  
  return acceleration;
}

}  // namespace qyh_teleoperation_controller
