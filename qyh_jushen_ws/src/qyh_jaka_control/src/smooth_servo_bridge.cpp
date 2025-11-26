#include "qyh_jaka_control/smooth_servo_bridge.hpp"
#include <algorithm>
#include <cmath>

namespace qyh_jaka_control
{

SmoothServoBridge::SmoothServoBridge(
    rclcpp::Logger logger,
    size_t buffer_size,
    double target_frequency_hz
)
    : logger_(logger),
      buffer_size_(buffer_size),
      target_frequency_hz_(target_frequency_hz),
      target_period_ms_(1000.0 / target_frequency_hz),
      interpolation_enabled_(true),
      interpolation_weight_(0.5),
      has_last_output_(false),
      last_command_time_(std::chrono::steady_clock::now()),
      last_output_time_(std::chrono::steady_clock::now())
{
    RCLCPP_INFO(logger_, "SmoothServoBridge initialized:");
    RCLCPP_INFO(logger_, "  Buffer size: %zu", buffer_size_);
    RCLCPP_INFO(logger_, "  Target frequency: %.1f Hz", target_frequency_hz_);
    RCLCPP_INFO(logger_, "  Target period: %.2f ms", target_period_ms_);
    RCLCPP_INFO(logger_, "  Interpolation: %s", interpolation_enabled_ ? "enabled" : "disabled");
}

bool SmoothServoBridge::addCommand(const std::vector<double>& joint_positions)
{
    if (joint_positions.size() != 7) {
        RCLCPP_ERROR_THROTTLE(logger_, *rclcpp::Clock::SharedPtr(new rclcpp::Clock()), 1000,
            "Invalid joint command size: %zu (expected 7)", joint_positions.size());
        return false;
    }
    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // 检查缓冲器是否已满
    if (command_buffer_.size() >= buffer_size_) {
        // 移除最旧的命令
        command_buffer_.pop_front();
        
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.buffer_overflow_count++;
    }
    
    // 添加新命令
    command_buffer_.emplace_back(joint_positions);
    
    last_command_time_ = std::chrono::steady_clock::now();
    
    return true;
}

bool SmoothServoBridge::getInterpolatedCommand(std::vector<double>& interpolated_positions)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (command_buffer_.empty()) {
        // 如果缓冲器为空，使用上一次的输出（保持不动）
        if (has_last_output_) {
            interpolated_positions = last_output_command_.positions;
            return true;
        }
        return false;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    // 获取最新的命令
    const JointCommand& latest_command = command_buffer_.back();
    
    if (!interpolation_enabled_ || !has_last_output_) {
        // 不插值，直接使用最新命令
        interpolated_positions = latest_command.positions;
        last_output_command_ = latest_command;
        has_last_output_ = true;
    } else {
        // 使用插值平滑
        interpolated_positions = interpolate(
            last_output_command_.positions,
            latest_command.positions,
            interpolation_weight_
        );
        
        // 更新last_output_command_
        last_output_command_.positions = interpolated_positions;
        last_output_command_.timestamp = now;
    }
    
    // 计算延迟
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
        now - latest_command.timestamp
    ).count() / 1000.0;
    
    // 更新性能统计
    updatePerformanceStats(true, latency);
    
    last_output_time_ = now;
    
    // 清理缓冲器（可选：只保留最新的几个点）
    // 这里我们保留所有点，让缓冲器自动管理
    
    return true;
}

void SmoothServoBridge::clearBuffer()
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    command_buffer_.clear();
    has_last_output_ = false;
    RCLCPP_INFO(logger_, "Command buffer cleared");
}

bool SmoothServoBridge::isEmpty() const
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return command_buffer_.empty();
}

size_t SmoothServoBridge::getBufferSize() const
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return command_buffer_.size();
}

ServoPerformanceStats SmoothServoBridge::getPerformanceStats() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void SmoothServoBridge::resetPerformanceStats()
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.reset();
    RCLCPP_INFO(logger_, "Performance stats reset");
}

void SmoothServoBridge::setInterpolationWeight(double weight)
{
    interpolation_weight_ = std::clamp(weight, 0.0, 1.0);
    RCLCPP_INFO(logger_, "Interpolation weight set to: %.2f", interpolation_weight_);
}

void SmoothServoBridge::enableInterpolation(bool enable)
{
    interpolation_enabled_ = enable;
    RCLCPP_INFO(logger_, "Interpolation %s", enable ? "enabled" : "disabled");
}

std::vector<double> SmoothServoBridge::interpolate(
    const std::vector<double>& from,
    const std::vector<double>& to,
    double weight
) const
{
    std::vector<double> result(from.size());
    
    for (size_t i = 0; i < from.size(); ++i) {
        result[i] = from[i] + weight * (to[i] - from[i]);
    }
    
    return result;
}

void SmoothServoBridge::updatePerformanceStats(bool success, double latency_ms)
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.command_count++;
    
    if (!success) {
        stats_.error_count++;
    }
    
    // 更新平均延迟（指数移动平均）
    const double alpha = 0.1;  // 平滑因子
    if (stats_.command_count == 1) {
        stats_.average_latency_ms = latency_ms;
    } else {
        stats_.average_latency_ms = alpha * latency_ms + (1.0 - alpha) * stats_.average_latency_ms;
    }
    
    // 更新平均频率
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - stats_.last_update_time
    ).count();
    
    if (duration > 1000) {  // 每秒更新一次频率
        double time_sec = duration / 1000.0;
        stats_.average_frequency = stats_.command_count / time_sec;
        
        // 重置计数器
        stats_.command_count = 0;
        stats_.last_update_time = now;
    }
}

} // namespace qyh_jaka_control
