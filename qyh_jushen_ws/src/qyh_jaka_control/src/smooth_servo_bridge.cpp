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
      command_timeout_sec_(0.5),  // 默认500ms无指令则失效
      stale_threshold_sec_(0.3),  // 200ms后缓存过期，适应手柄30Hz输入
      velocity_limits_(7, 2.0),  // 默认2.0 rad/s
      velocity_safety_factor_(0.8),  // 默认80%安全速度
      cycle_time_sec_(1.0 / target_frequency_hz),
      last_command_time_(std::chrono::steady_clock::now()),
      last_output_time_(std::chrono::steady_clock::now())
{
    RCLCPP_INFO(logger_, "SmoothServoBridge initialized:");
    RCLCPP_INFO(logger_, "  Buffer size: %zu", buffer_size_);
    RCLCPP_INFO(logger_, "  Target frequency: %.1f Hz", target_frequency_hz_);
    RCLCPP_INFO(logger_, "  Target period: %.2f ms", target_period_ms_);
    RCLCPP_INFO(logger_, "  Cycle time: %.1f ms", cycle_time_sec_ * 1000.0);
    RCLCPP_INFO(logger_, "  Interpolation: %s", interpolation_enabled_ ? "enabled" : "disabled");
    RCLCPP_INFO(logger_, "  Velocity safety factor: %.1f%%", velocity_safety_factor_ * 100.0);
    RCLCPP_INFO(logger_, "  Command timeout: %.1f sec (auto re-sync on idle)", command_timeout_sec_);
}

bool SmoothServoBridge::addCommand(const std::vector<double>& joint_positions,
                                    const std::vector<double>& current_position)
{
    if (joint_positions.size() != 7) {
        RCLCPP_ERROR(logger_,
            "Invalid joint command size: %zu (expected 7)", joint_positions.size());
        return false;
    }
    
    if (!current_position.empty() && current_position.size() != 7) {
        RCLCPP_ERROR(logger_,
            "Invalid current position size: %zu (expected 7)", current_position.size());
        return false;
    }

    //输出一下日志，显示添加的命令和当前位置，以及他们之间的差异
    RCLCPP_INFO(logger_,
        "[Bridge] Adding command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        joint_positions[0], joint_positions[1], joint_positions[2],
        joint_positions[3], joint_positions[4], joint_positions[5],
        joint_positions[6]);
    if (!current_position.empty()) {
        RCLCPP_INFO(logger_,
            "[Bridge] Current position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            current_position[0], current_position[1], current_position[2],
            current_position[3], current_position[4], current_position[5],
            current_position[6]);
        RCLCPP_INFO(logger_,
            "[Bridge] Position delta: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            joint_positions[0] - current_position[0], joint_positions[1] - current_position[1],
            joint_positions[2] - current_position[2], joint_positions[3] - current_position[3],
            joint_positions[4] - current_position[4], joint_positions[5] - current_position[5],
            joint_positions[6] - current_position[6]);
    }

    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto now = std::chrono::steady_clock::now();
    
    // 智能选择起始位置
    std::vector<double> from_position;
    bool using_current = false;
    
    if (!command_buffer_.empty()) {
        // Buffer非空，检查最后一个命令的时间戳
        const auto& last_cmd = command_buffer_.back();
        auto time_since_last = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_cmd.timestamp
        ).count() / 1000000.0;
        
        if (time_since_last > stale_threshold_sec_ && !current_position.empty()) {
            // 缓存过期且有当前位置，使用当前真实位置
            from_position = current_position;
            using_current = true;
            RCLCPP_INFO(logger_, "[Bridge] Using current position (last_cmd %.1fms ago > %.1fms threshold)",
                        time_since_last * 1000.0, stale_threshold_sec_ * 1000.0);
        } else {
            // 缓存新鲜，使用buffer最后一个命令
            from_position = last_cmd.positions;
        }
    } else if (has_last_output_) {
        // Buffer为空，检查last_output的时间戳
        auto time_since_output = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_output_command_.timestamp
        ).count() / 1000000.0;
        
        if (time_since_output > stale_threshold_sec_ && !current_position.empty()) {
            // 缓存过期且有当前位置，使用当前真实位置
            from_position = current_position;
            using_current = true;
            RCLCPP_INFO(logger_, "[Bridge] Using current position (last_output %.1fms ago > %.1fms threshold)",
                        time_since_output * 1000.0, stale_threshold_sec_ * 1000.0);
        } else {
            // 缓存新鲜，使用last_output
            from_position = last_output_command_.positions;
        }
    } else {
        // 还没有初始化
        if (!current_position.empty()) {
            from_position = current_position;
            using_current = true;
            RCLCPP_INFO(logger_, "[Bridge] First command, using current position");
        } else {
            // 没有current_position，直接添加
            command_buffer_.emplace_back(joint_positions);
            last_command_time_ = now;
            RCLCPP_INFO(logger_, "[Bridge] First command added (no interpolation): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        joint_positions[0], joint_positions[1], joint_positions[2],
                        joint_positions[3], joint_positions[4], joint_positions[5],
                        joint_positions[6]);
            return true;
        }
    }
    
    // 计算需要的时间（基于速度限制）
    double required_time = calculateRequiredTime(from_position, joint_positions);
    
    // 如果需要的时间超过两倍周期，需要插入中间点
    if (required_time > 2.0 * cycle_time_sec_) {
        std::vector<std::vector<double>> intermediate_points;
        insertIntermediatePoints(from_position, joint_positions, intermediate_points);
        
        // 添加中间点
        for (const auto& point : intermediate_points) {
            if (command_buffer_.size() >= buffer_size_) {
                command_buffer_.pop_front();
                std::lock_guard<std::mutex> stats_lock(stats_mutex_);
                stats_.buffer_overflow_count++;
            }
            command_buffer_.emplace_back(point);
        }
        
        RCLCPP_INFO(logger_, "[Bridge] Inserted %zu intermediate points (required_time=%.1fms, cycle=%.1fms)%s",
                    intermediate_points.size(), required_time * 1000.0, cycle_time_sec_ * 1000.0,
                    using_current ? " [from current pos]" : "");
    }
    
    // 添加最终目标
    if (command_buffer_.size() >= buffer_size_) {
        command_buffer_.pop_front();
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        stats_.buffer_overflow_count++;
    }
    command_buffer_.emplace_back(joint_positions);
    
    last_command_time_ = now;
    
    RCLCPP_INFO(logger_, "[Bridge] Command added: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f], buffer_size=%zu%s",
                joint_positions[0], joint_positions[1], joint_positions[2],
                joint_positions[3], joint_positions[4], joint_positions[5],
                joint_positions[6], command_buffer_.size(),
                using_current ? " [from current pos]" : "");
    
    return true;
}

bool SmoothServoBridge::getInterpolatedCommand(std::vector<double>& interpolated_positions)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // ⭐ 伺服模式：持续保持最后位置，不超时失效
    // 这样可以保证单次命令后机械臂保持在目标位置
    auto now = std::chrono::steady_clock::now();
    
    if (command_buffer_.empty()) {
        // 如果缓冲器为空，持续使用上一次的输出（保持位置）
        if (has_last_output_) {
            interpolated_positions = last_output_command_.positions;
            RCLCPP_INFO(logger_, 
                "[Bridge] Buffer empty, holding last position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                interpolated_positions[0], interpolated_positions[1], interpolated_positions[2],
                interpolated_positions[3], interpolated_positions[4], interpolated_positions[5],
                interpolated_positions[6]);
            return true;
        }
        RCLCPP_INFO(logger_, "[Bridge] Buffer empty and no last output, returning false");
        return false;  // 仅在从未收到指令时返回false
    }
    
    // 获取最新的命令
    const JointCommand& latest_command = command_buffer_.back();
    
    if (!interpolation_enabled_ || !has_last_output_) {
        // 不插值，直接使用最新命令
        // （如果has_last_output_为false，说明是超时后第一个指令，需要重新同步）
        if (!has_last_output_) {
            RCLCPP_INFO(logger_,
                "Re-syncing: using latest command directly after timeout/init");
        }
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
        
        // 检查是否已经足够接近目标（用于判断是否可以清空buffer）
        // 因为我们使用.back()获取最新命令，如果已达到最新目标，说明所有中间命令都可以跳过
        double max_diff = 0.0;
        for (size_t i = 0; i < interpolated_positions.size(); ++i) {
            double diff = std::abs(interpolated_positions[i] - latest_command.positions[i]);
            max_diff = std::max(max_diff, diff);
        }
        
        // 如果所有关节都已经非常接近最新目标（误差小于0.001弧度，约0.057度）
        // 则认为已经达到目标，清空buffer（因为使用.back()时，中间的命令可以跳过）
        const double REACHED_THRESHOLD = 0.001;
        if (max_diff < REACHED_THRESHOLD && command_buffer_.size() > 0) {
            size_t cleared_count = command_buffer_.size();
            command_buffer_.clear();
            RCLCPP_DEBUG(logger_, 
                "[Bridge] Target reached (max_diff=%.6f), cleared %zu command(s) from buffer", 
                max_diff, cleared_count);
        }
    }
    
    // 计算延迟
    auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
        now - latest_command.timestamp
    ).count() / 1000.0;
    
    // 更新性能统计
    updatePerformanceStats(true, latency);
    
    last_output_time_ = now;
    
    RCLCPP_INFO(logger_, 
        "[Bridge] Sending interpolated: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f], buffer_size=%zu",
        interpolated_positions[0], interpolated_positions[1], interpolated_positions[2],
        interpolated_positions[3], interpolated_positions[4], interpolated_positions[5],
        interpolated_positions[6], command_buffer_.size());
    
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

double SmoothServoBridge::getIdleTime() const
{
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_time_
    ).count();
    return duration / 1000.0;  // 返回秒
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

bool SmoothServoBridge::initializeFromCurrent(const std::vector<double>& current_positions)
{
    if (current_positions.size() != 7) {
        RCLCPP_ERROR(logger_, "Invalid current positions size: %zu (expected 7)", 
                     current_positions.size());
        return false;
    }
    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // 初始化last_output_command_为当前机械臂位置
    last_output_command_.positions = current_positions;
    last_output_command_.timestamp = std::chrono::steady_clock::now();
    has_last_output_ = true;
    
    // 清空缓冲区，确保从干净状态开始
    command_buffer_.clear();
    
    RCLCPP_INFO(logger_, "Bridge initialized from current position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                current_positions[0], current_positions[1], current_positions[2],
                current_positions[3], current_positions[4], current_positions[5],
                current_positions[6]);
    
    return true;
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

void SmoothServoBridge::setVelocityLimits(const std::vector<double>& velocity_limits)
{
    if (velocity_limits.size() != 7) {
        RCLCPP_ERROR(logger_, "Invalid velocity limits size: %zu (expected 7)", 
                     velocity_limits.size());
        return;
    }
    velocity_limits_ = velocity_limits;
    RCLCPP_INFO(logger_, "Velocity limits updated: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] rad/s",
                velocity_limits[0], velocity_limits[1], velocity_limits[2],
                velocity_limits[3], velocity_limits[4], velocity_limits[5],
                velocity_limits[6]);
}

void SmoothServoBridge::setVelocitySafetyFactor(double factor)
{
    velocity_safety_factor_ = std::clamp(factor, 0.1, 1.0);
    RCLCPP_INFO(logger_, "Velocity safety factor set to: %.1f%%", velocity_safety_factor_ * 100.0);
}

double SmoothServoBridge::calculateRequiredTime(
    const std::vector<double>& from,
    const std::vector<double>& to
) const
{
    double max_time = 0.0;
    
    for (size_t i = 0; i < 7; ++i) {
        double delta = std::abs(to[i] - from[i]);
        double max_velocity = velocity_limits_[i] * velocity_safety_factor_;
        double required_time = delta / max_velocity;
        max_time = std::max(max_time, required_time);
    }
    
    return max_time;
}

void SmoothServoBridge::insertIntermediatePoints(
    const std::vector<double>& from,
    const std::vector<double>& to,
    std::vector<std::vector<double>>& intermediate_points
) const
{
    intermediate_points.clear();
    
    // 计算需要的时间
    double required_time = calculateRequiredTime(from, to);
    
    // 计算需要的周期数（向上取整）
    int num_cycles = static_cast<int>(std::ceil(required_time / cycle_time_sec_));
    
    // 如果只需要1个周期，不需要插入中间点
    if (num_cycles <= 1) {
        return;
    }
    
    // 插入 num_cycles-1 个中间点
    // 例如：num_cycles=3，需要2个中间点：I1(1/3), I2(2/3)
    for (int i = 1; i < num_cycles; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_cycles);
        std::vector<double> point(7);
        for (size_t j = 0; j < 7; ++j) {
            point[j] = from[j] + t * (to[j] - from[j]);
        }
        intermediate_points.push_back(point);
    }
    
    RCLCPP_DEBUG(logger_, "[Bridge] Inserted %d intermediate points for %d cycles (%.1fms total)",
                 num_cycles - 1, num_cycles, required_time * 1000.0);
}

} // namespace qyh_jaka_control
