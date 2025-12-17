#ifndef QYH_JAKA_CONTROL_SMOOTH_SERVO_BRIDGE_HPP_
#define QYH_JAKA_CONTROL_SMOOTH_SERVO_BRIDGE_HPP_

#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace qyh_jaka_control
{

/**
 * @brief 关节状态结构
 */
struct JointCommand
{
    std::vector<double> positions;  // 7 DOF joint positions
    std::chrono::steady_clock::time_point timestamp;
    
    JointCommand() : positions(7, 0.0) {}
    
    explicit JointCommand(const std::vector<double>& pos) 
        : positions(pos), timestamp(std::chrono::steady_clock::now()) {}
};

/**
 * @brief 性能统计
 */
struct ServoPerformanceStats
{
    double average_frequency{0.0};       // Hz
    double average_latency_ms{0.0};      // ms
    size_t command_count{0};
    size_t error_count{0};
    size_t buffer_overflow_count{0};
    
    std::chrono::steady_clock::time_point last_update_time;
    
    ServoPerformanceStats() : last_update_time(std::chrono::steady_clock::now()) {}
    
    void reset()
    {
        average_frequency = 0.0;
        average_latency_ms = 0.0;
        command_count = 0;
        error_count = 0;
        buffer_overflow_count = 0;
        last_update_time = std::chrono::steady_clock::now();
    }
};

/**
 * @brief 平滑伺服桥接类
 * 
 * 功能：
 * 1. 轨迹缓冲器：维护最近的N个命令点
 * 2. 插值平滑：在缓冲点之间进行线性插值
 * 3. 时间同步：确保125Hz稳定输出
 * 4. 性能监控：频率、延迟、错误统计
 */
class SmoothServoBridge
{
public:
    /**
     * @brief 构造函数
     * @param logger ROS2 logger for logging
     * @param buffer_size 缓冲器大小（默认10）
     * @param target_frequency_hz 目标输出频率（默认125Hz）
     */
    explicit SmoothServoBridge(
        rclcpp::Logger logger,
        size_t buffer_size = 10,
        double target_frequency_hz = 125.0
    );
    
    ~SmoothServoBridge() = default;
    
    /**
     * @brief 添加新的关节命令到缓冲器
     * @param joint_positions 7 DOF关节位置
     * @return true if successful
     */
    bool addCommand(const std::vector<double>& joint_positions);
    
    /**
     * @brief 获取插值后的关节命令（用于125Hz定时器）
     * @param interpolated_positions 输出：插值后的关节位置
     * @return true if valid command available
     */
    bool getInterpolatedCommand(std::vector<double>& interpolated_positions);
    
    /**
     * @brief 清空缓冲器
     */
    void clearBuffer();
    
    /**
     * @brief 检查缓冲器是否为空
     */
    bool isEmpty() const;
    
    /**
     * @brief 获取缓冲器当前大小
     */
    size_t getBufferSize() const;
    
    /**
     * @brief 获取性能统计
     */
    ServoPerformanceStats getPerformanceStats() const;
    
    /**
     * @brief 重置性能统计
     */
    void resetPerformanceStats();
    
    /**
     * @brief 设置插值权重（0.0-1.0）
     * 权重越大，越倾向于使用新命令（响应快但可能不平滑）
     * 权重越小，越倾向于使用旧命令（平滑但响应慢）
     * @param weight 插值权重，默认0.5
     */
    void setInterpolationWeight(double weight);
    
    /**
     * @brief 启用/禁用插值
     * @param enable true=插值，false=直接使用最新命令
     */
    void enableInterpolation(bool enable);
    
    /**
     * @brief 从当前机械臂位置初始化，避免启动时的跳变
     * 应在startServo后、发送第一个命令前调用
     * @param current_positions 当前机械臂的关节位置 (7 DOF)
     * @return true if successful
     */
    bool initializeFromCurrent(const std::vector<double>& current_positions);

private:
    rclcpp::Logger logger_;
    
    // 缓冲器设置
    size_t buffer_size_;
    double target_frequency_hz_;
    double target_period_ms_;
    
    // 命令缓冲器
    std::deque<JointCommand> command_buffer_;
    mutable std::mutex buffer_mutex_;
    
    // 插值设置
    bool interpolation_enabled_;
    double interpolation_weight_;  // 0.0-1.0
    
    // 上一次输出的命令（用于插值）
    JointCommand last_output_command_;
    bool has_last_output_;
    
    // 超时重新同步设置
    double command_timeout_sec_;  // 无指令超时时间（秒），超时后自动失效last_output
    
    // 性能统计
    ServoPerformanceStats stats_;
    mutable std::mutex stats_mutex_;
    
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point last_output_time_;
    
    // 辅助函数
    
    /**
     * @brief 线性插值
     */
    std::vector<double> interpolate(
        const std::vector<double>& from,
        const std::vector<double>& to,
        double weight
    ) const;
    
    /**
     * @brief 更新性能统计
     */
    void updatePerformanceStats(bool success, double latency_ms);
};

} // namespace qyh_jaka_control

#endif // QYH_JAKA_CONTROL_SMOOTH_SERVO_BRIDGE_HPP_
