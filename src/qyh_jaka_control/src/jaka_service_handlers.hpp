#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <chrono>
#include <std_srvs/srv/trigger.hpp>
#include "qyh_jaka_control_msgs/srv/start_servo.hpp"
#include "qyh_jaka_control_msgs/srv/stop_servo.hpp"
#include "qyh_jaka_control_msgs/srv/move_j.hpp"
#include "qyh_jaka_control_msgs/srv/move_l.hpp"
#include "qyh_jaka_control_msgs/srv/set_tool_offset.hpp"
#include "qyh_jaka_control_msgs/srv/set_payload.hpp"
#include "qyh_jaka_control_msgs/srv/get_payload.hpp"
#include "qyh_jaka_control_msgs/srv/jog.hpp"
#include "qyh_jaka_control_msgs/srv/jog_stop.hpp"
#include "JAKAZuRobot.h"
#include <array>

namespace qyh_jaka_control {

// JAKA ZU7 关节限位结构
struct JointLimits {
    double min_pos;    // 最小位置 (rad)
    double max_pos;    // 最大位置 (rad)
    double max_vel;    // 最大速度 (rad/s)
};

// JAKA ZU7 各关节限位 (7轴)
static const std::array<JointLimits, 7> JAKA_ZU7_LIMITS = {{
    {-6.2832, 6.2832, 1.5708},   // 关节1: ±360°, 90°/s
    {-1.8326, 1.8326, 1.5708},   // 关节2: ±105°, 90°/s
    {-6.2832, 6.2832, 2.0944},   // 关节3: ±360°, 120°/s
    {-2.5307, 0.5236, 2.0944},   // 关节4: -145°~30°, 120°/s
    {-6.2832, 6.2832, 2.6180},   // 关节5: ±360°, 150°/s
    {-1.8326, 1.8326, 2.6180},   // 关节6: ±105°, 150°/s
    {-6.2832, 6.2832, 2.6180}    // 关节7: ±360°, 150°/s
}};

static constexpr double JOG_SAFETY_MARGIN = 0.0873;  // 5° 安全裕度

/**
 * @brief 服务处理器类 - 封装所有ROS服务的回调逻辑
 * 
 * 职责：
 * - 处理所有ROS服务请求
 * - 调用JakaInterface执行硬件操作
 * - 管理机器人状态标志（powered, enabled, servo_running）
 */
class JakaServiceHandlers {
public:
    JakaServiceHandlers(
        rclcpp::Node* node,
        std::shared_ptr<JAKAZuRobot> robot,
        std::atomic<bool>& connected,
        std::atomic<bool>& powered,
        std::atomic<bool>& enabled,
        std::atomic<bool>& servo_running,
        std::function<bool()> start_servo_callback,
        std::function<bool()> stop_servo_callback);
    
    // 基础控制服务
    void handlePowerOn(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handlePowerOff(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handleEnable(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handleDisable(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handleClearError(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handleMotionAbort(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    
    // 伺服控制服务
    void handleStartServo(const qyh_jaka_control_msgs::srv::StartServo::Request::SharedPtr req, qyh_jaka_control_msgs::srv::StartServo::Response::SharedPtr res);
    void handleStopServo(const qyh_jaka_control_msgs::srv::StopServo::Request::SharedPtr req, qyh_jaka_control_msgs::srv::StopServo::Response::SharedPtr res);

    // 运动控制服务
    void handleMoveJ(const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res);
    void handleMoveL(const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res);
    void handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res);
    void handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res);
    void handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res);
    
    // 点动控制服务
    void handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res);
    void handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res);

    // 连续Jog心跳检测
    void checkJogTimeout();
    void stopContinuousJog();
    
private:
    rclcpp::Node* node_;
    // JakaInterface& jaka_interface_;
    std::shared_ptr<JAKAZuRobot> robot_;
    std::atomic<bool>& connected_;
    std::atomic<bool>& powered_;
    std::atomic<bool>& enabled_;
    std::atomic<bool>& servo_running_;
    std::function<bool()> start_servo_callback_;
    std::function<bool()> stop_servo_callback_;
    
    // 连续Jog状态管理
    std::atomic<bool> continuous_jog_active_{false};
    std::chrono::steady_clock::time_point last_jog_heartbeat_;
    rclcpp::TimerBase::SharedPtr jog_timeout_timer_;
    std::mutex jog_mutex_;
    static constexpr int JOG_TIMEOUT_MS = 300;  // 2帧超时 (150ms * 2)
};

} // namespace qyh_jaka_control
