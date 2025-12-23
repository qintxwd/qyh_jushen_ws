#pragma once

#include <rclcpp/rclcpp.hpp>
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
#include "jaka_interface.hpp"

namespace qyh_jaka_control {

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
        JakaInterface& jaka_interface,
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
    void handleBridgeStartServo(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void handleBridgeStopServo(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    
    // 运动控制服务
    void handleMoveJ(const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res);
    void handleMoveL(const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res);
    void handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res);
    void handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res);
    void handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res);
    
    // 点动控制服务
    void handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res);
    void handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res);

private:
    rclcpp::Node* node_;
    JakaInterface& jaka_interface_;
    std::atomic<bool>& connected_;
    std::atomic<bool>& powered_;
    std::atomic<bool>& enabled_;
    std::atomic<bool>& servo_running_;
    std::function<bool()> start_servo_callback_;
    std::function<bool()> stop_servo_callback_;
};

} // namespace qyh_jaka_control
