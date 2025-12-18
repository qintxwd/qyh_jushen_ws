#include "qyh_jaka_control/jaka_service_handlers.hpp"
#include <cmath>

namespace qyh_jaka_control {

JakaServiceHandlers::JakaServiceHandlers(
    rclcpp::Node* node,
    JakaInterface& jaka_interface,
    std::atomic<bool>& connected,
    std::atomic<bool>& powered,
    std::atomic<bool>& enabled,
    std::atomic<bool>& servo_running,
    std::function<bool()> start_servo_callback,
    std::function<bool()> stop_servo_callback)
    : node_(node),
      jaka_interface_(jaka_interface),
      connected_(connected),
      powered_(powered),
      enabled_(enabled),
      servo_running_(servo_running),
      start_servo_callback_(start_servo_callback),
      stop_servo_callback_(stop_servo_callback)
{
}

// ==================== 基础控制服务 ====================
void JakaServiceHandlers::handlePowerOn(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (!connected_) { res->success = false; res->message = "Robot not connected"; return; }
    if (jaka_interface_.powerOn()) { powered_ = true; res->success = true; res->message = "Power ON success"; }
    else { res->success = false; res->message = "Power ON failed"; }
}

void JakaServiceHandlers::handlePowerOff(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
    if (jaka_interface_.powerOff()) { powered_ = false; enabled_ = false; res->success = true; res->message = "Power OFF success"; }
    else { res->success = false; res->message = "Power OFF failed"; }
}

void JakaServiceHandlers::handleEnable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (!powered_) { res->success = false; res->message = "Robot not powered on"; return; }
    if (jaka_interface_.enableRobot()) { enabled_ = true; res->success = true; res->message = "Robot ENABLED"; }
    else { res->success = false; res->message = "Enable failed"; }
}

void JakaServiceHandlers::handleDisable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
    if (jaka_interface_.disableRobot()) { enabled_ = false; res->success = true; res->message = "Robot DISABLED"; }
    else { res->success = false; res->message = "Disable failed"; }
}

void JakaServiceHandlers::handleClearError(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (jaka_interface_.clearError()) { res->success = true; res->message = "Error cleared"; }
    else { res->success = false; res->message = "Clear error failed"; }
}

void JakaServiceHandlers::handleMotionAbort(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (jaka_interface_.motionAbort()) { servo_running_ = false; res->success = true; res->message = "Motion aborted"; }
    else { res->success = false; res->message = "Motion abort failed"; }
}

// ==================== 伺服控制服务 ====================
void JakaServiceHandlers::handleStartServo(const qyh_jaka_control_msgs::srv::StartServo::Request::SharedPtr, qyh_jaka_control_msgs::srv::StartServo::Response::SharedPtr res)
{
    if (start_servo_callback_()) { res->success = true; res->message = "Servo started"; }
    else { res->success = false; res->message = "Start servo failed"; }
}

void JakaServiceHandlers::handleStopServo(const qyh_jaka_control_msgs::srv::StopServo::Request::SharedPtr, qyh_jaka_control_msgs::srv::StopServo::Response::SharedPtr res)
{
    if (stop_servo_callback_()) { res->success = true; res->message = "Servo stopped"; }
    else { res->success = false; res->message = "Stop servo failed"; }
}

void JakaServiceHandlers::handleBridgeStartServo(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (start_servo_callback_()) { res->success = true; res->message = "Servo started (Bridge)"; }
    else { res->success = false; res->message = "Start servo failed"; }
}

void JakaServiceHandlers::handleBridgeStopServo(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (stop_servo_callback_()) { res->success = true; res->message = "Servo stopped (Bridge)"; }
    else { res->success = false; res->message = "Stop servo failed"; }
}

// ==================== 运动控制服务 ====================
void JakaServiceHandlers::handleMoveJ(const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res) {
    res->success = jaka_interface_.moveJ(req->robot_id, std::vector<double>(req->joint_positions.begin(), req->joint_positions.end()), 
                                       req->move_mode, req->velocity, req->acceleration, req->is_block);
}

void JakaServiceHandlers::handleMoveL(const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res) {
    res->success = jaka_interface_.moveL(req->robot_id, req->target_pose, req->move_mode, req->velocity, req->acceleration, req->is_block);
}

void JakaServiceHandlers::handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res) {
    res->success = true; // Placeholder
}

void JakaServiceHandlers::handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res) {
    res->success = true; // Placeholder
}

void JakaServiceHandlers::handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res) {
    res->success = true; // Placeholder
}

} // namespace qyh_jaka_control
