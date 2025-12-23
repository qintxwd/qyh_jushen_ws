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

void JakaServiceHandlers::handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res) {
    res->success = jaka_interface_.setToolOffset(req->robot_id, req->tool_offset);
}

void JakaServiceHandlers::handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res) {
    res->success = jaka_interface_.setPayload(req->robot_id, req->mass);
}

void JakaServiceHandlers::handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res) {
    double mass;
    double centroid_x, centroid_y, centroid_z;
    
    res->success = jaka_interface_.getPayload(req->robot_id, mass, centroid_x, centroid_y, centroid_z);
    if (res->success) {
        res->mass = mass;
        res->centroid_x = centroid_x;
        res->centroid_y = centroid_y;
        res->centroid_z = centroid_z;
    }
}

// ==================== 点动控制服务 ====================
void JakaServiceHandlers::handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res)
{
    RCLCPP_INFO(node_->get_logger(), "Jog: robot=%d axis=%d mode=%d coord=%d vel=%.3f pos=%.3f",
        req->robot_id, req->axis_num, req->move_mode, req->coord_type, req->velocity, req->position);
    
    // 检查机器人状态
    if (!connected_) {
        res->success = false;
        res->message = "Robot not connected";
        return;
    }
    
    if (!enabled_) {
        res->success = false;
        res->message = "Robot not enabled";
        return;
    }
    
    if (servo_running_) {
        res->success = false;
        res->message = "Cannot jog while servo is running";
        return;
    }
    
    // 将 1-based 索引转换为 0-based
    int axis_index = req->axis_num - 1;
    
    bool success = false;
    
    // 根据坐标系类型选择点动方式
    if (req->coord_type == 1) {  // COORD_JOINT=1 关节空间
        if (req->move_mode == 0) {  // MOVE_CONTINUOUS 连续运动
            int direction = (req->velocity > 0) ? 1 : -1;
            success = jaka_interface_.jogJointContinuous(
                req->robot_id, 
                axis_index, 
                direction,
                std::abs(req->velocity) * 30.0  // 将 rad/s 转换为速度百分比近似值
            );
        } else {  // MOVE_STEP 步进运动
            success = jaka_interface_.jogJoint(
                req->robot_id,
                axis_index,
                req->position * 180.0 / M_PI,  // 弧度转角度
                std::abs(req->velocity) * 30.0
            );
        }
    } else {  // 笛卡尔空间 (COORD_BASE=0, COORD_TOOL=2)
        // coord_type 直接使用请求值，已经与 JAKA SDK CoordType 枚举对齐
        int coord_type = req->coord_type;
        
        if (req->move_mode == 0) {  // MOVE_CONTINUOUS 连续运动
            int direction = (req->velocity > 0) ? 1 : -1;
            success = jaka_interface_.jogCartesianContinuous(
                req->robot_id,
                axis_index,
                direction,
                std::abs(req->velocity) * 10.0,  // 将 mm/s 转换为速度百分比近似值
                coord_type
            );
        } else {  // MOVE_STEP 步进运动
            double step = req->position;
            // 对于旋转轴 (3,4,5)，需要转换为角度
            if (axis_index >= 3) {
                step = req->position * 180.0 / M_PI;
            }
            success = jaka_interface_.jogCartesian(
                req->robot_id,
                axis_index,
                step,
                std::abs(req->velocity) * 10.0,
                coord_type
            );
        }
    }
    
    res->success = success;
    res->message = success ? "Jog command sent" : "Jog command failed";
}

void JakaServiceHandlers::handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res)
{
    RCLCPP_INFO(node_->get_logger(), "Jog Stop: robot=%d axis=%d", req->robot_id, req->axis_num);
    
    bool success = jaka_interface_.jogStop(req->robot_id);
    
    res->success = success;
    res->message = success ? "Jog stopped" : "Jog stop failed";
}

} // namespace qyh_jaka_control
