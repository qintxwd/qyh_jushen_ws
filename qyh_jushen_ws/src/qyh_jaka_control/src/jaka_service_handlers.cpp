#include "qyh_jaka_control/jaka_service_handlers.hpp"
#include <cmath>

namespace qyh_jaka_control {

CartesianPose rosPoseToJaka(const geometry_msgs::msg::Pose& ros_pose)
{
    CartesianPose jaka_pose;
    jaka_pose.tran.x = ros_pose.position.x * 1000.0;  // m to mm
    jaka_pose.tran.y = ros_pose.position.y * 1000.0;
    jaka_pose.tran.z = ros_pose.position.z * 1000.0;

    // Convert quaternion to Euler angles (XYZ fixed-axis convention)
    // JAKA uses XYZ Euler: R = Rz(rz) * Ry(ry) * Rx(rx)
    // This is verified to match JAKA's official controller data
    tf2::Quaternion q(ros_pose.orientation.x, ros_pose.orientation.y, 
                      ros_pose.orientation.z, ros_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    // Extract XYZ Euler angles from rotation matrix
    // Reference: 正逆解说明.md - Section 2.2.3
    double rx, ry, rz;
    double sy = std::sqrt(m[2][1] * m[2][1] + m[2][2] * m[2][2]);
    
    if (sy > 1e-6) {  // Not at singularity
        rx = std::atan2(m[2][1], m[2][2]);
        ry = std::atan2(-m[2][0], sy);
        rz = std::atan2(m[1][0], m[0][0]);
    } else {  // Gimbal lock case (ry = ±90°)
        rx = std::atan2(-m[1][2], m[1][1]);
        ry = std::atan2(-m[2][0], sy);
        rz = 0.0;
    }
    
    jaka_pose.rpy.rx = rx;
    jaka_pose.rpy.ry = ry;
    jaka_pose.rpy.rz = rz;

    return jaka_pose;
}

geometry_msgs::msg::Pose jakaPoseToRos(const CartesianPose& jaka_pose)
{
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = jaka_pose.tran.x / 1000.0;  // mm to m
    ros_pose.position.y = jaka_pose.tran.y / 1000.0;
    ros_pose.position.z = jaka_pose.tran.z / 1000.0;

    // Convert XYZ Euler angles to quaternion
    // JAKA uses XYZ Euler: R = Rz(rz) * Ry(ry) * Rx(rx)
    double rx = jaka_pose.rpy.rx;
    double ry = jaka_pose.rpy.ry;
    double rz = jaka_pose.rpy.rz;
    
    // Build rotation matrix from XYZ Euler angles
    double cx = std::cos(rx), sx = std::sin(rx);
    double cy = std::cos(ry), sy = std::sin(ry);
    double cz = std::cos(rz), sz = std::sin(rz);
    
    tf2::Matrix3x3 m(
        cy*cz,              -cy*sz,             sy,
        cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,  -sx*cy,
        sx*sz - cx*sy*cz,   sx*cz + cx*sy*sz,   cx*cy
    );
    
    tf2::Quaternion q;
    m.getRotation(q);
    
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();
    ros_pose.orientation.w = q.w();

    return ros_pose;
}

JakaServiceHandlers::JakaServiceHandlers(
    rclcpp::Node* node,
    std::shared_ptr<JAKAZuRobot> robot,
    std::atomic<bool>& connected,
    std::atomic<bool>& powered,
    std::atomic<bool>& enabled,
    std::atomic<bool>& servo_running,
    std::function<bool()> start_servo_callback,
    std::function<bool()> stop_servo_callback)
    : node_(node),
      robot_(robot),
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
    if (robot_->power_on() == ERR_SUCC) { powered_ = true; res->success = true; res->message = "Power ON success"; }
    else { res->success = false; res->message = "Power ON failed"; }
}

void JakaServiceHandlers::handlePowerOff(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
    if (robot_->power_off() == ERR_SUCC) { powered_ = false; enabled_ = false; res->success = true; res->message = "Power OFF success"; }
    else { res->success = false; res->message = "Power OFF failed"; }
}

void JakaServiceHandlers::handleEnable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (!powered_) { res->success = false; res->message = "Robot not powered on"; return; }
    if (robot_->enable_robot() == ERR_SUCC) { enabled_ = true; res->success = true; res->message = "Robot ENABLED"; }
    else { res->success = false; res->message = "Enable failed"; }
}

void JakaServiceHandlers::handleDisable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
    if (robot_->disable_robot() == ERR_SUCC) { enabled_ = false; res->success = true; res->message = "Robot DISABLED"; }
    else { res->success = false; res->message = "Disable failed"; }
}

void JakaServiceHandlers::handleClearError(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (robot_->clear_error() == ERR_SUCC) { res->success = true; res->message = "Error cleared"; }
    else { res->success = false; res->message = "Clear error failed"; }
}

void JakaServiceHandlers::handleMotionAbort(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    if (robot_->motion_abort() == ERR_SUCC) { servo_running_ = false; res->success = true; res->message = "Motion aborted"; }
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

// ==================== 运动控制服务 ====================
void JakaServiceHandlers::handleMoveJ(const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res) {
    
    auto &robot_id = req->robot_id;
    auto &joint_positions = req->joint_positions;
    auto &move_mode = req->move_mode;
    auto &velocity = req->velocity;
    auto &acceleration = req->acceleration;
    auto &is_block = req->is_block;

    if (joint_positions.size() != 14 && joint_positions.size() != 7) {
        RCLCPP_ERROR(logger_, "Invalid joint positions size: %zu, expected 7 or 14", joint_positions.size());
        return false;
    }

    JointValue jpos[2];
    memset(&jpos, 0, sizeof(jpos));

    if (joint_positions.size() == 14) {
        // Dual arm control
        for (size_t i = 0; i < 7; ++i) {
            jpos[0].jVal[i] = joint_positions[i];
            jpos[1].jVal[i] = joint_positions[i + 7];
        }
    } else {
        // Single arm control
        for (size_t i = 0; i < 7; ++i) {
            jpos[0].jVal[i] = joint_positions[i];
        }
    }

    MoveMode modes[2] = {move_mode ? MoveMode::INCR : MoveMode::ABS, 
                         move_mode ? MoveMode::INCR : MoveMode::ABS};
    double vel[2] = {velocity, velocity};
    double acc[2] = {acceleration, acceleration};
    
    errno_t ret = robot_->robot_run_multi_movj(robot_id, modes, is_block, jpos, vel, acc);    
    res.success = (ret == ERR_SUCC);
    if(!res.success) {
        RCLCPP_ERROR(logger_, "MoveJ failed with error code: %d", ret);
    }
}

void JakaServiceHandlers::handleMoveL(const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res) {
    auto &robot_id = req->robot_id;
    auto &target_pose = req->target_pose;
    auto &move_mode = req->move_mode;
    auto &velocity = req->velocity;
    auto &acceleration = req->acceleration;
    auto &is_block = req->is_block;

    CartesianPose poses[2];
    memset(&poses, 0, sizeof(poses));
    poses[0] = rosPoseToJaka(target_pose);
    poses[1] = poses[0];

    MoveMode modes[2] = {move_mode ? MoveMode::INCR : MoveMode::ABS, 
                            move_mode ? MoveMode::INCR : MoveMode::ABS};
    double vel[2] = {velocity, velocity};
    double acc[2] = {acceleration, acceleration};

    errno_t ret = robot_->robot_run_multi_movl(robot_id, modes, is_block, poses, vel, acc);
    res.success = (ret == ERR_SUCC);
    if(!res.success) {
        RCLCPP_ERROR(logger_, "MoveL failed with error code: %d", ret);
    }
}

void JakaServiceHandlers::handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res) {
    // res->success = robot_->setToolOffset(req->robot_id, req->tool_offset);
    auto &robot_id = req->robot_id;
    auto &tool_offset = req->tool_offset;
    CartesianPose jaka_offset = rosPoseToJaka(tool_offset);
    errno_t ret = robot_->robot_set_tool_offset(robot_id, jaka_offset);
    res->success = (ret == ERR_SUCC);
    if(!res->success) {
        RCLCPP_ERROR(logger_, "SetToolOffset failed with error code: %d", ret);
    }
}

void JakaServiceHandlers::handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res) {
    // res->success = robot_->setPayload(req->robot_id, req->mass);
    auto &robot_id = req->robot_id;
    auto &mass = req->mass;
    auto centroid_x = 150.0;  // 默认质心位置，单位mm
    PayLoad payload;
    payload.mass = mass;
    payload.centroid.x = 0.0;  
    payload.centroid.y = 0.0;
    payload.centroid.z = centroid_x;// 默认150mm (15cm向前)
    
    RCLCPP_INFO(logger_, "Setting payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                robot_id, mass, centroid_x, payload.centroid.y, payload.centroid.z);
    
    errno_t ret = robot_->robot_set_tool_payload(robot_id, &payload);
}

void JakaServiceHandlers::handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr req, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res) {
    double mass;
    double centroid_x, centroid_y, centroid_z;
    
    // res->success = robot_->getPayload(req->robot_id, mass, centroid_x, centroid_y, centroid_z);
    // if (res->success) {
    //     res->mass = mass;
    //     res->centroid_x = centroid_x;
    //     res->centroid_y = centroid_y;
    //     res->centroid_z = centroid_z;
    // }

    PayLoad payload;
    errno_t ret = robot_->robot_get_tool_payload(&payload);
    
    if (ret == ERR_SUCC) {
        res->mass = payload.mass;
        res->centroid_x = payload.centroid.x;
        res->centroid_y = payload.centroid.y;
        res->centroid_z = payload.centroid.z;
        RCLCPP_INFO(logger_, "Got payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                    robot_id, res->mass, res->centroid_x, res->centroid_y, res->centroid_z);
    }else{
        RCLCPP_ERROR(logger_, "GetPayload failed with error code: %d", ret);
    }
}

// // ==================== 点动控制服务 ====================
// void JakaServiceHandlers::handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res)
// {
//     RCLCPP_INFO(node_->get_logger(), "Jog: robot=%d axis=%d mode=%d coord=%d vel=%.3f pos=%.3f",
//         req->robot_id, req->axis_num, req->move_mode, req->coord_type, req->velocity, req->position);
    
//     // 检查机器人状态
//     if (!connected_) {
//         res->success = false;
//         res->message = "Robot not connected";
//         return;
//     }
    
//     if (!enabled_) {
//         res->success = false;
//         res->message = "Robot not enabled";
//         return;
//     }
    
//     if (servo_running_) {
//         res->success = false;
//         res->message = "Cannot jog while servo is running";
//         return;
//     }
    
//     // 将 1-based 索引转换为 0-based
//     int axis_index = req->axis_num - 1;
    
//     bool success = false;
    
//     // 根据坐标系类型选择点动方式
//     if (req->coord_type == 1) {  // COORD_JOINT=1 关节空间
//         if (req->move_mode == 0) {  // MOVE_CONTINUOUS 连续运动
//             int direction = (req->velocity > 0) ? 1 : -1;
//             success = robot_->jogJointContinuous(
//                 req->robot_id, 
//                 axis_index, 
//                 direction,
//                 std::abs(req->velocity) * 30.0  // 将 rad/s 转换为速度百分比近似值
//             );
//         } else {  // MOVE_STEP 步进运动
//             success = robot_->jogJoint(
//                 req->robot_id,
//                 axis_index,
//                 req->position * 180.0 / M_PI,  // 弧度转角度
//                 std::abs(req->velocity) * 30.0
//             );
//         }
//     } else {  // 笛卡尔空间 (COORD_BASE=0, COORD_TOOL=2)
//         // coord_type 直接使用请求值，已经与 JAKA SDK CoordType 枚举对齐
//         int coord_type = req->coord_type;
        
//         if (req->move_mode == 0) {  // MOVE_CONTINUOUS 连续运动
//             int direction = (req->velocity > 0) ? 1 : -1;
//             success = robot_->jogCartesianContinuous(
//                 req->robot_id,
//                 axis_index,
//                 direction,
//                 std::abs(req->velocity) * 10.0,  // 将 mm/s 转换为速度百分比近似值
//                 coord_type
//             );
//         } else {  // MOVE_STEP 步进运动
//             double step = req->position;
//             // 对于旋转轴 (3,4,5)，需要转换为角度
//             if (axis_index >= 3) {
//                 step = req->position * 180.0 / M_PI;
//             }
//             success = robot_->jogCartesian(
//                 req->robot_id,
//                 axis_index,
//                 step,
//                 std::abs(req->velocity) * 10.0,
//                 coord_type
//             );
//         }
//     }
    
//     res->success = success;
//     res->message = success ? "Jog command sent" : "Jog command failed";
// }

// void JakaServiceHandlers::handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res)
// {
//     RCLCPP_INFO(node_->get_logger(), "Jog Stop: robot=%d axis=%d", req->robot_id, req->axis_num);
    
//     bool success = robot_->jogStop(req->robot_id);
    
//     res->success = success;
//     res->message = success ? "Jog stopped" : "Jog stop failed";
// }

} // namespace qyh_jaka_control
