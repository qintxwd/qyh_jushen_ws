#include "jaka_service_handlers.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
        res->success = false;
        res->message = "Invalid joint positions size: expected 7 or 14";
        RCLCPP_ERROR(node_->get_logger(), "Invalid joint positions size: %zu, expected 7 or 14", joint_positions.size());
        return;
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
    res->success = (ret == ERR_SUCC);
    if(!res->success) {
        RCLCPP_ERROR(node_->get_logger(), "MoveJ failed with error code: %d", ret);
        res->message = "MoveJ failed";
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
    res->success = (ret == ERR_SUCC);
    if(!res->success) {
        RCLCPP_ERROR(node_->get_logger(), "MoveL failed with error code: %d", ret);
        res->message = "MoveL failed";
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
        RCLCPP_ERROR(node_->get_logger(), "SetToolOffset failed with error code: %d", ret);
        res->message = "SetToolOffset failed";
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
    
    RCLCPP_INFO(node_->get_logger(), "Setting payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                robot_id, mass, centroid_x, payload.centroid.y, payload.centroid.z);

    errno_t ret = robot_->robot_set_tool_payload(robot_id, &payload);
    res->success = (ret == ERR_SUCC);
    if(!res->success) {
        RCLCPP_ERROR(node_->get_logger(), "SetPayload failed with error code: %d", ret);
        res->message = "SetPayload failed";
    }
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

    auto &robot_id = req->robot_id;
    PayLoad payload;
    errno_t ret = robot_->robot_get_tool_payload(&payload);

    if (ret == ERR_SUCC) {
        res->mass = payload.mass;
        res->centroid_x = payload.centroid.x;
        res->centroid_y = payload.centroid.y;
        res->centroid_z = payload.centroid.z;
        RCLCPP_INFO(node_->get_logger(), "Got payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                    robot_id, res->mass, res->centroid_x, res->centroid_y, res->centroid_z);
        res->success = true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "GetPayload failed with error code: %d", ret);
        res->success = false;
        res->message = "GetPayload failed";
    }
}

// ==================== 点动控制服务 ====================

// 连续Jog心跳超时检测
void JakaServiceHandlers::checkJogTimeout() {
    std::lock_guard<std::mutex> lock(jog_mutex_);
    if (!continuous_jog_active_) return;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_jog_heartbeat_).count();
    
    if (elapsed > JOG_TIMEOUT_MS) {
        RCLCPP_WARN(node_->get_logger(), "Jog heartbeat timeout (%ld ms), stopping motion", elapsed);
        continuous_jog_active_ = false;
        robot_->motion_abort();
    }
}

// 停止连续Jog
void JakaServiceHandlers::stopContinuousJog() {
    std::lock_guard<std::mutex> lock(jog_mutex_);
    if (continuous_jog_active_) {
        continuous_jog_active_ = false;
        robot_->motion_abort();
        RCLCPP_INFO(node_->get_logger(), "Continuous jog stopped");
    }
}

void JakaServiceHandlers::handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res)
{
    RCLCPP_INFO(node_->get_logger(), "Jog request: robot=%d axis=%d mode=%d coord=%d vel=%.3f pos=%.3f",
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
    
    if (axis_index < 0 || axis_index > 6) {
        res->success = false;
        res->message = "Invalid axis number (must be 1-7)";
        return;
    }
    
    errno_t ret = ERR_FUCTION_CALL_ERROR;
    
    // 目前只支持关节空间点动（COORD_JOINT=1）
    if (req->coord_type == 1) {
        JointValue target_joints[2];  // 双臂数组
        memset(target_joints, 0, sizeof(target_joints));
        
        if (req->move_mode == 1) {
            // ========== 步进模式: 移动固定距离 ==========
            // 根据robot_id决定设置哪个臂的关节
            int arm_idx = (req->robot_id == 1) ? 1 : 0;
            target_joints[arm_idx].jVal[axis_index] = req->position;
            
            MoveMode modes[2] = {MoveMode::INCR, MoveMode::INCR};
            double vel[2] = {std::abs(req->velocity), std::abs(req->velocity)};
            double acc[2] = {5.0, 5.0};
            
            RCLCPP_INFO(node_->get_logger(), "Step jog: arm=%d axis=%d pos=%.4f vel=%.3f", 
                        arm_idx, axis_index + 1, req->position, req->velocity);
            
            ret = robot_->robot_run_multi_movj(req->robot_id, modes, FALSE, target_joints, vel, acc);
            
        } else {
            // ========== 连续模式: 发送大目标 + 心跳机制 ==========
            std::lock_guard<std::mutex> lock(jog_mutex_);
            
            // 更新心跳时间
            last_jog_heartbeat_ = std::chrono::steady_clock::now();
            
            // 如果已经在连续运动中
            if (continuous_jog_active_) {
                // 检测是否已到位，如果到位则发送新目标继续运动
                int inpos[2] = {0, 0};
                robot_->robot_is_inpos(inpos);
                bool robot_stopped = (req->robot_id == 0) ? inpos[0] : 
                                     (req->robot_id == 1) ? inpos[1] : (inpos[0] && inpos[1]);
                
                if (robot_stopped) {
                    // 机械臂已到位，需要发送新目标继续运动
                    RCLCPP_DEBUG(node_->get_logger(), "Robot in position, sending new target");
                    continuous_jog_active_ = false;  // 重置标志，让下面的代码发送新目标
                } else {
                    // 机械臂仍在运动，只更新心跳
                    res->success = true;
                    res->message = "Jog heartbeat received";
                    return;
                }
            }
            
            // 首次启动连续Jog：发送目标到关节限位
            continuous_jog_active_ = true;
            
            // 获取当前关节位置
            JointValue current_joint;
            CartesianPose current_pose;
            unsigned char arm_idx = (req->robot_id == 1) ? 1 : 0;  // 0=左臂, 1=右臂
            errno_t stat_ret = robot_->edg_get_stat(arm_idx, &current_joint, &current_pose);
            
            const auto& limits = JAKA_ZU7_LIMITS[axis_index];
            MoveMode modes[2];
            
            if (stat_ret == ERR_SUCC) {
                // 成功获取当前位置，使用绝对模式
                // 复制当前所有关节位置作为目标
                for (int i = 0; i < 7; ++i) {
                    target_joints[arm_idx].jVal[i] = current_joint.jVal[i];
                }
                
                // 计算目标位置（到限位减去安全裕度）
                double target_pos;
                if (req->velocity > 0) {
                    target_pos = limits.max_pos - JOG_SAFETY_MARGIN;
                } else {
                    target_pos = limits.min_pos + JOG_SAFETY_MARGIN;
                }
                
                target_joints[arm_idx].jVal[axis_index] = target_pos;
                modes[0] = modes[1] = MoveMode::ABS;
                
                RCLCPP_INFO(node_->get_logger(), "Continuous jog (ABS): axis=%d, current=%.3f, target=%.3f",
                            axis_index + 1, current_joint.jVal[axis_index], target_pos);
            } else {
                // 获取位置失败，回退到增量模式，使用保守的增量值
                RCLCPP_WARN(node_->get_logger(), "edg_get_stat failed (%d), using INCR mode", stat_ret);
                
                // 使用较大的增量（但不超过限位范围的一半）
                double max_range = (limits.max_pos - limits.min_pos) / 2.0;
                double direction = (req->velocity > 0) ? 1.0 : -1.0;
                double increment = direction * std::min(1.5, max_range);  // 最大1.5rad (~86度)
                
                target_joints[arm_idx].jVal[axis_index] = increment;
                modes[0] = modes[1] = MoveMode::INCR;
                
                RCLCPP_INFO(node_->get_logger(), "Continuous jog (INCR): axis=%d, increment=%.3f",
                            axis_index + 1, increment);
            }
            double vel[2] = {std::abs(req->velocity), std::abs(req->velocity)};
            double acc[2] = {3.0, 3.0};  // 适中的加速度
            
            ret = robot_->robot_run_multi_movj(req->robot_id, modes, FALSE, target_joints, vel, acc);
            
            // 启动超时检测定时器 (如果尚未启动)
            if (!jog_timeout_timer_) {
                jog_timeout_timer_ = node_->create_wall_timer(
                    std::chrono::milliseconds(100),  // 每100ms检测一次
                    std::bind(&JakaServiceHandlers::checkJogTimeout, this)
                );
            }
            
            if (ret != ERR_SUCC) {
                continuous_jog_active_ = false;
            }
        }
        
        res->success = (ret == ERR_SUCC);
        if (!res->success) {
            RCLCPP_ERROR(node_->get_logger(), "Jog failed with error code: %d", ret);
            res->message = "Jog command failed";
        } else {
            res->message = "Jog command sent";
        }
    } else {
        res->success = false;
        res->message = "Cartesian jog not implemented yet (only joint space supported)";
        RCLCPP_WARN(node_->get_logger(), "Cartesian jog not supported, use joint space (coord_type=1)");
    }
}

void JakaServiceHandlers::handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res)
{
    RCLCPP_INFO(node_->get_logger(), "Jog Stop: robot=%d axis=%d", req->robot_id, req->axis_num);
    
    // 停止连续Jog
    stopContinuousJog();
    
    res->success = true;
    res->message = "Jog stopped";
}

} // namespace qyh_jaka_control
