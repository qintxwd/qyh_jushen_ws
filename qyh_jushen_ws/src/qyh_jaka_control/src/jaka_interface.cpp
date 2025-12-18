#include "qyh_jaka_control/jaka_interface.hpp"
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "jkerr.h"  // JAKA 错误码定义

namespace qyh_jaka_control
{

JakaInterface::JakaInterface(rclcpp::Logger logger)
    : logger_(logger), connected_(false), servo_enabled_(false)
{
    robot_ = std::make_unique<JAKAZuRobot>();
}

JakaInterface::~JakaInterface()
{
    if (connected_) {
        if (servo_enabled_) {
            servoMoveEnable(false);
        }
        disconnect();
    }
}

bool JakaInterface::connect(const std::string& ip)
{
    errno_t ret = robot_->login_in(ip.c_str());
    if (checkReturn(ret, "login_in")) {
        connected_ = true;
        RCLCPP_INFO(logger_, "Successfully connected to robot at %s", ip.c_str());
        return true;
    }
    return false;
}

bool JakaInterface::disconnect()
{
    errno_t ret = robot_->login_out();
    if (checkReturn(ret, "login_out")) {
        connected_ = false;
        servo_enabled_ = false;
        RCLCPP_INFO(logger_, "Disconnected from robot");
        return true;
    }
    return false;
}

bool JakaInterface::powerOn()
{
    // Check if robot is in error state
    int error[2] = {0, 0};
    errno_t ret = robot_->robot_is_in_error(error);
    if (ret == ERR_SUCC && (error[0] != 0 || error[1] != 0)) {
        RCLCPP_WARN(logger_, "Robot in error state (left:%d, right:%d), clearing errors before power on", error[0], error[1]);
        robot_->clear_error();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Check if already powered
    RobotState state;
    ret = robot_->get_robot_state(&state);
    if (ret == ERR_SUCC && state.poweredOn) {
        RCLCPP_INFO(logger_, "Robot is already powered on");
        return true;
    }

    ret = robot_->power_on();
    return checkReturn(ret, "power_on");
}

bool JakaInterface::powerOff()
{
    errno_t ret = robot_->power_off();
    return checkReturn(ret, "power_off");
}

bool JakaInterface::enableRobot()
{
    errno_t ret = robot_->enable_robot();
    return checkReturn(ret, "enable_robot");
}

bool JakaInterface::disableRobot()
{
    errno_t ret = robot_->disable_robot();
    return checkReturn(ret, "disable_robot");
}

bool JakaInterface::clearError()
{
    errno_t ret = robot_->clear_error();
    return checkReturn(ret, "clear_error");
}

bool JakaInterface::motionAbort()
{
    errno_t ret = robot_->motion_abort();
    return checkReturn(ret, "motion_abort");
}

bool JakaInterface::servoMoveEnable(bool enable, int robot_id)
{
    errno_t ret = robot_->servo_move_enable(enable, robot_id);
    if (checkReturn(ret, enable ? "servo_move_enable" : "servo_move_disable")) {
        servo_enabled_ = enable;
        return true;
    }
    return false;
}

bool JakaInterface::servoJ(int robot_id, const std::vector<double>& joint_positions, bool is_abs)
{
    if (!servo_enabled_) {
        RCLCPP_ERROR(logger_, "Servo mode not enabled");
        return false;
    }

    if (joint_positions.size() != 14) {
        RCLCPP_ERROR(logger_, "Invalid joint positions size: %zu, expected 14", joint_positions.size());
        return false;
    }

    JointValue jpos[2];
    memset(&jpos, 0, sizeof(jpos));
    
    for (size_t i = 0; i < 7; ++i) {
        jpos[0].jVal[i] = joint_positions[i];
        jpos[1].jVal[i] = joint_positions[i + 7];
    }

    // Prepare servo commands for both arms
    errno_t ret = robot_->edg_servo_j(0, &jpos[0], is_abs ? MoveMode::ABS : MoveMode::INCR);
    if (ret != ERR_SUCC) return checkReturn(ret, "servo_j left");

    ret = robot_->edg_servo_j(1, &jpos[1], is_abs ? MoveMode::ABS : MoveMode::INCR);
    if (ret != ERR_SUCC) return checkReturn(ret, "servo_j right");

    // NOTE: Caller must call edgSend() to actually send commands
    return true;
}

bool JakaInterface::servoP(int robot_id, const geometry_msgs::msg::Pose& pose, bool is_abs)
{
    if (!servo_enabled_) {
        RCLCPP_ERROR(logger_, "Servo mode not enabled");
        return false;
    }

    CartesianPose jaka_pose = rosPoseToJaka(pose);
    errno_t ret = robot_->edg_servo_p((unsigned char)robot_id, &jaka_pose, is_abs ? MoveMode::ABS : MoveMode::INCR);
    
    // NOTE: Caller must call edgSend() to actually send commands
    return checkReturn(ret, "servo_p");
}

bool JakaInterface::setFilterNone()
{
    errno_t ret = robot_->servo_move_use_none_filter();
    return checkReturn(ret, "servo_move_use_none_filter");
}

bool JakaInterface::setFilterJointLPF(double cutoff_freq)
{
    errno_t ret = robot_->servo_move_use_joint_LPF(cutoff_freq);
    return checkReturn(ret, "servo_move_use_joint_LPF");
}

bool JakaInterface::setFilterJointNLF(double max_vr, double max_ar, double max_jr)
{
    errno_t ret = robot_->servo_move_use_joint_NLF(max_vr, max_ar, max_jr);
    return checkReturn(ret, "servo_move_use_joint_NLF");
}

bool JakaInterface::setFilterCarteNLF(double max_vp, double max_ap, double max_jp,
                                       double max_vr, double max_ar, double max_jr)
{
    errno_t ret = robot_->servo_move_use_carte_NLF(max_vp, max_ap, max_jp, max_vr, max_ar, max_jr);
    return checkReturn(ret, "servo_move_use_carte_NLF");
}

// Note: edg_recv() is deprecated in SDK 2.3.0.12+
// edg_get_stat() no longer depends on edg_recv()

bool JakaInterface::edgSend(uint32_t* cmd_index)
{
    errno_t ret = robot_->edg_send(cmd_index);
    return checkReturn(ret, "edg_send");
}

bool JakaInterface::edgGetStat(int robot_id, JointValue& joint_pos, CartesianPose& cartesian_pose)
{
    errno_t ret = robot_->edg_get_stat((unsigned char)robot_id, &joint_pos, &cartesian_pose);
    return checkReturn(ret, "edg_get_stat");
}

bool JakaInterface::edgServoJ(int robot_id, const JointValue& joint_pos, bool is_abs)
{
    errno_t ret = robot_->edg_servo_j((unsigned char)robot_id, &joint_pos, is_abs ? MoveMode::ABS : MoveMode::INCR);
    return checkReturn(ret, "edg_servo_j");
}

bool JakaInterface::edgServoP(int robot_id, const CartesianPose& pose, bool is_abs)
{
    errno_t ret = robot_->edg_servo_p((unsigned char)robot_id, &pose, is_abs ? MoveMode::ABS : MoveMode::INCR);
    return checkReturn(ret, "edg_servo_p");
}

bool JakaInterface::kineForward(int robot_id, const JointValue& joint_pos, CartesianPose& cartesian_pose)
{
    errno_t ret = robot_->kine_forward(robot_id, &joint_pos, &cartesian_pose);
    return checkReturn(ret, "kine_forward");
}

bool JakaInterface::kineInverse(int robot_id, const JointValue& ref_pos, 
                                const CartesianPose& cartesian_pose, JointValue& joint_pos)
{
    errno_t ret = robot_->kine_inverse(robot_id, &ref_pos, &cartesian_pose, &joint_pos);
    return checkReturn(ret, "kine_inverse");
}

bool JakaInterface::getRobotState(RobotState& state)
{
    errno_t ret = robot_->get_robot_state(&state);
    return checkReturn(ret, "get_robot_state");
}

bool JakaInterface::getLastError(ErrorCode& error_code)
{
    errno_t ret = robot_->get_last_error(&error_code);
    return checkReturn(ret, "get_last_error");
}

bool JakaInterface::isInError(int error[2])
{
    errno_t ret = robot_->robot_is_in_error(error);
    return checkReturn(ret, "is_in_error");
}

bool JakaInterface::isInPosition(int inpos[2])
{
    errno_t ret = robot_->robot_is_inpos(inpos);
    return checkReturn(ret, "is_in_pos");
}

bool JakaInterface::getJointPositions(int robot_id, JointValue& joint_pos)
{
    CartesianPose temp_pose;
    errno_t ret = robot_->edg_get_stat((unsigned char)robot_id, &joint_pos, &temp_pose);
    return checkReturn(ret, "get_joint_position");
}

bool JakaInterface::getCartesianPose(int robot_id, CartesianPose& cartesian_pose)
{
    JointValue temp_joint;
    errno_t ret = robot_->edg_get_stat((unsigned char)robot_id, &temp_joint, &cartesian_pose);
    return checkReturn(ret, "get_tcp_position");
}

bool JakaInterface::moveJ(int robot_id, const std::vector<double>& joint_positions,
                          bool move_mode, double velocity, double acceleration, bool is_block)
{
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
    return checkReturn(ret, "move_j");
}

bool JakaInterface::moveL(int robot_id, const geometry_msgs::msg::Pose& target_pose,
                          bool move_mode, double velocity, double acceleration, bool is_block)
{
    CartesianPose poses[2];
    memset(&poses, 0, sizeof(poses));
    poses[0] = rosPoseToJaka(target_pose);
    poses[1] = poses[0];
    
    MoveMode modes[2] = {move_mode ? MoveMode::INCR : MoveMode::ABS, 
                         move_mode ? MoveMode::INCR : MoveMode::ABS};
    double vel[2] = {velocity, velocity};
    double acc[2] = {acceleration, acceleration};
    
    errno_t ret = robot_->robot_run_multi_movl(robot_id, modes, is_block, poses, vel, acc);
    return checkReturn(ret, "move_l");
}

// ==================== 点动控制实现 (非伺服模式) ====================

// 步进值映射: 0=连续, 1=0.01, 2=0.05, 3=0.1, 4=0.5, 5=1, 6=5, 7=10
static const double JOG_STEP_VALUES[] = {0.0, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0};
static const int JOG_STEP_COUNT = 8;

double JakaInterface::getStepValue(int step_mode)
{
    if (step_mode < 0 || step_mode >= JOG_STEP_COUNT) {
        return 1.0;  // 默认1度或1mm
    }
    return JOG_STEP_VALUES[step_mode];
}

bool JakaInterface::jogJoint(int robot_id, int joint_index, double step_deg, double velocity_percent)
{
    // 检查前置条件
    if (!connected_) {
        RCLCPP_ERROR(logger_, "Jog failed: not connected to robot");
        return false;
    }
    
    if (servo_enabled_) {
        RCLCPP_ERROR(logger_, "Jog failed: servo mode is enabled, please disable servo mode first");
        return false;
    }
    
    if (joint_index < 0 || joint_index > 6) {
        RCLCPP_ERROR(logger_, "Invalid joint index: %d, must be 0-6", joint_index);
        return false;
    }
    
    if (velocity_percent < 1.0) velocity_percent = 1.0;
    if (velocity_percent > 100.0) velocity_percent = 100.0;
    
    // 获取当前关节位置
    JointValue current_jpos;
    CartesianPose temp_pose;
    errno_t ret = robot_->edg_get_stat((unsigned char)robot_id, &current_jpos, &temp_pose);
    if (ret != ERR_SUCC) {
        RCLCPP_ERROR(logger_, "Failed to get current joint position");
        return false;
    }
    
    // 计算目标位置 (step_deg 已经是度，转换为弧度)
    double step_rad = step_deg * M_PI / 180.0;
    JointValue target_jpos = current_jpos;
    target_jpos.jVal[joint_index] += step_rad;
    
    RCLCPP_INFO(logger_, "JogJoint: robot=%d, J%d, step=%.3f deg (%.5f rad), vel=%.1f%%",
                robot_id, joint_index + 1, step_deg, step_rad, velocity_percent);
    
    // 使用 robot_run_multi_movj 执行运动
    // 只控制单个机器人，另一个机器人位置保持不变
    JointValue jpos_arr[2];
    MoveMode modes[2] = {MoveMode::ABS, MoveMode::ABS};
    double vel[2] = {velocity_percent / 100.0, velocity_percent / 100.0};  // 速度比例
    double acc[2] = {0.5, 0.5};  // 加速度，点动时用较小值更平滑
    
    // 获取另一个机器人的当前位置
    int other_robot = (robot_id == 0) ? 1 : 0;
    JointValue other_jpos;
    CartesianPose other_pose;
    ret = robot_->edg_get_stat((unsigned char)other_robot, &other_jpos, &other_pose);
    if (ret != ERR_SUCC) {
        RCLCPP_WARN(logger_, "Failed to get other robot position, using zeros");
        memset(&other_jpos, 0, sizeof(other_jpos));
    }
    
    if (robot_id == 0) {
        jpos_arr[0] = target_jpos;
        jpos_arr[1] = other_jpos;
    } else {
        jpos_arr[0] = other_jpos;
        jpos_arr[1] = target_jpos;
    }
    
    // 非阻塞执行，让调用方可以控制停止
    ret = robot_->robot_run_multi_movj(robot_id, modes, FALSE, jpos_arr, vel, acc);
    return checkReturn(ret, "jog_joint");
}

bool JakaInterface::jogCartesian(int robot_id, int axis, double step, double velocity_percent, int coord_type)
{
    // 检查前置条件
    if (!connected_) {
        RCLCPP_ERROR(logger_, "Jog failed: not connected to robot");
        return false;
    }
    
    if (servo_enabled_) {
        RCLCPP_ERROR(logger_, "Jog failed: servo mode is enabled, please disable servo mode first");
        return false;
    }
    
    if (axis < 0 || axis > 5) {
        RCLCPP_ERROR(logger_, "Invalid axis: %d, must be 0-5 (X/Y/Z/RX/RY/RZ)", axis);
        return false;
    }
    
    if (velocity_percent < 1.0) velocity_percent = 1.0;
    if (velocity_percent > 100.0) velocity_percent = 100.0;
    
    // 获取当前笛卡尔位置和关节位置
    JointValue current_jpos;
    CartesianPose current_pose;
    errno_t ret = robot_->edg_get_stat((unsigned char)robot_id, &current_jpos, &current_pose);
    if (ret != ERR_SUCC) {
        RCLCPP_ERROR(logger_, "Failed to get current position");
        return false;
    }
    
    // 计算目标笛卡尔位置
    CartesianPose target_pose = current_pose;
    
    // axis: 0=X, 1=Y, 2=Z (mm), 3=RX, 4=RY, 5=RZ (度转弧度)
    switch (axis) {
        case 0: target_pose.tran.x += step; break;  // X (mm)
        case 1: target_pose.tran.y += step; break;  // Y (mm)
        case 2: target_pose.tran.z += step; break;  // Z (mm)
        case 3: target_pose.rpy.rx += step * M_PI / 180.0; break;  // RX (deg to rad)
        case 4: target_pose.rpy.ry += step * M_PI / 180.0; break;  // RY (deg to rad)
        case 5: target_pose.rpy.rz += step * M_PI / 180.0; break;  // RZ (deg to rad)
    }
    
    const char* axis_names[] = {"X", "Y", "Z", "RX", "RY", "RZ"};
    const char* unit = (axis < 3) ? "mm" : "deg";
    RCLCPP_INFO(logger_, "JogCartesian: robot=%d, axis=%s, step=%.3f %s, vel=%.1f%%, coord=%d",
                robot_id, axis_names[axis], step, unit, velocity_percent, coord_type);
    
    // 逆运动学求解目标关节位置
    JointValue target_jpos;
    ret = robot_->kine_inverse(robot_id, &current_jpos, &target_pose, &target_jpos);
    if (ret != ERR_SUCC) {
        RCLCPP_ERROR(logger_, "Inverse kinematics failed, target pose may be unreachable");
        return false;
    }
    
    // 使用 robot_run_multi_movj 执行运动
    JointValue jpos_arr[2];
    MoveMode modes[2] = {MoveMode::ABS, MoveMode::ABS};
    double vel[2] = {velocity_percent / 100.0, velocity_percent / 100.0};
    double acc[2] = {0.5, 0.5};
    
    // 获取另一个机器人的当前位置
    int other_robot = (robot_id == 0) ? 1 : 0;
    JointValue other_jpos;
    CartesianPose other_pose;
    ret = robot_->edg_get_stat((unsigned char)other_robot, &other_jpos, &other_pose);
    if (ret != ERR_SUCC) {
        RCLCPP_WARN(logger_, "Failed to get other robot position, using zeros");
        memset(&other_jpos, 0, sizeof(other_jpos));
    }
    
    if (robot_id == 0) {
        jpos_arr[0] = target_jpos;
        jpos_arr[1] = other_jpos;
    } else {
        jpos_arr[0] = other_jpos;
        jpos_arr[1] = target_jpos;
    }
    
    ret = robot_->robot_run_multi_movj(robot_id, modes, FALSE, jpos_arr, vel, acc);
    return checkReturn(ret, "jog_cartesian");
}

bool JakaInterface::jogJointContinuous(int robot_id, int joint_index, int direction, double velocity_percent)
{
    // 检查前置条件
    if (!connected_) {
        RCLCPP_ERROR(logger_, "Jog failed: not connected to robot");
        return false;
    }
    
    if (servo_enabled_) {
        RCLCPP_ERROR(logger_, "Jog failed: servo mode is enabled");
        return false;
    }
    
    if (joint_index < 0 || joint_index > 6) {
        RCLCPP_ERROR(logger_, "Invalid joint index: %d", joint_index);
        return false;
    }
    
    if (direction != 1 && direction != -1) {
        RCLCPP_ERROR(logger_, "Invalid direction: %d, must be 1 or -1", direction);
        return false;
    }
    
    // 如果已经在点动，先停止
    if (jog_active_) {
        jogStop(robot_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 设置点动参数
    jog_robot_id_ = robot_id;
    jog_axis_num_ = joint_index;
    jog_coord_type_ = 1;  // COORD_JOINT
    jog_direction_ = direction;
    jog_velocity_percent_ = velocity_percent;
    jog_stop_requested_ = false;
    jog_active_ = true;
    
    // 启动连续点动线程
    if (jog_thread_.joinable()) {
        jog_thread_.join();
    }
    jog_thread_ = std::thread(&JakaInterface::jogContinuousThread, this);
    
    RCLCPP_INFO(logger_, "Started continuous jog: robot=%d, J%d, dir=%d, vel=%.1f%%",
                robot_id, joint_index + 1, direction, velocity_percent);
    
    return true;
}

bool JakaInterface::jogCartesianContinuous(int robot_id, int axis, int direction, double velocity_percent, int coord_type)
{
    // 检查前置条件
    if (!connected_) {
        RCLCPP_ERROR(logger_, "Jog failed: not connected to robot");
        return false;
    }
    
    if (servo_enabled_) {
        RCLCPP_ERROR(logger_, "Jog failed: servo mode is enabled");
        return false;
    }
    
    if (axis < 0 || axis > 5) {
        RCLCPP_ERROR(logger_, "Invalid axis: %d", axis);
        return false;
    }
    
    if (direction != 1 && direction != -1) {
        RCLCPP_ERROR(logger_, "Invalid direction: %d", direction);
        return false;
    }
    
    // 如果已经在点动，先停止
    if (jog_active_) {
        jogStop(robot_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 设置点动参数
    jog_robot_id_ = robot_id;
    jog_axis_num_ = axis;
    jog_coord_type_ = coord_type;  // 0=BASE, 2=TOOL
    jog_direction_ = direction;
    jog_velocity_percent_ = velocity_percent;
    jog_stop_requested_ = false;
    jog_active_ = true;
    
    // 启动连续点动线程
    if (jog_thread_.joinable()) {
        jog_thread_.join();
    }
    jog_thread_ = std::thread(&JakaInterface::jogContinuousThread, this);
    
    const char* axis_names[] = {"X", "Y", "Z", "RX", "RY", "RZ"};
    RCLCPP_INFO(logger_, "Started continuous jog: robot=%d, %s, dir=%d, vel=%.1f%%, coord=%d",
                robot_id, axis_names[axis], direction, velocity_percent, coord_type);
    
    return true;
}

void JakaInterface::jogContinuousThread()
{
    // 连续点动：每次移动一小步，循环执行直到停止请求
    const double JOINT_STEP_DEG = 2.0;    // 关节每次移动2度
    const double LINEAR_STEP_MM = 5.0;    // 线性每次移动5mm
    const double ANGULAR_STEP_DEG = 2.0;  // 旋转每次移动2度
    const int CYCLE_MS = 100;             // 每100ms执行一次
    
    RCLCPP_INFO(logger_, "Continuous jog thread started");
    
    while (!jog_stop_requested_ && connected_ && !servo_enabled_) {
        bool success = false;
        
        if (jog_coord_type_ == 1) {
            // 关节点动
            double step = JOINT_STEP_DEG * jog_direction_;
            success = jogJoint(jog_robot_id_, jog_axis_num_, step, jog_velocity_percent_);
        } else {
            // 笛卡尔点动
            double step;
            if (jog_axis_num_ < 3) {
                step = LINEAR_STEP_MM * jog_direction_;  // X/Y/Z
            } else {
                step = ANGULAR_STEP_DEG * jog_direction_;  // RX/RY/RZ
            }
            success = jogCartesian(jog_robot_id_, jog_axis_num_, step, jog_velocity_percent_, jog_coord_type_);
        }
        
        if (!success) {
            RCLCPP_WARN(logger_, "Jog step failed, stopping continuous jog");
            break;
        }
        
        // 等待一个周期
        std::this_thread::sleep_for(std::chrono::milliseconds(CYCLE_MS));
    }
    
    jog_active_ = false;
    RCLCPP_INFO(logger_, "Continuous jog thread ended");
}

bool JakaInterface::jogStop(int robot_id)
{
    RCLCPP_INFO(logger_, "Jog stop requested for robot %d", robot_id);
    
    // 请求停止连续点动线程
    jog_stop_requested_ = true;
    
    // 等待线程结束
    if (jog_thread_.joinable()) {
        jog_thread_.join();
    }
    
    // 调用 motion_abort 立即停止当前运动
    errno_t ret = robot_->motion_abort();
    
    jog_active_ = false;
    return checkReturn(ret, "jog_stop");
}

bool JakaInterface::setCollisionLevel(int robot_id, int level)
{
    if (level < 0 || level > 5) {
        RCLCPP_ERROR(logger_, "Invalid collision level: %d, must be 0-5", level);
        return false;
    }
    errno_t ret = robot_->set_collision_level(robot_id, level);
    return checkReturn(ret, "set_collision_level");
}

bool JakaInterface::setToolOffset(int robot_id, const geometry_msgs::msg::Pose& tool_offset)
{
    CartesianPose jaka_offset = rosPoseToJaka(tool_offset);
    errno_t ret = robot_->robot_set_tool_offset(robot_id, jaka_offset);
    return checkReturn(ret, "set_tool_offset");
}

CartesianPose JakaInterface::rosPoseToJaka(const geometry_msgs::msg::Pose& ros_pose)
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

geometry_msgs::msg::Pose JakaInterface::jakaPoseToRos(const CartesianPose& jaka_pose)
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

bool JakaInterface::setPayload(int robot_id, double mass, double centroid_x)
{
    PayLoad payload;
    payload.mass = mass;
    payload.centroid.x = centroid_x;  // 默认150mm (15cm向前)
    payload.centroid.y = 0.0;
    payload.centroid.z = 0.0;
    
    RCLCPP_INFO(logger_, "Setting payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                robot_id, mass, centroid_x, payload.centroid.y, payload.centroid.z);
    
    errno_t ret = robot_->robot_set_tool_payload(robot_id, &payload);
    return checkReturn(ret, "set_payload");
}

bool JakaInterface::getPayload(int robot_id, double& mass, double& centroid_x, double& centroid_y, double& centroid_z)
{
    // Note: JAKA SDK get_payload may not support robot_id parameter
    // This is a limitation of the SDK
    PayLoad payload;
    errno_t ret = robot_->robot_get_tool_payload(&payload);
    
    if (ret == ERR_SUCC) {
        mass = payload.mass;
        centroid_x = payload.centroid.x;
        centroid_y = payload.centroid.y;
        centroid_z = payload.centroid.z;
        RCLCPP_INFO(logger_, "Got payload for robot %d: mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                    robot_id, mass, centroid_x, centroid_y, centroid_z);
        return true;
    }
    return checkReturn(ret, "get_payload");
}

bool JakaInterface::checkReturn(errno_t ret, const std::string& operation)
{
    if (ret == ERR_SUCC) {
        return true;
    }
    
    // Provide detailed error messages (根据 jkerr.h)
    std::string error_msg;
    switch (ret) {
        case ERR_INVALID_HANDLER: error_msg = "Invalid handler"; break;  // -1
        case ERR_INVALID_PARAMETER: error_msg = "Invalid parameter"; break;  // -2
        case ERR_COMMUNICATION_ERR: error_msg = "Communication error"; break;  // -3
        case ERR_KINE_INVERSE_ERR: error_msg = "IK failed (out of workspace or singular)"; break;  // -4
        case ERR_EMERGENCY_PRESSED: error_msg = "Emergency stop pressed"; break;  // -5
        case ERR_NOT_POWERED: error_msg = "Robot not powered on"; break;  // -6
        case ERR_NOT_ENABLED: error_msg = "Robot not enabled"; break;  // -7
        case ERR_DISABLE_SERVOMODE: error_msg = "Servo mode not enabled"; break;  // -8
        case ERR_NOT_OFF_ENABLE: error_msg = "Robot not disabled"; break;  // -9
        case ERR_PROGRAM_IS_RUNNING: error_msg = "Program is running"; break;  // -10
        case ERR_CANNOT_OPEN_FILE: error_msg = "Cannot open file"; break;  // -11
        case ERR_MOTION_ABNORMAL: error_msg = "Motion abnormal"; break;  // -12
        case ERR_FTP_PREFROM: error_msg = "FTP error"; break;  // -14
        case ERR_VALUE_OVERSIZE: error_msg = "Value oversize"; break;  // -15
        case ERR_TROQUE_CONTROL_NOT_ENABLE: error_msg = "Torque control not enabled"; break;  // -22
        case ERR_ROBOT_NOT_STOPPED: error_msg = "Robot not stopped"; break;  // -23
        case ERR_INVERSE_OUT_OF_LIMIT: error_msg = "IK out of soft limit"; break;  // -24
        case ROBOT_IN_ERROR: error_msg = "Robot in error state - check teach pendant"; break;  // 0xFF
        default: error_msg = "Unknown error (check teach pendant)"; break;
    }
    
    RCLCPP_ERROR(logger_, "Operation '%s' failed with error code: %d (%s)", 
                 operation.c_str(), ret, error_msg.c_str());
    
    // Additional hint for error 255
    if (ret == 255 || ret < -10) {
        RCLCPP_WARN(logger_, "Hint: Try calling clear_error service first, or check the robot teach pendant for error details");
    }
    
    return false;
}

} // namespace qyh_jaka_control
