#include "qyh_jaka_control/jaka_interface.hpp"
#include <cstring>
#include <chrono>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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

    // Send to Left (0)
    errno_t ret = robot_->edg_servo_j(0, &jpos[0], is_abs ? MoveMode::ABS : MoveMode::INCR);
    if (ret != ERR_SUCC) return checkReturn(ret, "servo_j left");

    // Send to Right (1)
    ret = robot_->edg_servo_j(1, &jpos[1], is_abs ? MoveMode::ABS : MoveMode::INCR);
    return checkReturn(ret, "servo_j right");
}

bool JakaInterface::servoP(int robot_id, const geometry_msgs::msg::Pose& pose, bool is_abs)
{
    if (!servo_enabled_) {
        RCLCPP_ERROR(logger_, "Servo mode not enabled");
        return false;
    }

    CartesianPose jaka_pose = rosPoseToJaka(pose);
    errno_t ret = robot_->edg_servo_p((unsigned char)robot_id, &jaka_pose, is_abs ? MoveMode::ABS : MoveMode::INCR);
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

bool JakaInterface::edgRecv(struct timespec *next)
{
    errno_t ret = robot_->edg_recv(next);
    return checkReturn(ret, "edg_recv");
}

bool JakaInterface::edgSend()
{
    errno_t ret = robot_->edg_send();
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

    // Convert quaternion to Euler angles (ZYX convention)
    tf2::Quaternion q(ros_pose.orientation.x, ros_pose.orientation.y, 
                      ros_pose.orientation.z, ros_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    jaka_pose.rpy.rx = roll;
    jaka_pose.rpy.ry = pitch;
    jaka_pose.rpy.rz = yaw;

    return jaka_pose;
}

geometry_msgs::msg::Pose JakaInterface::jakaPoseToRos(const CartesianPose& jaka_pose)
{
    geometry_msgs::msg::Pose ros_pose;
    ros_pose.position.x = jaka_pose.tran.x / 1000.0;  // mm to m
    ros_pose.position.y = jaka_pose.tran.y / 1000.0;
    ros_pose.position.z = jaka_pose.tran.z / 1000.0;

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(jaka_pose.rpy.rx, jaka_pose.rpy.ry, jaka_pose.rpy.rz);
    
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();
    ros_pose.orientation.w = q.w();

    return ros_pose;
}

bool JakaInterface::checkReturn(errno_t ret, const std::string& operation)
{
    if (ret == ERR_SUCC) {
        return true;
    }
    
    // Provide detailed error messages
    std::string error_msg;
    switch (ret) {
        case -1: error_msg = "General error"; break;
        case -2: error_msg = "Invalid parameter"; break;
        case -3: error_msg = "Robot not ready or in error state"; break;
        case -4: error_msg = "Communication error"; break;
        case -5: error_msg = "Operation timeout"; break;
        case -6: error_msg = "Robot is not powered on"; break;
        case -7: error_msg = "Robot is not enabled"; break;
        case -8: error_msg = "Motion in progress"; break;
        case -9: error_msg = "Servo mode not enabled"; break;
        case -10: error_msg = "Robot in emergency stop"; break;
        default: error_msg = "Unknown error"; break;
    }
    
    RCLCPP_ERROR(logger_, "Operation '%s' failed with error code: %d (%s)", 
                 operation.c_str(), ret, error_msg.c_str());
    return false;
}

} // namespace qyh_jaka_control
