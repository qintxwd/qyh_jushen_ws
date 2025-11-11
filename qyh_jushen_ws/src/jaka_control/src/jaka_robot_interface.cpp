#include "jaka_control/jaka_robot_interface.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace jaka_control
{

JakaRobotInterface::JakaRobotInterface(rclcpp::Logger logger)
    : logger_(logger), connected_(false)
{
    robot_ = std::make_unique<JAKAZuRobot>();
}

JakaRobotInterface::~JakaRobotInterface()
{
    if (connected_) {
        disconnect();
    }
}

bool JakaRobotInterface::connect(const std::string& ip)
{
    errno_t ret = robot_->login_in(ip.c_str());
    if (checkReturn(ret, "login_in")) {
        connected_ = true;
        RCLCPP_INFO(logger_, "Successfully connected to robot at %s", ip.c_str());
        return true;
    }
    return false;
}

bool JakaRobotInterface::disconnect()
{
    errno_t ret = robot_->login_out();
    if (checkReturn(ret, "login_out")) {
        connected_ = false;
        RCLCPP_INFO(logger_, "Disconnected from robot");
        return true;
    }
    return false;
}

bool JakaRobotInterface::powerOn()
{
    errno_t ret = robot_->power_on();
    return checkReturn(ret, "power_on");
}

bool JakaRobotInterface::powerOff()
{
    errno_t ret = robot_->power_off();
    return checkReturn(ret, "power_off");
}

bool JakaRobotInterface::enableRobot()
{
    errno_t ret = robot_->enable_robot();
    return checkReturn(ret, "enable_robot");
}

bool JakaRobotInterface::disableRobot()
{
    errno_t ret = robot_->disable_robot();
    return checkReturn(ret, "disable_robot");
}

bool JakaRobotInterface::clearError()
{
    errno_t ret = robot_->clear_error();
    return checkReturn(ret, "clear_error");
}

bool JakaRobotInterface::motionAbort()
{
    errno_t ret = robot_->motion_abort();
    return checkReturn(ret, "motion_abort");
}

bool JakaRobotInterface::moveJ(int robot_id, const std::vector<double>& joint_positions,
                                bool move_mode, double velocity, double acceleration, bool is_block)
{
    if (joint_positions.size() != 14) {
        RCLCPP_ERROR(logger_, "Invalid joint positions size: %zu, expected 14 (7 for left + 7 for right)", 
                     joint_positions.size());
        return false;
    }

    JointValue jpos[2];
    memset(&jpos, 0, sizeof(jpos));
    
    // 填充左臂关节位置 (0-6)
    for (size_t i = 0; i < 7; ++i) {
        jpos[0].jVal[i] = joint_positions[i];
    }
    
    // 填充右臂关节位置 (7-13)
    for (size_t i = 0; i < 7; ++i) {
        jpos[1].jVal[i] = joint_positions[i + 7];
    }

    MoveMode moveop[2] = {move_mode ? INCR : ABS, move_mode ? INCR : ABS};
    double vel[2] = {velocity, velocity};
    double acc[2] = {acceleration, acceleration};

    errno_t ret = robot_->robot_run_multi_movj(robot_id, moveop, is_block, jpos, vel, acc);
    return checkReturn(ret, "robot_run_multi_movj");
}

bool JakaRobotInterface::moveL(int robot_id, const geometry_msgs::msg::Pose& target_pose,
                                bool move_mode, double velocity, double acceleration, bool is_block)
{
    CartesianPose cpos[2];
    memset(&cpos, 0, sizeof(cpos));
    
    // 转换ROS Pose到JAKA CartesianPose
    int idx = (robot_id == LEFT) ? 0 : (robot_id == RIGHT) ? 1 : 0;
    cpos[idx] = rosPoseToJaka(target_pose);
    
    if (robot_id == DUAL) {
        // 对于双臂，复制到第二个机器人
        memcpy(&cpos[1], &cpos[0], sizeof(CartesianPose));
    }

    MoveMode moveop[2] = {move_mode ? INCR : ABS, move_mode ? INCR : ABS};
    double vel[2] = {velocity, velocity};
    double acc[2] = {acceleration, acceleration};

    errno_t ret = robot_->robot_run_multi_movl(robot_id, moveop, is_block, cpos, vel, acc);
    return checkReturn(ret, "robot_run_multi_movl");
}

bool JakaRobotInterface::setCollisionLevel(int robot_id, int level)
{
    if (level < 0 || level > 5) {
        RCLCPP_ERROR(logger_, "Invalid collision level: %d, must be 0-5", level);
        return false;
    }

    errno_t ret = robot_->set_collision_level(robot_id, level);
    return checkReturn(ret, "set_collision_level");
}

bool JakaRobotInterface::setToolOffset(int robot_id, const geometry_msgs::msg::Pose& tool_offset)
{
    CartesianPose offset = rosPoseToJaka(tool_offset);
    errno_t ret = robot_->robot_set_tool_offset(robot_id, offset);
    return checkReturn(ret, "robot_set_tool_offset");
}

bool JakaRobotInterface::getRobotState(RobotState& state)
{
    errno_t ret = robot_->get_robot_state(&state);
    return checkReturn(ret, "get_robot_state");
}

bool JakaRobotInterface::getLastError(ErrorCode& error_code)
{
    errno_t ret = robot_->get_last_error(&error_code);
    return checkReturn(ret, "get_last_error");
}

bool JakaRobotInterface::kineForward(int robot_id, const JointValue& joint_pos, CartesianPose& cartesian_pose)
{
    errno_t ret = robot_->kine_forward(robot_id, &joint_pos, &cartesian_pose);
    return checkReturn(ret, "kine_forward");
}

bool JakaRobotInterface::kineInverse(int robot_id, const JointValue& ref_pos,
                                      const CartesianPose& cartesian_pose, JointValue& joint_pos)
{
    errno_t ret = robot_->kine_inverse(robot_id, &ref_pos, &cartesian_pose, &joint_pos);
    return checkReturn(ret, "kine_inverse");
}

bool JakaRobotInterface::isInError(int error[2])
{
    errno_t ret = robot_->robot_is_in_error(error);
    return checkReturn(ret, "robot_is_in_error");
}

bool JakaRobotInterface::isInPosition(int inpos[2])
{
    errno_t ret = robot_->robot_is_inpos(inpos);
    return checkReturn(ret, "robot_is_inpos");
}

bool JakaRobotInterface::servoMoveEnable(bool enable, int robot_id)
{
    errno_t ret = robot_->servo_move_enable(enable, robot_id);
    return checkReturn(ret, "servo_move_enable");
}

bool JakaRobotInterface::servoJ(int robot_id, const std::vector<double>& joint_positions, bool move_mode)
{
    if (joint_positions.size() != 14) {
        RCLCPP_ERROR(logger_, "Invalid joint positions size: %zu, expected 14 (7 for left + 7 for right)", 
                     joint_positions.size());
        return false;
    }

    JointValue jpos[2];
    memset(&jpos, 0, sizeof(jpos));
    
    // 填充左臂关节位置 (0-6)
    for (size_t i = 0; i < 7; ++i) {
        jpos[0].jVal[i] = joint_positions[i];
    }
    
    // 填充右臂关节位置 (7-13)
    for (size_t i = 0; i < 7; ++i) {
        jpos[1].jVal[i] = joint_positions[i + 7];
    }

    MoveMode mode = move_mode ? INCR : ABS;
    
    // 根据robot_id选择要发送的数据
    if (robot_id == LEFT) {
        errno_t ret = robot_->edg_servo_j(LEFT, &jpos[0], mode);
        return checkReturn(ret, "edg_servo_j");
    } else if (robot_id == RIGHT) {
        errno_t ret = robot_->edg_servo_j(RIGHT, &jpos[1], mode);
        return checkReturn(ret, "edg_servo_j");
    } else {
        // DUAL模式：需要分别发送
        errno_t ret1 = robot_->edg_servo_j(LEFT, &jpos[0], mode);
        errno_t ret2 = robot_->edg_servo_j(RIGHT, &jpos[1], mode);
        return checkReturn(ret1, "edg_servo_j(left)") && checkReturn(ret2, "edg_servo_j(right)");
    }
}

bool JakaRobotInterface::servoP(int robot_id, const geometry_msgs::msg::Pose& target_pose, bool move_mode)
{
    CartesianPose cpos = rosPoseToJaka(target_pose);
    MoveMode mode = move_mode ? INCR : ABS;
    
    errno_t ret = robot_->edg_servo_p(robot_id, &cpos, mode);
    return checkReturn(ret, "edg_servo_p");
}

bool JakaRobotInterface::servoMoveUseNoneFilter()
{
    errno_t ret = robot_->servo_move_use_none_filter();
    return checkReturn(ret, "servo_move_use_none_filter");
}

bool JakaRobotInterface::servoMoveUseJointLPF(double cutoff_freq)
{
    errno_t ret = robot_->servo_move_use_joint_LPF(cutoff_freq);
    return checkReturn(ret, "servo_move_use_joint_LPF");
}

bool JakaRobotInterface::servoMoveUseJointNLF(double max_vr, double max_ar, double max_jr)
{
    errno_t ret = robot_->servo_move_use_joint_NLF(max_vr, max_ar, max_jr);
    return checkReturn(ret, "servo_move_use_joint_NLF");
}

bool JakaRobotInterface::servoMoveUseCarteNLF(double max_vp, double max_ap, double max_jp,
                                               double max_vr, double max_ar, double max_jr)
{
    errno_t ret = robot_->servo_move_use_carte_NLF(max_vp, max_ap, max_jp, max_vr, max_ar, max_jr);
    return checkReturn(ret, "servo_move_use_carte_NLF");
}

bool JakaRobotInterface::edgRecv(struct timespec *next)
{
    errno_t ret = robot_->edg_recv(next);
    return checkReturn(ret, "edg_recv");
}

bool JakaRobotInterface::edgSend()
{
    errno_t ret = robot_->edg_send();
    return checkReturn(ret, "edg_send");
}

bool JakaRobotInterface::edgGetStat(int robot_id, JointValue& joint_pos, CartesianPose& cartesian_pose)
{
    errno_t ret = robot_->edg_get_stat(robot_id, &joint_pos, &cartesian_pose);
    return checkReturn(ret, "edg_get_stat");
}

bool JakaRobotInterface::checkReturn(errno_t ret, const std::string& operation)
{
    if (ret == ERR_SUCC) {
        return true;
    }
    
    RCLCPP_ERROR(logger_, "Operation '%s' failed with error code: 0x%x", 
                 operation.c_str(), ret);
    
    // 尝试获取详细错误信息
    ErrorCode error_code;
    if (robot_->get_last_error(&error_code) == ERR_SUCC && error_code.code != 0) {
        RCLCPP_ERROR(logger_, "Error details: [0x%lx] %s", error_code.code, error_code.message);
    }
    
    return false;
}

CartesianPose JakaRobotInterface::rosPoseToJaka(const geometry_msgs::msg::Pose& ros_pose)
{
    CartesianPose jaka_pose;
    
    // 位置转换：ROS使用米，JAKA使用毫米
    jaka_pose.tran.x = ros_pose.position.x * 1000.0;
    jaka_pose.tran.y = ros_pose.position.y * 1000.0;
    jaka_pose.tran.z = ros_pose.position.z * 1000.0;
    
    // 姿态转换：四元数转欧拉角
    tf2::Quaternion q(
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z,
        ros_pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    jaka_pose.rpy.rx = roll;
    jaka_pose.rpy.ry = pitch;
    jaka_pose.rpy.rz = yaw;
    
    return jaka_pose;
}

geometry_msgs::msg::Pose JakaRobotInterface::jakaPoseToRos(const CartesianPose& jaka_pose)
{
    geometry_msgs::msg::Pose ros_pose;
    
    // 位置转换：JAKA使用毫米，ROS使用米
    ros_pose.position.x = jaka_pose.tran.x / 1000.0;
    ros_pose.position.y = jaka_pose.tran.y / 1000.0;
    ros_pose.position.z = jaka_pose.tran.z / 1000.0;
    
    // 姿态转换：欧拉角转四元数
    tf2::Quaternion q;
    q.setRPY(jaka_pose.rpy.rx, jaka_pose.rpy.ry, jaka_pose.rpy.rz);
    
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();
    ros_pose.orientation.w = q.w();
    
    return ros_pose;
}

} // namespace jaka_control
