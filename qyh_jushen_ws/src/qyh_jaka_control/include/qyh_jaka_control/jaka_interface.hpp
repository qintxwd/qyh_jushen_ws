#ifndef QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_
#define QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <JAKAZuRobot.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace qyh_jaka_control
{

/**
 * @brief JAKA双臂机器人接口封装
 * 提供对JAKA SDK的线程安全封装和高级功能
 */
class JakaInterface
{
public:
    explicit JakaInterface(rclcpp::Logger logger);
    ~JakaInterface();

    // 基础连接功能
    bool connect(const std::string& ip);
    bool disconnect();
    bool isConnected() const { return connected_; }

    // 电源和使能
    bool powerOn();
    bool powerOff();
    bool enableRobot();
    bool disableRobot();
    bool clearError();
    bool motionAbort();

    // 伺服模式控制
    bool servoMoveEnable(bool enable, int robot_id = -1);
    bool servoJ(int robot_id, const std::vector<double>& joint_positions, bool is_abs);
    bool servoP(int robot_id, const geometry_msgs::msg::Pose& pose, bool is_abs);
    
    // 滤波器设置
    bool setFilterNone();
    bool setFilterJointLPF(double cutoff_freq);
    bool setFilterJointNLF(double max_vr, double max_ar, double max_jr);
    bool setFilterCarteNLF(double max_vp, double max_ap, double max_jp,
                           double max_vr, double max_ar, double max_jr);

    // EtherCAT同步模式 (SDK 2.3.0.13+)
    // Note: edg_recv() is deprecated in SDK 2.3.0.12+
    bool edgSend();
    bool edgGetStat(int robot_id, JointValue& joint_pos, CartesianPose& cartesian_pose);
    bool edgServoJ(int robot_id, const JointValue& joint_pos, bool is_abs);
    bool edgServoP(int robot_id, const CartesianPose& pose, bool is_abs);

    // 运动学
    bool kineForward(int robot_id, const JointValue& joint_pos, CartesianPose& cartesian_pose);
    bool kineInverse(int robot_id, const JointValue& ref_pos, 
                     const CartesianPose& cartesian_pose, JointValue& joint_pos);

    // 点到点运动
    bool moveJ(int robot_id, const std::vector<double>& joint_positions,
               bool move_mode, double velocity, double acceleration, bool is_block);
    bool moveL(int robot_id, const geometry_msgs::msg::Pose& target_pose,
               bool move_mode, double velocity, double acceleration, bool is_block);

    // 配置功能
    bool setCollisionLevel(int robot_id, int level);
    bool setToolOffset(int robot_id, const geometry_msgs::msg::Pose& tool_offset);

    // 状态查询
    bool getRobotState(RobotState& state);
    bool getLastError(ErrorCode& error_code);
    bool isInError(int error[2]);
    bool isInPosition(int inpos[2]);
    bool getJointPositions(int robot_id, JointValue& joint_pos);
    bool getCartesianPose(int robot_id, CartesianPose& cartesian_pose);

    // 辅助函数：ROS消息转换
    CartesianPose rosPoseToJaka(const geometry_msgs::msg::Pose& ros_pose);
    geometry_msgs::msg::Pose jakaPoseToRos(const CartesianPose& jaka_pose);

private:
    rclcpp::Logger logger_;
    std::unique_ptr<JAKAZuRobot> robot_;
    bool connected_;
    bool servo_enabled_;

    bool checkReturn(errno_t ret, const std::string& operation);
};

} // namespace qyh_jaka_control

#endif // QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_
