#ifndef JAKA_ROBOT_INTERFACE_HPP_
#define JAKA_ROBOT_INTERFACE_HPP_

#include <memory>
#include <string>
#include <JAKAZuRobot.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace jaka_control
{

/**
 * @brief 封装JAKA SDK的接口类
 * 
 * 该类提供了对JAKA双臂机器人SDK的C++封装，
 * 简化了在ROS 2环境中使用JAKA机器人的操作
 */
class JakaRobotInterface
{
public:
    /**
     * @brief 构造函数
     * @param logger ROS 2日志记录器
     */
    explicit JakaRobotInterface(rclcpp::Logger logger);
    
    /**
     * @brief 析构函数
     */
    ~JakaRobotInterface();

    /**
     * @brief 连接到机器人控制器
     * @param ip 控制器IP地址
     * @return true if successful
     */
    bool connect(const std::string& ip);

    /**
     * @brief 断开与控制器的连接
     * @return true if successful
     */
    bool disconnect();

    /**
     * @brief 上电
     * @return true if successful
     */
    bool powerOn();

    /**
     * @brief 下电
     * @return true if successful
     */
    bool powerOff();

    /**
     * @brief 使能机器人
     * @return true if successful
     */
    bool enableRobot();

    /**
     * @brief 下使能机器人
     * @return true if successful
     */
    bool disableRobot();

    /**
     * @brief 清除错误
     * @return true if successful
     */
    bool clearError();

    /**
     * @brief 终止当前运动
     * @return true if successful
     */
    bool motionAbort();

    /**
     * @brief 关节运动
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1, DUAL=-1)
     * @param joint_positions 关节位置数组 (14个关节: 左臂7个+右臂7个，单位：弧度)
     * @param move_mode 运动模式 (ABS=0, INCR=1)
     * @param velocity 速度 (弧度/秒)
     * @param acceleration 加速度 (弧度/秒²)
     * @param is_block 是否阻塞
     * @return true if successful
     */
    bool moveJ(int robot_id, const std::vector<double>& joint_positions,
               bool move_mode, double velocity, double acceleration, bool is_block);

    /**
     * @brief 笛卡尔空间直线运动
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1, DUAL=-1)
     * @param target_pose 目标位姿
     * @param move_mode 运动模式 (ABS=0, INCR=1)
     * @param velocity 速度 (毫米/秒)
     * @param acceleration 加速度 (毫米/秒²)
     * @param is_block 是否阻塞
     * @return true if successful
     */
    bool moveL(int robot_id, const geometry_msgs::msg::Pose& target_pose,
               bool move_mode, double velocity, double acceleration, bool is_block);

    /**
     * @brief 设置碰撞检测等级
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param level 碰撞等级 (0-5)
     * @return true if successful
     */
    bool setCollisionLevel(int robot_id, int level);

    /**
     * @brief 设置工具偏置
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param tool_offset 工具偏置位姿
     * @return true if successful
     */
    bool setToolOffset(int robot_id, const geometry_msgs::msg::Pose& tool_offset);

    /**
     * @brief 获取机器人状态
     * @param state 机器人状态结构体
     * @return true if successful
     */
    bool getRobotState(RobotState& state);

    /**
     * @brief 获取最后的错误信息
     * @param error_code 错误码结构体
     * @return true if successful
     */
    bool getLastError(ErrorCode& error_code);

    /**
     * @brief 正运动学：根据关节角度计算笛卡尔位姿
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param joint_pos 关节位置
     * @param cartesian_pose 输出的笛卡尔位姿
     * @return true if successful
     */
    bool kineForward(int robot_id, const JointValue& joint_pos, CartesianPose& cartesian_pose);

    /**
     * @brief 逆运动学：根据笛卡尔位姿计算关节角度
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param ref_pos 参考关节位置
     * @param cartesian_pose 目标笛卡尔位姿
     * @param joint_pos 输出的关节位置
     * @return true if successful
     */
    bool kineInverse(int robot_id, const JointValue& ref_pos, 
                     const CartesianPose& cartesian_pose, JointValue& joint_pos);

    /**
     * @brief 检查机器人是否处于错误状态
     * @param error 错误状态数组 [left, right]
     * @return true if successful
     */
    bool isInError(int error[2]);

    /**
     * @brief 检查机器人是否到位
     * @param inpos 到位状态数组 [left, right]
     * @return true if successful
     */
    bool isInPosition(int inpos[2]);

    /**
     * @brief 使能/关闭伺服运动模式
     * @param enable true: 进入伺服模式, false: 退出伺服模式
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1, DUAL=-1)
     * @return true if successful
     */
    bool servoMoveEnable(bool enable, int robot_id);

    /**
     * @brief 伺服关节运动
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1, DUAL=-1)
     * @param joint_positions 关节位置数组 (14个关节: 左臂7个+右臂7个)
     * @param move_mode 运动模式 (ABS=0, INCR=1)
     * @return true if successful
     */
    bool servoJ(int robot_id, const std::vector<double>& joint_positions, bool move_mode);

    /**
     * @brief 伺服笛卡尔空间运动
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param target_pose 目标位姿
     * @param move_mode 运动模式 (ABS=0, INCR=1)
     * @return true if successful
     */
    bool servoP(int robot_id, const geometry_msgs::msg::Pose& target_pose, bool move_mode);

    /**
     * @brief 设置伺服模式滤波器 - 不使用滤波器
     * @return true if successful
     */
    bool servoMoveUseNoneFilter();

    /**
     * @brief 设置伺服模式滤波器 - 关节空间一阶低通滤波
     * @param cutoff_freq 截止频率
     * @return true if successful
     */
    bool servoMoveUseJointLPF(double cutoff_freq);

    /**
     * @brief 设置伺服模式滤波器 - 关节空间非线性滤波
     * @param max_vr 姿态变化速度上限(°/s)
     * @param max_ar 姿态变化加速度上限(°/s²)
     * @param max_jr 姿态变化加加速度上限(°/s³)
     * @return true if successful
     */
    bool servoMoveUseJointNLF(double max_vr, double max_ar, double max_jr);

    /**
     * @brief 设置伺服模式滤波器 - 笛卡尔空间非线性滤波
     * @param max_vp 位置速度上限(mm/s)
     * @param max_ap 位置加速度上限(mm/s²)
     * @param max_jp 位置加加速度上限(mm/s³)
     * @param max_vr 姿态变化速度上限(°/s)
     * @param max_ar 姿态变化加速度上限(°/s²)
     * @param max_jr 姿态变化加加速度上限(°/s³)
     * @return true if successful
     */
    bool servoMoveUseCarteNLF(double max_vp, double max_ap, double max_jp,
                               double max_vr, double max_ar, double max_jr);

    /**
     * @brief 接收EtherCAT同步数据
     * @return true if successful
     */
    bool edgRecv(struct timespec *next = nullptr);

    /**
     * @brief 发送EtherCAT同步数据
     * @return true if successful
     */
    bool edgSend();

    /**
     * @brief 获取EtherCAT同步状态
     * @param robot_id 机器人ID (LEFT=0, RIGHT=1)
     * @param joint_pos 输出关节位置
     * @param cartesian_pose 输出笛卡尔位姿
     * @return true if successful
     */
    bool edgGetStat(int robot_id, JointValue& joint_pos, CartesianPose& cartesian_pose);

private:
    rclcpp::Logger logger_;
    std::unique_ptr<JAKAZuRobot> robot_;
    bool connected_;

    /**
     * @brief 检查SDK调用的返回值
     * @param ret SDK函数返回值
     * @param operation 操作名称
     * @return true if ret == ERR_SUCC
     */
    bool checkReturn(errno_t ret, const std::string& operation);

    /**
     * @brief 将ROS Pose转换为JAKA CartesianPose
     */
    CartesianPose rosPoseToJaka(const geometry_msgs::msg::Pose& ros_pose);

    /**
     * @brief 将JAKA CartesianPose转换为ROS Pose
     */
    geometry_msgs::msg::Pose jakaPoseToRos(const CartesianPose& jaka_pose);
};

} // namespace jaka_control

#endif // JAKA_ROBOT_INTERFACE_HPP_
