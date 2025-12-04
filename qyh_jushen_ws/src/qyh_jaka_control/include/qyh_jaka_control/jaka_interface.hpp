#ifndef QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_
#define QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
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

    // 点动控制 (Jog control) - 非伺服模式下使用
    // 注意：伺服模式下禁止使用点动功能
    //
    // 关节点动 (Joint Jog)
    // robot_id: 0=左臂, 1=右臂
    // joint_index: 0-6 对应 J1-J7
    // step_deg: 步进值(度), 正值正向，负值反向
    //           支持: 0.01, 0.05, 0.1, 0.5, 1, 5, 10
    //           特殊值 0 表示连续模式
    // velocity_percent: 速度百分比 1-100
    bool jogJoint(int robot_id, int joint_index, double step_deg, double velocity_percent = 30.0);
    
    // 笛卡尔点动 (Cartesian Jog)
    // robot_id: 0=左臂, 1=右臂
    // axis: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
    // step: 步进值, X/Y/Z单位mm, RX/RY/RZ单位度
    //       支持: 0.01, 0.05, 0.1, 0.5, 1, 5, 10
    //       特殊值 0 表示连续模式
    // velocity_percent: 速度百分比 1-100
    // coord_type: 0=基座标系(COORD_BASE), 2=工具坐标系(COORD_TOOL)
    bool jogCartesian(int robot_id, int axis, double step, double velocity_percent = 30.0, int coord_type = 0);
    
    // 连续点动 (Continuous Jog) - 按住持续运动
    // direction: 1=正向, -1=反向
    bool jogJointContinuous(int robot_id, int joint_index, int direction, double velocity_percent = 30.0);
    bool jogCartesianContinuous(int robot_id, int axis, int direction, double velocity_percent = 30.0, int coord_type = 0);
    
    // 停止点动
    bool jogStop(int robot_id);
    
    // 检查是否可以执行点动（非伺服模式）
    bool canJog() const { return connected_ && !servo_enabled_; }

    // 配置功能
    bool setCollisionLevel(int robot_id, int level);
    bool setToolOffset(int robot_id, const geometry_msgs::msg::Pose& tool_offset);
    
    // 负载管理
    // mass: 质量(kg), centroid_x: 质心X位置(mm), 默认150mm
    bool setPayload(int robot_id, double mass, double centroid_x = 150.0);
    bool getPayload(int robot_id, double& mass, double& centroid_x, double& centroid_y, double& centroid_z);

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
    
    // 点动状态
    std::atomic<bool> jog_active_{false};
    std::atomic<bool> jog_stop_requested_{false};
    int jog_robot_id_{-1};
    int jog_axis_num_{0};
    int jog_coord_type_{0};
    int jog_direction_{0};
    double jog_velocity_percent_{30.0};
    std::thread jog_thread_;
    
    // 点动内部实现
    void jogContinuousThread();
    double getStepValue(int step_mode);  // 获取步进值

    bool checkReturn(errno_t ret, const std::string& operation);
};

} // namespace qyh_jaka_control

#endif // QYH_JAKA_CONTROL_JAKA_INTERFACE_HPP_
