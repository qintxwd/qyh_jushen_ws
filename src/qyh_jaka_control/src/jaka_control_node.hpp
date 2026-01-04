#include "jaka_service_handlers.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>
#include <deque>
#include <algorithm>
#include <qyh_jaka_control_msgs/srv/start_servo.hpp>
#include <qyh_jaka_control_msgs/srv/stop_servo.hpp>
#include <qyh_jaka_control_msgs/srv/move_j.hpp>
#include <qyh_jaka_control_msgs/srv/move_l.hpp>
#include <qyh_jaka_control_msgs/srv/set_tool_offset.hpp>
#include <qyh_jaka_control_msgs/srv/set_payload.hpp>
#include <qyh_jaka_control_msgs/srv/get_payload.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <array>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <sstream>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

// struct jog_joint_info{
//     int robot_id;// robot_id: 0=左臂, 1=右臂
//     int joint_index;// joint_index: 0-6 对应 J1-J7
//     double step_degree;// step_deg: 步进值(度), 正值正向，负值反向
//     //           支持: 0.01, 0.05, 0.1, 0.5, 1, 5, 10
//     //           特殊值 0 表示连续模式
//     double velocity_percent; // velocity_percent: 速度百分比 1-100
// };

// struct jog_cartesian_info{
//     int robot_id;// robot_id: 0=左臂, 1=右臂
//     int axis; // axis: 0=X, 1=Y, 2=Z, 3=RX, 4=RY, 5=RZ
//     double step; // 步进值, X/Y/Z单位mm, RX/RY/RZ单位度
//     //       支持: 0.01, 0.05, 0.1, 0.5, 1, 5, 10
//     //       特殊值 0 表示连续模式
//     double velocity_percent; // velocity_percent: 速度百分比 1-100
// };

// struct jog_info{
//     bool is_joint; // true: joint jog, false: cartesian jog
//     jog_joint_info joint_info;
//     jog_cartesian_info cartesian_info;
//     CartesianPose last_cartesian_pose;
//     double last_joint_positions[JAKA_ROBOT_MAX_JOINT];
// };

class JakaControlNode : public rclcpp::Node
{
public:
    JakaControlNode();

    ~JakaControlNode();
private:
    
    void left_timer_callback();

    void right_timer_callback();

    void command_p_timer_callback();

    // void jog_timer_callback();

    void command_p_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void publishStatus();

    // ==================== VR目标位姿回调函数 ====================
    void leftServoPCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    void rightServoPCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ==================== 负载配置加载 ====================
    bool loadAndSetPayloadFromConfig();    

    bool startServoInternal();
    
    bool stopServoInternal();

private:
    std::shared_ptr<JAKAZuRobot> robot_;
    std::unique_ptr<qyh_jaka_control::JakaServiceHandlers> service_handlers_;
    
    // 参数
    std::string robot_ip_;
    double cycle_time_{0.008};               // 控制周期 (s)

    enum class ServoInputState { NEVER_RECEIVED, ACTIVE, HOLD, TIMEOUT };
    //记录收到的左右手servo p指令，用于合并后发送【当收到时，状态变为ACTIVE】
    CartesianPose left_command_servo_p_val;
    CartesianPose right_command_servo_p_val;
    //记录收到的左右手servo p指令的时间戳
    rclcpp::Time left_last_command_p_time_;
    rclcpp::Time right_last_command_p_time_;   
    // 将短期“仍为ACTIVE”的窗口（秒）和总体超时时间区分开
    rclcpp::Duration active_window_ = rclcpp::Duration::from_seconds(0.05);
    // Per-arm input state (NEVER_RECEIVED/ACTIVE/HOLD/TIMEOUT)
    ServoInputState left_input_state_ = ServoInputState::NEVER_RECEIVED;
    ServoInputState right_input_state_ = ServoInputState::NEVER_RECEIVED;
    // Last target pose remembered when ACTIVE -> used for HOLD
    CartesianPose left_last_target_pose_;
    CartesianPose right_last_target_pose_;
    // 新增：命令过期时间（ms），超过则认为数据可能已过期
    rclcpp::Duration command_timeout_ = rclcpp::Duration::from_seconds(0.2);

    // 状态
    std::atomic<bool> connected_;
    std::atomic<bool> powered_;
    std::atomic<bool> enabled_;
    std::atomic<bool> servo_running_;

    std::vector<std::string> left_joint_names_;
    std::vector<std::string> right_joint_names_;
    std::vector<double> msg_pre_left_joint_state;
    std::vector<double> msg_pre_right_joint_state;
    JointValue cached_left_joints_;
    JointValue cached_right_joints_;
    CartesianPose cached_left_pose_;
    CartesianPose cached_right_pose_;
    bool has_left_cached_state_{false};
    bool has_right_cached_state_{false};

    // jog_info joging_info_;

    // 机器人状态发布
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
    // 关节状态发布
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_joint_state_pub_;
    //  TCP位姿发布
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_tcp_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_tcp_pose_pub_;
    // torque力矩发布
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_torque_sensor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_torque_sensor_pub_;
    // VR遥操作订阅
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_vr_servo_p_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_vr_servo_p_sub_;
    
    //自订自发的topic，用于合并左右手的数据
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr dual_arm_command_p_publisher_; //用定时器发送servo p指令[以固定的8ms频率发送]，将收到的左右手指令合并后发送出去
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr dual_arm_command_p_subscriber_; //接收上面的8ms频率的servo p指令，具体进行执行

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_on_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_off_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_error_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_motion_abort_;    
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartServo>::SharedPtr srv_start_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopServo>::SharedPtr srv_stop_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetPayload>::SharedPtr srv_set_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetPayload>::SharedPtr srv_get_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::Jog>::SharedPtr srv_jog_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::JogStop>::SharedPtr srv_jog_stop_;
    // 定时器
    rclcpp::TimerBase::SharedPtr left_timer_;
    rclcpp::TimerBase::SharedPtr right_timer_;
    // rclcpp::TimerBase::SharedPtr command_j_timer_;
    rclcpp::TimerBase::SharedPtr command_p_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    // rclcpp::TimerBase::SharedPtr jog_timer_;    
};

