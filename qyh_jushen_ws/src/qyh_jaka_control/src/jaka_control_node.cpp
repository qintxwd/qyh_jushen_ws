#include "qyh_jaka_control/jaka_interface.hpp"
#include "qyh_jaka_control/velocity_servo_controller.hpp"
#include "qyh_jaka_control/jaka_service_handlers.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_servo_status.hpp>
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

// ==================== IK求解相关定义 ====================
// JAKA Zu7 关节限位和速度限制
struct JointLimits {
    double pos_min;  // 负限位（弧度）
    double pos_max;  // 正限位（弧度）
    double vel_max;  // 速度限制（弧度/秒）
};

const std::array<JointLimits, 7> JAKA_ZU7_LIMITS = {{
    {-6.2832, 6.2832, 1.5708},   // 关节1: ±360°, 90°/s
    {-1.8326, 1.8326, 1.5708},   // 关节2: ±105°, 90°/s
    {-6.2832, 6.2832, 2.0944},   // 关节3: ±360°, 120°/s
    {-2.5307, 0.5236, 2.0944},   // 关节4: -145°~30°, 120°/s
    {-6.2832, 6.2832, 2.6180},   // 关节5: ±360°, 150°/s
    {-1.8326, 1.8326, 2.6180},   // 关节6: ±105°, 150°/s
    {-6.2832, 6.2832, 2.6180}    // 关节7: ±360°, 150°/s
}};

const double SAFETY_MARGIN_POS = 0.0873;  // 5° 安全裕度
const double SAFETY_MARGIN_VEL = 0.8;     // 安全起见，削减20%

// 归一化角度到[-π, π]范围
static inline double normalizeAngle(double angle) {
    return std::fmod(angle + M_PI, 2.0*M_PI) - M_PI;
}

/**
 * @brief JAKA双臂机器人统一控制节点
 * 
 * 集成了原 jaka_control_node 和 jaka_bridge_node 的功能：
 * 1. 基础控制：上电、下电、使能、去使能、清除错误、急停
 * 2. 伺服模式：支持直接指令(JakaDualJointServo)和桥接指令(JointState)
 * 3. 轨迹平滑：集成 SmoothServoBridge
 * 4. 高级功能：MoveJ, MoveL, Jog, Payload
 * 5. 笛卡尔遥操作：接收VR位姿 → JAKA IK → 平滑 → Servo
 */
class JakaControlNode : public rclcpp::Node
{
public:
    JakaControlNode() 
        : Node("jaka_control_node"),
          jaka_interface_(this->get_logger())
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("cycle_time_ms", 8.0); // 默认 125Hz
        declare_parameter<bool>("visualization_only", false); // 仅可视化模式，不发送给真实机器人
        
        // 获取参数
        robot_ip_ = get_parameter("robot_ip").as_string();
        cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();
        visualization_only_ = get_parameter("visualization_only").as_bool();
        
        if (visualization_only_) {
            RCLCPP_WARN(get_logger(), "========================================");
            RCLCPP_WARN(get_logger(), "  VISUALIZATION ONLY MODE ENABLED");
            RCLCPP_WARN(get_logger(), "  Commands will NOT be sent to real robot");
            RCLCPP_WARN(get_logger(), "  Only publishing to /joint_states for RViz");
            RCLCPP_WARN(get_logger(), "========================================");
        }

        // Publishers
        status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>("/jaka/servo/status", 10);
        robot_state_pub_ = create_publisher<qyh_jaka_control_msgs::msg::RobotState>("/jaka/robot_state", 10);
        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 速度控制模式参数
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        declare_parameter<bool>("ik_solver.has_z_offset", true);
        declare_parameter<double>("ik_solver.left_z_offset", 0.219885132);
        declare_parameter<double>("ik_solver.right_z_offset", 0.217950931);
        
        // 声明速度控制器参数（供 VelocityServoController 使用）
        declare_parameter<double>("velocity_control.dt", 0.008);
        declare_parameter<double>("velocity_control.linear_gain", 2.0);
        declare_parameter<double>("velocity_control.angular_gain", 1.0);
        declare_parameter<double>("velocity_control.max_linear_vel", 0.5);
        declare_parameter<double>("velocity_control.max_angular_vel", 1.0);
        declare_parameter<double>("velocity_control.joint_vel_limit", 1.5);
        declare_parameter<double>("velocity_control.q_dot_min", 1e-4);
        declare_parameter<double>("velocity_control.max_delta_q", 0.02);
        declare_parameter<double>("velocity_control.max_joint_accel", 50.0);  // rad/s²
        declare_parameter<double>("velocity_control.lambda_min", 1e-4);
        declare_parameter<double>("velocity_control.position_deadzone", 0.001);
        declare_parameter<double>("velocity_control.orientation_deadzone", 0.017);
        
        // 目标变化死区：过滤VR手柄的微小抖动，避免不必要的指令更新
        declare_parameter<double>("velocity_control.target_change_position_threshold", 0.002);  // 2mm
        declare_parameter<double>("velocity_control.target_change_orientation_threshold", 0.035); // ~2°
        
        // 关节限位（默认值，会在 initVelocityControllers 中设置）
        std::vector<double> default_joint_min(7, -6.2832);
        std::vector<double> default_joint_max(7, 6.2832);
        declare_parameter<std::vector<double>>("velocity_control.joint_pos_min", default_joint_min);
        declare_parameter<std::vector<double>>("velocity_control.joint_pos_max", default_joint_max);
        
        has_z_offset_ = get_parameter("ik_solver.has_z_offset").as_bool();
        left_z_offset_ = get_parameter("ik_solver.left_z_offset").as_double();
        right_z_offset_ = get_parameter("ik_solver.right_z_offset").as_double();
        
        target_change_pos_threshold_ = get_parameter("velocity_control.target_change_position_threshold").as_double();
        target_change_ori_threshold_ = get_parameter("velocity_control.target_change_orientation_threshold").as_double();
        
        // 🎯 新增参数：VR目标更新频率和周期
        declare_parameter<double>("teleop_target_update_time_ms", 66.0);
        declare_parameter<double>("velocity_control.target_update_dt", 0.066);
        
        teleop_target_update_time_ms_ = get_parameter("teleop_target_update_time_ms").as_double();
        
        // 速度控制器将在构造函数完成后初始化（避免shared_from_this()问题）
        RCLCPP_INFO(get_logger(), "🎯 速度积分控制模式已启用");
        
        // 初始化TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 订阅VR目标位姿
        left_vr_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/left_hand/target", qos,
            std::bind(&JakaControlNode::leftVRTargetCallback, this, std::placeholders::_1));
        
        right_vr_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/right_hand/target", qos,
            std::bind(&JakaControlNode::rightVRTargetCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "  ✓ TF监听器已初始化");
        RCLCPP_INFO(get_logger(), "  ✓ 订阅VR目标位姿话题");
        RCLCPP_INFO(get_logger(), "  ⏳ 速度控制器将在节点完全初始化后创建");

        // 创建服务处理器（负责所有服务回调）
        service_handlers_ = std::make_unique<qyh_jaka_control::JakaServiceHandlers>(
            this,
            jaka_interface_,
            connected_,
            powered_,
            enabled_,
            servo_running_,
            std::bind(&JakaControlNode::startServoInternal, this),
            std::bind(&JakaControlNode::stopServoInternal, this)
        );

        // Services (Basic Control)
        srv_power_on_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/power_on",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handlePowerOn, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
        
        srv_power_off_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/power_off",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handlePowerOff, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_enable_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/enable",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleEnable, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_disable_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/disable",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleDisable, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_clear_error_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/clear_error",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleClearError, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_motion_abort_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/motion_abort",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleMotionAbort, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));

        // Services (Servo Control)
        srv_start_servo_ = create_service<qyh_jaka_control_msgs::srv::StartServo>(
            "/jaka/servo/start",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleStartServo, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_stop_servo_ = create_service<qyh_jaka_control_msgs::srv::StopServo>(
            "/jaka/servo/stop",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleStopServo, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));

        // Services (Bridge Compatibility)
        srv_bridge_start_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/start_servo",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleBridgeStartServo, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_bridge_stop_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/stop_servo",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleBridgeStopServo, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));

        // Services (Motion & Others)
        srv_move_j_ = create_service<qyh_jaka_control_msgs::srv::MoveJ>(
            "/jaka/move_j",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleMoveJ, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
        
        srv_move_l_ = create_service<qyh_jaka_control_msgs::srv::MoveL>(
            "/jaka/move_l",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleMoveL, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
            
        srv_set_tool_offset_ = create_service<qyh_jaka_control_msgs::srv::SetToolOffset>(
            "/jaka/set_tool_offset",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleSetToolOffset, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));

        srv_set_payload_ = create_service<qyh_jaka_control_msgs::srv::SetPayload>(
            "/jaka/set_payload",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleSetPayload, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));
        
        srv_get_payload_ = create_service<qyh_jaka_control_msgs::srv::GetPayload>(
            "/jaka/get_payload",
            std::bind(&qyh_jaka_control::JakaServiceHandlers::handleGetPayload, service_handlers_.get(), 
                      std::placeholders::_1, std::placeholders::_2));

        // Initialization
        connected_ = false;
        powered_ = false;
        enabled_ = false;
        servo_running_ = false;

        // 按照官方示例程序的标准初始化流程
        RCLCPP_INFO(get_logger(), "Connecting to robot at %s...", robot_ip_.c_str());
        if (jaka_interface_.connect(robot_ip_)) {
            connected_ = true;
            RCLCPP_INFO(get_logger(), "✓ Connected to robot");
            
            // 1. 清除错误
            RCLCPP_INFO(get_logger(), "Clearing errors...");
            jaka_interface_.clearError();
            
            // 2. 关闭伺服模式（确保从干净状态开始）
            RCLCPP_INFO(get_logger(), "Disabling servo mode...");
            jaka_interface_.servoMoveEnable(false, -1);
            
            // 3. 设置滤波器为none
            RCLCPP_INFO(get_logger(), "Setting filter to none...");
            jaka_interface_.setFilterNone();
            
            // 🔧 新增：在上电前设置负载
            RCLCPP_INFO(get_logger(), "Loading and setting payload configuration...");
            loadAndSetPayloadFromConfig();
            
            // 4. 上电
            RCLCPP_INFO(get_logger(), "Powering on...");
            if (jaka_interface_.powerOn()) {
                powered_ = true;
                RCLCPP_INFO(get_logger(), "✓ Powered on");
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to power on");
            }
            
            // 5. 使能
            RCLCPP_INFO(get_logger(), "Enabling robot...");
            if (jaka_interface_.enableRobot()) {
                enabled_ = true;
                RCLCPP_INFO(get_logger(), "✓ Robot enabled");
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to enable robot");
            }
            
            // 6. 停止当前动作
            RCLCPP_INFO(get_logger(), "Aborting any motion...");
            jaka_interface_.motionAbort();
            
            // 7. 等待5秒让机器人稳定
            RCLCPP_INFO(get_logger(), "Waiting 5 seconds for robot to stabilize...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
            
            RCLCPP_INFO(get_logger(), "✓ Robot initialization complete and ready");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to connect to robot at %s", robot_ip_.c_str());
        }

        // Timers
        auto period = std::chrono::microseconds(static_cast<int>(cycle_time_ms_ * 1000));
        main_timer_ = create_wall_timer(period, std::bind(&JakaControlNode::mainLoop, this));
        status_timer_ = create_wall_timer(33ms, std::bind(&JakaControlNode::publishStatus, this));
        
        RCLCPP_INFO(get_logger(), "=== JAKA Unified Control Node Initialized ===");
    }

    ~JakaControlNode()
    {
        if (servo_running_) {
            stopServoInternal();
        }
        if (connected_) {
            jaka_interface_.disconnect();
        }
    }
    
    // 在构造函数完成后调用，初始化速度控制器
    void initVelocityControllers() {
        
        RCLCPP_INFO(get_logger(), "[初始化] 创建速度控制器...");
        
        // 使用ROS2包查找机制获取URDF路径
        std::string urdf_path;
        try {
            std::string package_path = ament_index_cpp::get_package_share_directory("qyh_dual_arms_description");
            urdf_path = package_path + "/urdf/dual_arms.urdf";
            RCLCPP_INFO(get_logger(), "[初始化] URDF路径: %s", urdf_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[初始化] 找不到包 qyh_dual_arms_description: %s", e.what());
            return;
        }
        
        left_vel_controller_ = std::make_unique<qyh_jaka_control::VelocityServoController>(shared_from_this(), "left");
        if (!left_vel_controller_->initialize(urdf_path, "base_link_left", "forward_lt")) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize left velocity controller");
        }
        
        right_vel_controller_ = std::make_unique<qyh_jaka_control::VelocityServoController>(shared_from_this(), "right");
        if (!right_vel_controller_->initialize(urdf_path, "base_link_right", "forward_rt")) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize right velocity controller");
        }
        
        // 📋 统一设置关节限位（从 JAKA_ZU7_LIMITS 应用安全裕度）
        std::vector<double> joint_min(7), joint_max(7), joint_vel_limit(7);
        for (int i = 0; i < 7; ++i) {
            joint_min[i] = JAKA_ZU7_LIMITS[i].pos_min + SAFETY_MARGIN_POS;
            joint_max[i] = JAKA_ZU7_LIMITS[i].pos_max - SAFETY_MARGIN_POS;
            joint_vel_limit[i] = JAKA_ZU7_LIMITS[i].vel_max * SAFETY_MARGIN_VEL;
        }
        if (left_vel_controller_) left_vel_controller_->setJointLimits(joint_min, joint_max, joint_vel_limit);
        if (right_vel_controller_) right_vel_controller_->setJointLimits(joint_min, joint_max, joint_vel_limit);
        
        RCLCPP_INFO(get_logger(), "✓ 速度伺服控制器已初始化");
        RCLCPP_INFO(get_logger(), "  has_z_offset=%s", has_z_offset_ ? "true" : "false");
    }

private:
    // ==================== 主循环 ====================
    void mainLoop()
    {
        // 🔍 看门狗日志：确认主循环是否存活
        static int loop_watchdog = 0;
        if (++loop_watchdog % 125 == 0) { // 每秒打印一次
             RCLCPP_INFO(get_logger(), "[MainLoop] Alive. ServoRunning: %d", servo_running_.load());
        }

        auto start = std::chrono::high_resolution_clock::now();
        
        // ⚡ 始终获取当前机械臂位姿（参考30.edgservo.cpp）
        // 无论servo是否运行，都需要更新状态缓存供publishStatus使用
        jaka_interface_.getJointPositions(0, cached_left_joints_);
        jaka_interface_.getJointPositions(1, cached_right_joints_);
        jaka_interface_.getCartesianPose(0, cached_left_pose_);
        jaka_interface_.getCartesianPose(1, cached_right_pose_);
        has_cached_state_ = true;

        // 如果伺服未运行，只更新状态缓存，不执行伺服命令
        if (!servo_running_) {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, 
                "[MainLoop] Servo not running, only updating state cache");
            return;
        }

        // 🔍 调试日志：检查控制器状态
        if (!left_vel_controller_ || !right_vel_controller_) {
             RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "[MainLoop] Controllers not initialized!");
             return;
        }

        RCLCPP_DEBUG_ONCE(get_logger(), "[MainLoop] First servo cycle");
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[MainLoop] Running at %.1f Hz (cycle: %.2f ms)", 
            1000.0 / cycle_time_ms_, cycle_time_ms_);
        
        // 速度积分控制模式：计算下一个关节指令
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Computing velocity commands...");
        std::vector<double> left_cmd, right_cmd;
        bool has_left = false;
        bool has_right = false;
        
        // 左臂
        if (left_vel_controller_) {
            std::vector<double> current_left(7);
            for(int i=0; i<7; ++i) current_left[i] = cached_left_joints_.jVal[i];
            left_vel_controller_->updateRobotState(current_left);
            
            has_left = left_vel_controller_->computeNextCommand(left_cmd);
            
            // 限位检查：防止超出关节限位
            if (has_left) {
                JointValue tmp_jv = convertToJointValue(left_cmd);
                if (!checkJointLimits(tmp_jv, "Left")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[Safety] Left command exceeds joint limits, using current position");
                    left_cmd = current_left;
                }
            }
            
            // 关键：即使没有目标，也要发送当前位置保持连接
            if (!has_left) {
                left_cmd = current_left;
                has_left = true; 
            }
            
            // 🔍 调试：打印当前值、目标值和差值
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Left] Current: [%f, %f, %f, %f, %f, %f, %f]",
                current_left[0], current_left[1], current_left[2], current_left[3],
                current_left[4], current_left[5], current_left[6]);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Left] Command: [%f, %f, %f, %f, %f, %f, %f]",
                left_cmd[0], left_cmd[1], left_cmd[2], left_cmd[3],
                left_cmd[4], left_cmd[5], left_cmd[6]);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Left] Delta:   [%f, %f, %f, %f, %f, %f, %f] (rad)",
                left_cmd[0]-current_left[0], left_cmd[1]-current_left[1], 
                left_cmd[2]-current_left[2], left_cmd[3]-current_left[3],
                left_cmd[4]-current_left[4], left_cmd[5]-current_left[5], 
                left_cmd[6]-current_left[6]);
        }
        
        // 右臂
        if (right_vel_controller_) {
            std::vector<double> current_right(7);
            for(int i=0; i<7; ++i) current_right[i] = cached_right_joints_.jVal[i];
            right_vel_controller_->updateRobotState(current_right);
            
            has_right = right_vel_controller_->computeNextCommand(right_cmd);
            
            // 限位检查：防止超出关节限位
            if (has_right) {
                JointValue tmp_jv = convertToJointValue(right_cmd);
                if (!checkJointLimits(tmp_jv, "Right")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[Safety] Right command exceeds joint limits, using current position");
                    right_cmd = current_right;
                }
            }
            
            if (!has_right) {
                right_cmd = current_right;
                has_right = true;
            }
            
            // 🔍 调试：打印当前值、目标值和差值
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Right] Current: [%f, %f, %f, %f, %f, %f, %f]",
                current_right[0], current_right[1], current_right[2], current_right[3],
                current_right[4], current_right[5], current_right[6]);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Right] Command: [%f, %f, %f, %f, %f, %f, %f]",
                right_cmd[0], right_cmd[1], right_cmd[2], right_cmd[3],
                right_cmd[4], right_cmd[5], right_cmd[6]);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[Right] Delta:   [%f, %f, %f, %f, %f, %f, %f] (rad)",
                right_cmd[0]-current_right[0], right_cmd[1]-current_right[1], 
                right_cmd[2]-current_right[2], right_cmd[3]-current_right[3],
                right_cmd[4]-current_right[4], right_cmd[5]-current_right[5], 
                right_cmd[6]-current_right[6]);
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] has_left=%d, has_right=%d", has_left, has_right);
        
        if (has_left || has_right) {
            // ========== 可视化模式：发布joint_states到RViz ==========
            if (visualization_only_) {
                auto joint_state_msg = sensor_msgs::msg::JointState();
                joint_state_msg.header.stamp = now();
                joint_state_msg.header.frame_id = "world";
                
                // 关节名称
                for (int i = 1; i <= 7; ++i) {
                    joint_state_msg.name.push_back("left_joint" + std::to_string(i));
                }
                for (int i = 1; i <= 7; ++i) {
                    joint_state_msg.name.push_back("right_joint" + std::to_string(i));
                }
                
                // 关节位置（使用伺服指令）
                if (has_left) {
                    joint_state_msg.position.insert(joint_state_msg.position.end(), 
                                                     left_cmd.begin(), left_cmd.end());
                } else {
                    // 左臂无新指令，使用零位或上次位置
                    joint_state_msg.position.insert(joint_state_msg.position.end(), 7, 0.0);
                }
                
                if (has_right) {
                    joint_state_msg.position.insert(joint_state_msg.position.end(), 
                                                     right_cmd.begin(), right_cmd.end());
                } else {
                    // 右臂无新指令，使用零位或上次位置
                    joint_state_msg.position.insert(joint_state_msg.position.end(), 7, 0.0);
                }
                
                joint_states_pub_->publish(joint_state_msg);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                    "[Visualization] Published joint_states to RViz (L:%d R:%d)", has_left, has_right);
            }
            // ========== 真实机器人模式：发送指令 ==========
            else {
                // 独立发送左臂指令
                if (has_left) {
                    RCLCPP_DEBUG_ONCE(get_logger(), "[MainLoop] First left command - preparing edgServoJ");
                    auto jv = convertToJointValue(left_cmd);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "[Bridge] Left: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        jv.jVal[0], jv.jVal[1], jv.jVal[2], jv.jVal[3], jv.jVal[4], jv.jVal[5], jv.jVal[6]);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgServoJ(0)...");
                    if (!jaka_interface_.edgServoJ(0, jv, true)) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
                            "[MainLoop] Failed to send left arm command, aborting this cycle");
                        return;  // 立即停止，避免发送半成命令
                    }
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgServoJ(0) returned");
                }
                
                // 独立发送右臂指令
                if (has_right) {
                    RCLCPP_DEBUG_ONCE(get_logger(), "[MainLoop] First right command - preparing edgServoJ");
                    auto jv = convertToJointValue(right_cmd);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "[Bridge] Right: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        jv.jVal[0], jv.jVal[1], jv.jVal[2], jv.jVal[3], jv.jVal[4], jv.jVal[5], jv.jVal[6]);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgServoJ(1)...");
                    if (!jaka_interface_.edgServoJ(1, jv, true)) {
                        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
                            "[MainLoop] Failed to send right arm command, aborting this cycle");
                        return;  // 立即停止，避免发送半成命令
                    }
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgServoJ(1) returned");
                }
                
                // 🔍 调试日志：打印每一帧发送给机械臂的关节值
                // 仅在调试模式下开启，避免刷屏
                static int log_counter = 0;
                if (++log_counter % 10 == 0) { // 每10帧打印一次，约12.5Hz
                    // std::string left_cmd_str = "[";
                    // std::string right_cmd_str = "[";
                    // for(int i=0; i<7; ++i) {
                    //     char buf[32];
                    //     snprintf(buf, sizeof(buf), "%.4f%s", left_next_joints[i], (i<6?",":""));
                    //     left_cmd_str += buf;
                    //     snprintf(buf, sizeof(buf), "%.4f%s", right_next_joints[i], (i<6?",":""));
                    //     right_cmd_str += buf;
                    // }
                    // left_cmd_str += "]";
                    // right_cmd_str += "]";
                    std::stringstream left_ss, right_ss;
                    left_ss << "[";
                    right_ss << "[";
                    for(int i=0; i<7; ++i) {
                        left_ss << std::fixed << std::setprecision(8) << left_cmd[i];
                        right_ss << std::fixed << std::setprecision(8) << right_cmd[i];
                        if (i < 6) {
                            left_ss << ", ";
                            right_ss << ", ";
                        }
                    }
                    left_ss << "]";
                    right_ss << "]";
                    std::string left_cmd_str = left_ss.str();
                    std::string right_cmd_str = right_ss.str();
                    RCLCPP_INFO(get_logger(), "📤 CMD L:%s R:%s", left_cmd_str.c_str(), right_cmd_str.c_str());
                }

                // 统一发送，保证双臂同步
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgSend()...");
                uint32_t index = cmd_index_.load();
                RCLCPP_DEBUG(get_logger(), "[MainLoop] Current cmd_index=%u", index);
                if (!jaka_interface_.edgSend(&index)) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
                        "[MainLoop] ❌ Failed to send servo commands via edgSend");
                    
                    // 🔧 检查机械臂错误状态
                    int error[2] = {0, 0};
                    if (jaka_interface_.isInError(error)) {
                        if (error[0] || error[1]) {
                            ErrorCode error_code;
                            if (jaka_interface_.getLastError(error_code)) {
                                RCLCPP_ERROR(get_logger(), 
                                    "[MainLoop] 🚨 Robot error detected - Left:%d Right:%d | Code:%d Msg:%s", 
                                    error[0], error[1], error_code.code, error_code.message);
                            }
                            // // 自动尝试恢复
                            // RCLCPP_WARN(get_logger(), "[MainLoop] Attempting automatic recovery...");
                            // jaka_interface_.clearError();
                            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                    }
                    return;  // 发送失败，停止本周期
                }
                index++;
                cmd_index_.store(index);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgSend() returned, cmd_index=%u", index);
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, 
                    "[MainLoop] Servo commands sent successfully (L:%d R:%d)", has_left, has_right);
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        last_cycle_duration_us_ = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        
        // 性能监控：如果周期超时，发出警告
        double cycle_time_us = cycle_time_ms_ * 1000.0;
        if (last_cycle_duration_us_ > cycle_time_us * 0.8) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[MainLoop] Cycle time high: %.2f ms (target: %.2f ms)",
                last_cycle_duration_us_ / 1000.0, cycle_time_ms_);
        }
        
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
            "[MainLoop] Cycle completed in %.2f ms", last_cycle_duration_us_ / 1000.0);
    }

    void publishStatus()
    {
        auto now = this->now();
        
        // ==================== 1. 伺服状态 ====================
        auto servo_msg = qyh_jaka_control_msgs::msg::JakaServoStatus();
        servo_msg.mode = servo_running_ ? "bridge_joint" : "idle";
        servo_msg.is_abs = true;
        servo_msg.cycle_time_ns = static_cast<int32_t>(cycle_time_ms_ * 1e6);
        servo_msg.publish_rate_hz = servo_running_ ? (1000.0 / cycle_time_ms_) : 0.0;
        servo_msg.latency_ms = last_cycle_duration_us_ / 1000.0;
        status_pub_->publish(servo_msg);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Status] Servo: %s, Rate: %.1f Hz, Latency: %.2f ms",
            servo_msg.mode.c_str(), servo_msg.publish_rate_hz, servo_msg.latency_ms);

        // ==================== 2. 机器人状态 ====================
        auto robot_state_msg = qyh_jaka_control_msgs::msg::RobotState();
        robot_state_msg.header.stamp = now;
        robot_state_msg.header.frame_id = "world";
        robot_state_msg.connected = connected_;
        robot_state_msg.robot_ip = robot_ip_;
        robot_state_msg.servo_mode_enabled = servo_running_;
        
        // ⚡ 始终使用缓存数据（mainLoop现在总是更新缓存）
        if (has_cached_state_) {
            robot_state_msg.powered_on = powered_;
            robot_state_msg.enabled = enabled_;
            
            // 关节位置
            for (size_t i = 0; i < 7; ++i) {
                robot_state_msg.left_joint_positions[i] = cached_left_joints_.jVal[i];
                robot_state_msg.right_joint_positions[i] = cached_right_joints_.jVal[i];
            }
            
            // TCP位姿
            robot_state_msg.left_cartesian_pose.position.x = cached_left_pose_.tran.x;
            robot_state_msg.left_cartesian_pose.position.y = cached_left_pose_.tran.y;
            robot_state_msg.left_cartesian_pose.position.z = cached_left_pose_.tran.z;

            tf2::Quaternion q_left;
            q_left.setRPY(cached_left_pose_.rpy.rx, cached_left_pose_.rpy.ry, cached_left_pose_.rpy.rz);
            robot_state_msg.left_cartesian_pose.orientation = tf2::toMsg(q_left);

            robot_state_msg.right_cartesian_pose.position.x = cached_right_pose_.tran.x;
            robot_state_msg.right_cartesian_pose.position.y = cached_right_pose_.tran.y;
            robot_state_msg.right_cartesian_pose.position.z = cached_right_pose_.tran.z;

            tf2::Quaternion q_right;
            q_right.setRPY(cached_right_pose_.rpy.rx, cached_right_pose_.rpy.ry, cached_right_pose_.rpy.rz);
            robot_state_msg.right_cartesian_pose.orientation = tf2::toMsg(q_right);
            
            // 检查错误状态
            int error[2] = {0, 0};
            if (jaka_interface_.isInError(error)) {
                robot_state_msg.in_error = (error[0] || error[1]);
                if (robot_state_msg.in_error) {
                    ErrorCode error_code;
                    if (jaka_interface_.getLastError(error_code)) {
                        robot_state_msg.error_message = error_code.message;
                    }
                }
            }
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                "[Status] No cached state available yet");
        }

        robot_state_pub_->publish(robot_state_msg);

        // ==================== 3. 标准JointState消息 ====================
        // 注意：仅在真实模式下发布，可视化模式由mainLoop发布伺服指令
        if (!visualization_only_) {
            auto joint_state_msg = sensor_msgs::msg::JointState();
            joint_state_msg.header.stamp = now;
            joint_state_msg.header.frame_id = "world";
            
            // 关节名称
            joint_state_msg.name.reserve(14);
            for (int i = 1; i <= 7; ++i) {
                joint_state_msg.name.push_back("left_joint" + std::to_string(i));
            }
            for (int i = 1; i <= 7; ++i) {
                joint_state_msg.name.push_back("right_joint" + std::to_string(i));
            }
            
            // 关节位置（从robot_state_msg复用）
            joint_state_msg.position.reserve(14);
            joint_state_msg.position.assign(
                robot_state_msg.left_joint_positions.begin(),
                robot_state_msg.left_joint_positions.end());
            joint_state_msg.position.insert(
                joint_state_msg.position.end(),
                robot_state_msg.right_joint_positions.begin(),
                robot_state_msg.right_joint_positions.end());
            
            joint_states_pub_->publish(joint_state_msg);
        }
        
        // RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 10000,
        //     "[Status] Published all states (powered:%d, enabled:%d, servo:%d, joints:%zu)",
        //     robot_state_msg.powered_on, robot_state_msg.enabled, 
        //     robot_state_msg.servo_mode_enabled, joint_state_msg.position.size());
    }

    // ==================== VR目标位姿回调函数 ====================
    void leftVRTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!servo_running_ || !left_vel_controller_) return;
        
        // 🎯 频率控制：降至 ~15Hz (66ms)
        // VR输入(30Hz) + IK(30Hz) 没收益，只会增加抖动
        static rclcpp::Time left_last_ik_time(0, 0, RCL_ROS_TIME);
        if ((now() - left_last_ik_time).seconds() < (teleop_target_update_time_ms_ / 1000.0)) {
            return;
        }
        

        try {
            // ① TF变换到base_link
            geometry_msgs::msg::PoseStamped input_pose = *msg;
            input_pose.header.stamp = now();
            geometry_msgs::msg::PoseStamped target_in_base = tf_buffer_->transform(
                input_pose, "base_link_left", tf2::durationFromSec(0.1));
            
            if (has_z_offset_) {
                target_in_base.pose.position.z += left_z_offset_;
            }
            
            // ② 目标变化检测（过滤微小抖动）
            if (has_left_target_) {
                double pos_change = std::sqrt(
                    std::pow(target_in_base.pose.position.x - left_last_target_.pose.position.x, 2) +
                    std::pow(target_in_base.pose.position.y - left_last_target_.pose.position.y, 2) +
                    std::pow(target_in_base.pose.position.z - left_last_target_.pose.position.z, 2));
                
                // 🎯 策略：位置变化很小时，锁死姿态（防止手抖导致末端乱转）
                if (pos_change < 0.003) { // 3mm
                     target_in_base.pose.orientation = left_last_target_.pose.orientation;
                }

                // 🔍 调试日志：检测VR输入的大幅跳变
                if (pos_change > 0.05) { // 5cm
                    RCLCPP_WARN(get_logger(), "[Left] ⚠️ Large VR Input Jump: %.4f m", pos_change);
                }
                
                double ori_change = std::sqrt(
                    std::pow(target_in_base.pose.orientation.x - left_last_target_.pose.orientation.x, 2) +
                    std::pow(target_in_base.pose.orientation.y - left_last_target_.pose.orientation.y, 2) +
                    std::pow(target_in_base.pose.orientation.z - left_last_target_.pose.orientation.z, 2) +
                    std::pow(target_in_base.pose.orientation.w - left_last_target_.pose.orientation.w, 2));
                
                if (pos_change < target_change_pos_threshold_ && ori_change < target_change_ori_threshold_) {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, 
                        "[Left] Target change too small (pos:%.4f ori:%.4f), ignoring", pos_change, ori_change);
                    return;
                }
            }
            
            // ③ IK求解（30Hz，只在这里执行）
            std::vector<double> seed_joints(7);
            // 🎯 策略：使用 integrated_q_ 作为 seed，保证 IK 连续性
            if (!left_vel_controller_->getIntegratedQ(seed_joints)) {
                // 如果还没有 integrated_q_ (刚启动)，则使用真实反馈
                for (int i = 0; i < 7; ++i) {
                    seed_joints[i] = cached_left_joints_.jVal[i];
                }
            }
            
            std::vector<double> joint_target;
            bool ik_ok = left_vel_controller_->solveIK(target_in_base.pose, seed_joints, joint_target);
            
            // 🎯 策略：Branch-Safe Check
            if (!ik_ok) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "[Left] ❌ IK Failed for target pose");
                return;
            }

            if (!left_vel_controller_->checkIKContinuity(seed_joints, joint_target)) {
                // 详细日志已在 checkIKContinuity 内部打印
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "[Left] ❌ IK Continuity Check Failed - Motion Aborted");
                return;
            }
            // ✅ 只有成功 IK 才更新时间
            left_last_ik_time = now();
            
            // ④ 设置关节目标（Servo层会连续追踪）
            left_vel_controller_->setJointTargetRef(joint_target);
            
            left_last_target_ = target_in_base;
            has_left_target_ = true;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Left TF Error: %s", ex.what());
        }
    }
    
    void rightVRTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!servo_running_ || !right_vel_controller_) return;
        
        // 🎯 频率控制：降至 ~15Hz (66ms)
        static rclcpp::Time right_last_ik_time(0, 0, RCL_ROS_TIME);
        if ((now() - right_last_ik_time).seconds() < (teleop_target_update_time_ms_ / 1000.0)) {
            return;
        }

        try {
            // ① TF变换到base_link
            geometry_msgs::msg::PoseStamped input_pose = *msg;
            input_pose.header.stamp = now();
            geometry_msgs::msg::PoseStamped target_in_base = tf_buffer_->transform(
                input_pose, "base_link_right", tf2::durationFromSec(0.1));
            
            if (has_z_offset_) {
                target_in_base.pose.position.z += right_z_offset_;
            }
            
            // ② 目标变化检测（过滤微小抖动）
            if (has_right_target_) {
                double pos_change = std::sqrt(
                    std::pow(target_in_base.pose.position.x - right_last_target_.pose.position.x, 2) +
                    std::pow(target_in_base.pose.position.y - right_last_target_.pose.position.y, 2) +
                    std::pow(target_in_base.pose.position.z - right_last_target_.pose.position.z, 2));

                // 🔍 调试日志：检测VR输入的大幅跳变
                if (pos_change > 0.05) { // 5cm
                    RCLCPP_WARN(get_logger(), "[Right] ⚠️ Large VR Input Jump: %.4f m", pos_change);
                }
                    std::pow(target_in_base.pose.position.z - right_last_target_.pose.position.z, 2));
                
                // 🎯 策略：位置变化很小时，锁死姿态（防止手抖导致末端乱转）
                if (pos_change < 0.003) { // 3mm
                     target_in_base.pose.orientation = right_last_target_.pose.orientation;
                }
                
                double ori_change = std::sqrt(
                    std::pow(target_in_base.pose.orientation.x - right_last_target_.pose.orientation.x, 2) +
                    std::pow(target_in_base.pose.orientation.y - right_last_target_.pose.orientation.y, 2) +
                    std::pow(target_in_base.pose.orientation.z - right_last_target_.pose.orientation.z, 2) +
                    std::pow(target_in_base.pose.orientation.w - right_last_target_.pose.orientation.w, 2));
                
                if (pos_change < target_change_pos_threshold_ && ori_change < target_change_ori_threshold_) {
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, 
                        "[Right] Target change too small (pos:%.4f ori:%.4f), ignoring", pos_change, ori_change);
                    return;
                }
            }
            
            // ③ IK求解（30Hz，只在这里执行）
            std::vector<double> seed_joints(7);
            // 🎯 策略：使用 integrated_q_ 作为 seed，保证 IK 连续性
            if (!right_vel_controller_->getIntegratedQ(seed_joints)) {
                // 如果还没有 integrated_q_ (刚启动)，则使用真实反馈
                for (int i = 0; i < 7; ++i) {
                    seed_joints[i] = cached_right_joints_.jVal[i];
                }
            }
            
            std::vector<double> joint_target;
            bool ik_ok = right_vel_controller_->solveIK(target_in_base.pose, seed_joints, joint_target);
            
            // 🎯 策略：Branch-Safe Check
            if (!ik_ok) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "[Right] ❌ IK Failed for target pose");
                return;
            }

            if (!right_vel_controller_->checkIKContinuity(seed_joints, joint_target)) {
                // 详细日志已在 checkIKContinuity 内部打印
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 500, "[Right] ❌ IK Continuity Check Failed - Motion Aborted");
                return;
            }

            // ✅ 只有成功 IK 才更新时间
            right_last_ik_time = now();
            
            // ④ 设置关节目标（Servo层会连续追踪）
            right_vel_controller_->setJointTargetRef(joint_target);
            
            right_last_target_ = target_in_base;
            has_right_target_ = true;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Right TF Error: %s", ex.what());
        }
    }

    // ==================== 负载配置加载 ====================
    /**
     * @brief 从YAML配置文件加载并设置夹爪负载
     * @return true if successful
     */
    bool loadAndSetPayloadFromConfig() {
        // 获取配置文件路径：~/qyh_jushen_ws/persistent/preset/payload_config.yaml
        std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : std::getenv("USERPROFILE");
        if (home_dir.empty()) {
            // 尝试使用当前工作目录的相对路径
            home_dir = ".";
        }
        
        fs::path config_path = fs::path(home_dir) / "qyh_jushen_ws" / "persistent" / "preset" / "payload_config.yaml";
        
        // 如果找不到，尝试从当前可执行文件往上找
        if (!fs::exists(config_path)) {
            fs::path alt_path = fs::current_path().parent_path().parent_path().parent_path() / "persistent" / "preset" / "payload_config.yaml";
            if (fs::exists(alt_path)) {
                config_path = alt_path;
            }
        }
        
        if (!fs::exists(config_path)) {
            RCLCPP_WARN(get_logger(), "Payload config file not found at: %s", config_path.string().c_str());
            RCLCPP_WARN(get_logger(), "Skipping payload configuration. Using default robot settings.");
            return false;
        }
        
        try {
            RCLCPP_INFO(get_logger(), "Loading payload config from: %s", config_path.string().c_str());
            YAML::Node config = YAML::LoadFile(config_path.string());
            
            // 读取左右夹爪质量
            double left_mass = 0.0;
            double right_mass = 0.0;
            
            if (config["left_gripper"] && config["left_gripper"]["mass"]) {
                left_mass = config["left_gripper"]["mass"].as<double>();
                RCLCPP_INFO(get_logger(), "  Left gripper mass: %.2f kg", left_mass);
            } else {
                RCLCPP_WARN(get_logger(), "  Left gripper mass not found in config, using 0.0 kg");
            }
            
            if (config["right_gripper"] && config["right_gripper"]["mass"]) {
                right_mass = config["right_gripper"]["mass"].as<double>();
                RCLCPP_INFO(get_logger(), "  Right gripper mass: %.2f kg", right_mass);
            } else {
                RCLCPP_WARN(get_logger(), "  Right gripper mass not found in config, using 0.0 kg");
            }
            
            // 设置负载到机器人（centroid_x默认150mm，即夹爪质心在末端前方15cm）
            RCLCPP_INFO(get_logger(), "Setting payload to robot...");
            
            bool left_success = jaka_interface_.setPayload(0, left_mass, 150.0);
            if (left_success) {
                RCLCPP_INFO(get_logger(), "  ✓ Left arm payload set: %.2f kg", left_mass);
            } else {
                RCLCPP_ERROR(get_logger(), "  ✗ Failed to set left arm payload");
            }
            
            bool right_success = jaka_interface_.setPayload(1, right_mass, 150.0);
            if (right_success) {
                RCLCPP_INFO(get_logger(), "  ✓ Right arm payload set: %.2f kg", right_mass);
            } else {
                RCLCPP_ERROR(get_logger(), "  ✗ Failed to set right arm payload");
            }
            
            // 等待设置生效
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            return left_success && right_success;
            
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to parse payload config: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error loading payload config: %s", e.what());
            return false;
        }
    }
    
    // ==================== 关节限位检查 ====================
    bool checkJointLimits(const JointValue& joints, const std::string& arm_name) {
        for (int i = 0; i < 7; ++i) {
            // 归一化角度（防止SDK返回超出[-π,π]的值）
            double angle = normalizeAngle(joints.jVal[i]);
            
            if (angle < JAKA_ZU7_LIMITS[i].pos_min + SAFETY_MARGIN_POS ||
                angle > JAKA_ZU7_LIMITS[i].pos_max - SAFETY_MARGIN_POS) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[Safety] %s 关节%d超出限位: %.3f (归一化: %.3f, 限位: %.3f ~ %.3f)",
                    arm_name.c_str(), i+1, joints.jVal[i], angle,
                    JAKA_ZU7_LIMITS[i].pos_min, JAKA_ZU7_LIMITS[i].pos_max);
                return false;
            }
        }
        return true;
    }

    // ==================== 内部辅助函数（供 JakaServiceHandlers 调用）====================
    bool startServoInternal() {
        RCLCPP_INFO(get_logger(), "[Servo] === Starting Servo Mode ===");
        
        if (!enabled_) {
            RCLCPP_ERROR(get_logger(), "[Servo] Cannot start: robot not enabled");
            return false;
        }
        
        RCLCPP_INFO(get_logger(), "[Servo] Step 1/5: Setting up filter...");
        // 按照官方示例，默认使用 none filter（已在初始化时设置）
        RCLCPP_INFO(get_logger(), "[Servo] Using none filter (set during initialization)");
        
        RCLCPP_INFO(get_logger(), "[Servo] Step 2/5: Enabling servo for left arm (id=0)...");
        // 显式启用双臂伺服 (0:左臂, 1:右臂)
        bool success = true;
        success &= jaka_interface_.servoMoveEnable(true, 0);
        if (!success) {
            RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable left arm servo!");
            return false;
        }
        RCLCPP_INFO(get_logger(), "[Servo] Left arm servo enabled successfully");
        
        RCLCPP_INFO(get_logger(), "[Servo] Step 3/5: Enabling servo for right arm (id=1)...");
        success &= jaka_interface_.servoMoveEnable(true, 1);
        if (!success) {
            RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable right arm servo!");
            jaka_interface_.servoMoveEnable(false, 0);  // 回滚左臂
            return false;
        }
        RCLCPP_INFO(get_logger(), "[Servo] Right arm servo enabled successfully");
        
        if (success) {
            // ★★★ 参考官方示例：servo_move_enable后立即获取状态并初始化 ★★★
            // 不需要额外延迟，SDK内部已处理状态同步
            
            RCLCPP_INFO(get_logger(), "[Servo] Step 4/5: Initializing controllers from current positions...");
            
            // 从当前机械臂位置初始化Controller，避免启动跳变
            RCLCPP_INFO(get_logger(), "[Servo] Getting left arm current position...");
            JointValue left_pos, right_pos;
            bool left_init_success = false;
            bool right_init_success = false;
            
            if (left_vel_controller_ && jaka_interface_.getJointPositions(0, left_pos)) {
                std::vector<double> left_joints(7);
                for (size_t i = 0; i < 7; ++i) left_joints[i] = left_pos.jVal[i];
                RCLCPP_INFO(get_logger(), "[Servo] Left joints: [%f, %f, %f, %f, %f, %f, %f]",
                    left_joints[0], left_joints[1], left_joints[2], left_joints[3],
                    left_joints[4], left_joints[5], left_joints[6]);
                left_vel_controller_->updateRobotState(left_joints);
                left_vel_controller_->reset();
                left_init_success = true;
                RCLCPP_INFO(get_logger(), "[Servo] ✓ Left controller initialized from current position");
            } else {
                RCLCPP_WARN(get_logger(), "[Servo] ✗ Failed to initialize left controller");
            }
            
            RCLCPP_INFO(get_logger(), "[Servo] Getting right arm current position...");
            if (right_vel_controller_ && jaka_interface_.getJointPositions(1, right_pos)) {
                std::vector<double> right_joints(7);
                for (size_t i = 0; i < 7; ++i) right_joints[i] = right_pos.jVal[i];
                RCLCPP_INFO(get_logger(), "[Servo] Right joints: [%f, %f, %f, %f, %f, %f, %f]",
                    right_joints[0], right_joints[1], right_joints[2], right_joints[3],
                    right_joints[4], right_joints[5], right_joints[6]);
                right_vel_controller_->updateRobotState(right_joints);
                right_vel_controller_->reset();
                right_init_success = true;
                RCLCPP_INFO(get_logger(), "[Servo] ✓ Right controller initialized from current position");
            } else {
                RCLCPP_WARN(get_logger(), "[Servo] ✗ Failed to initialize right controller");
            }
            
            // ★★★ 初始化完成后立即启动主循环 ★★★
            // VelocityServoController 已在 updateRobotState() 时同步了真实位置
            // 并设置了 has_initialized_command_=true，主循环会自动发送静止指令
            if (left_init_success || right_init_success) {
                RCLCPP_INFO(get_logger(), "[Servo] Step 5/5: Controllers initialized and ready");
                
                // 初始化命令索引
                cmd_index_.store(0);
                
                // 立即启动伺服模式，主循环会自动发送静止指令
                servo_running_ = true;
                RCLCPP_INFO(get_logger(), "[Servo] === Servo Mode Active - Main loop will send hold commands ===");
            } else {
                RCLCPP_ERROR(get_logger(), "[Servo] Both controllers failed to initialize, rolling back...");
                // 回滚：关闭伺服并重置controller状态
                jaka_interface_.servoMoveEnable(false, 0);
                jaka_interface_.servoMoveEnable(false, 1);
                if (left_vel_controller_) left_vel_controller_->reset();
                if (right_vel_controller_) right_vel_controller_->reset();
                return false;
            }
            
            return true;
        }
        // 如果失败，尝试回滚
        RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable servo, rolling back...");
        jaka_interface_.servoMoveEnable(false, 0);
        jaka_interface_.servoMoveEnable(false, 1);
        RCLCPP_ERROR(get_logger(), "[Servo] Servo start failed!");
        return false;
    }
    
    bool stopServoInternal() {
        RCLCPP_INFO(get_logger(), "[Servo] === Stopping Servo Mode ===");
        servo_running_ = false;
        
        RCLCPP_INFO(get_logger(), "[Servo] Disabling left arm servo...");
        bool success = true;
        success &= jaka_interface_.servoMoveEnable(false, 0);
        RCLCPP_INFO(get_logger(), "[Servo] Disabling right arm servo...");
        success &= jaka_interface_.servoMoveEnable(false, 1);
        
        if (success) {
            RCLCPP_INFO(get_logger(), "[Servo] === Servo Mode Stopped ===");
        } else {
            RCLCPP_ERROR(get_logger(), "[Servo] Failed to stop servo cleanly");
        }
        return success;
    }
    
    JointValue convertToJointValue(const std::vector<double>& joints) {
        JointValue jv;
        for(size_t i=0; i<7 && i<joints.size(); ++i) jv.jVal[i] = joints[i];
        return jv;
    }

    // 成员变量
    qyh_jaka_control::JakaInterface jaka_interface_;
    std::unique_ptr<qyh_jaka_control::VelocityServoController> left_vel_controller_;
    std::unique_ptr<qyh_jaka_control::VelocityServoController> right_vel_controller_;
    std::unique_ptr<qyh_jaka_control::JakaServiceHandlers> service_handlers_;
    
    // 参数
    std::string robot_ip_;
    double cycle_time_ms_;
    bool visualization_only_;  // 仅可视化模式，不发送给真实机器人

    // 状态
    std::atomic<bool> connected_;
    std::atomic<bool> powered_;
    std::atomic<bool> enabled_;
    std::atomic<bool> servo_running_;
    int64_t last_cycle_duration_us_ = 0;
    std::atomic<uint32_t> cmd_index_{0};  // 线程安全的命令计数器
    
    // 缓存的机械臂位姿（主循环更新，状态发布使用）
    JointValue cached_left_joints_;
    JointValue cached_right_joints_;
    CartesianPose cached_left_pose_;
    CartesianPose cached_right_pose_;
    bool has_cached_state_{false};

    // IK求解相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool target_x_left_{false};
    bool has_z_offset_{true};
    double left_z_offset_{0.219885132};
    double right_z_offset_{0.217950931};
    
    // 目标变化死区（过滤VR手柄微小抖动）
    double target_change_pos_threshold_{0.002};   // 位置变化阈值（米）
    double target_change_ori_threshold_{0.035};   // 姿态变化阈值（四元数距离）
    double teleop_target_update_time_ms_{66.0};   // VR目标更新频率 (ms)
    geometry_msgs::msg::PoseStamped left_last_target_;
    geometry_msgs::msg::PoseStamped right_last_target_;
    bool has_left_target_{false};
    bool has_right_target_{false};
    
    // ROS接口
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    // VR遥操作订阅
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_vr_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_vr_target_sub_;

    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_on_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_off_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_error_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_motion_abort_;
    
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartServo>::SharedPtr srv_start_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopServo>::SharedPtr srv_stop_servo_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_bridge_start_servo_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_bridge_stop_servo_;

    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetPayload>::SharedPtr srv_set_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetPayload>::SharedPtr srv_get_payload_;

    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaControlNode>();
    // 在shared_ptr创建完成后初始化速度控制器
    node->initVelocityControllers();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
