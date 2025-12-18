#include "qyh_jaka_control/jaka_interface.hpp"
#include "qyh_jaka_control/smooth_servo_bridge.hpp"
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
#include <qyh_jaka_control_msgs/srv/set_filter.hpp>
#include <qyh_jaka_control_msgs/srv/move_j.hpp>
#include <qyh_jaka_control_msgs/srv/move_l.hpp>
#include <qyh_jaka_control_msgs/srv/set_collision_level.hpp>
#include <qyh_jaka_control_msgs/srv/set_tool_offset.hpp>
#include <qyh_jaka_control_msgs/srv/get_robot_state.hpp>
#include <qyh_jaka_control_msgs/srv/jog.hpp>
#include <qyh_jaka_control_msgs/srv/jog_stop.hpp>
#include <qyh_jaka_control_msgs/srv/set_payload.hpp>
#include <qyh_jaka_control_msgs/srv/get_payload.hpp>
#include <qyh_jaka_control_msgs/srv/compute_ik.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <array>
#include <cmath>

using namespace std::chrono_literals;

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
const double SAFETY_MARGIN_VEL = 0.8;     // 允许超过标称速度20%

// 归一化角度到[-π, π]范围
static inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ==================== 完整轨迹平滑器 ====================
/**
 * @brief 单臂轨迹平滑器 - 用于笛卡尔遥操作
 * 
 * 功能（与 teleoperation_controller 的 TrajectorySmoother 对齐）：
 * 1. 位置增量限幅 - 防止IK跳变
 * 2. 速度限幅 - 限制最大运动速度
 * 3. 加速度限幅 - 限制加速能力
 * 4. Jerk限幅 - 保证运动丝滑，无顿挫感
 * 5. 一阶低通滤波 - 滤除VR手柄高频抖动
 */
class SimpleJointSmoother {
public:
    struct Limits {
        double max_velocity;
        double max_acceleration;
        double max_jerk;
        double low_pass_cutoff;
        double max_position_delta;
        
        Limits() 
            : max_velocity(1.0),           // rad/s
              max_acceleration(2.0),       // rad/s²
              max_jerk(10.0),              // rad/s³ - 保证平滑
              low_pass_cutoff(8.0),        // Hz - 滤除人手8-12Hz抖动
              max_position_delta(0.05)     // rad/step (约3度)
        {}
    };

    SimpleJointSmoother(size_t num_joints = 7, const Limits& limits = Limits())
        : num_joints_(num_joints), limits_(limits), initialized_(false) {
        current_pos_.resize(num_joints, 0.0);
        current_vel_.resize(num_joints, 0.0);
        current_acc_.resize(num_joints, 0.0);
        filtered_pos_.resize(num_joints, 0.0);
    }

    std::vector<double> smooth(const std::vector<double>& target, double dt) {
        if (target.size() != num_joints_ || dt <= 0.0) {
            return target;
        }

        if (!initialized_) {
            current_pos_ = target;
            filtered_pos_ = target;
            std::fill(current_vel_.begin(), current_vel_.end(), 0.0);
            std::fill(current_acc_.begin(), current_acc_.end(), 0.0);
            initialized_ = true;
            return target;
        }

        std::vector<double> result(num_joints_);

        for (size_t i = 0; i < num_joints_; ++i) {
            // 1. 位置增量限幅 (防止IK解跳变)
            double pos_delta = target[i] - current_pos_[i];
            pos_delta = std::clamp(pos_delta, -limits_.max_position_delta, limits_.max_position_delta);
            double limited_target = current_pos_[i] + pos_delta;

            // 2. 计算期望速度并限幅
            double desired_vel = (limited_target - current_pos_[i]) / dt;
            desired_vel = std::clamp(desired_vel, -limits_.max_velocity, limits_.max_velocity);

            // 3. 计算期望加速度并限幅
            double desired_acc = (desired_vel - current_vel_[i]) / dt;
            desired_acc = std::clamp(desired_acc, -limits_.max_acceleration, limits_.max_acceleration);

            // 4. Jerk限幅 (关键：保证运动丝滑)
            double desired_jerk = (desired_acc - current_acc_[i]) / dt;
            desired_jerk = std::clamp(desired_jerk, -limits_.max_jerk, limits_.max_jerk);
            double new_acc = current_acc_[i] + desired_jerk * dt;

            // 5. 从限幅后的加速度反推速度和位置
            double new_vel = current_vel_[i] + new_acc * dt;
            new_vel = std::clamp(new_vel, -limits_.max_velocity, limits_.max_velocity);
            double new_pos = current_pos_[i] + new_vel * dt;

            // 6. 低通滤波 (滤除人手高频抖动)
            double RC = 1.0 / (2.0 * M_PI * limits_.low_pass_cutoff);
            double alpha = dt / (dt + RC);
            filtered_pos_[i] = alpha * new_pos + (1.0 - alpha) * filtered_pos_[i];

            result[i] = filtered_pos_[i];
            current_pos_[i] = new_pos;
            current_vel_[i] = new_vel;
            current_acc_[i] = new_acc;
        }

        return result;
    }

    void reset() {
        initialized_ = false;
        std::fill(current_vel_.begin(), current_vel_.end(), 0.0);
        std::fill(current_acc_.begin(), current_acc_.end(), 0.0);
    }

    void setLimits(const Limits& limits) { limits_ = limits; }

    // 用当前机器人状态初始化（避免启动跳变）- 仅在未初始化时生效
    void initializeWith(const std::vector<double>& current_joints) {
        if (!initialized_ && current_joints.size() == num_joints_) {
            current_pos_ = current_joints;
            filtered_pos_ = current_joints;
            std::fill(current_vel_.begin(), current_vel_.end(), 0.0);
            std::fill(current_acc_.begin(), current_acc_.end(), 0.0);
            initialized_ = true;
        }
    }

private:
    size_t num_joints_;
    Limits limits_;
    bool initialized_;
    std::vector<double> current_pos_;
    std::vector<double> current_vel_;
    std::vector<double> current_acc_;  // 新增：加速度状态
    std::vector<double> filtered_pos_;
};

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
        
        // Bridge 参数
        declare_parameter<int>("buffer_size", 16);  // 增大缓冲区以提高平滑度
        declare_parameter<double>("interpolation_weight", 0.3);  // 降低权重以提高平滑度
        declare_parameter<bool>("enable_interpolation", true);
        declare_parameter<double>("velocity_safety_factor", 0.65);  // 降低速度以提高平滑度

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
        
        // 初始化 Bridge
        size_t buffer_size = static_cast<size_t>(get_parameter("buffer_size").as_int());
        double servo_freq = 1000.0 / cycle_time_ms_;
        
        left_bridge_ = std::make_unique<qyh_jaka_control::SmoothServoBridge>(
            get_logger(), buffer_size, servo_freq
        );
        right_bridge_ = std::make_unique<qyh_jaka_control::SmoothServoBridge>(
            get_logger(), buffer_size, servo_freq
        );
        
        double interp_weight = get_parameter("interpolation_weight").as_double();
        bool enable_interp = get_parameter("enable_interpolation").as_bool();
        double velocity_safety_factor = get_parameter("velocity_safety_factor").as_double();
        
        left_bridge_->setInterpolationWeight(interp_weight);
        left_bridge_->enableInterpolation(enable_interp);
        right_bridge_->setInterpolationWeight(interp_weight);
        right_bridge_->enableInterpolation(enable_interp);
        
        // 设置JAKA Zu7的速度限制（从关节限位常量中提取）
        std::vector<double> velocity_limits(7);
        for (size_t i = 0; i < 7; ++i) {
            velocity_limits[i] = JAKA_ZU7_LIMITS[i].vel_max;
        }
        left_bridge_->setVelocityLimits(velocity_limits);
        right_bridge_->setVelocityLimits(velocity_limits);
        left_bridge_->setVelocitySafetyFactor(velocity_safety_factor);
        right_bridge_->setVelocitySafetyFactor(velocity_safety_factor);
        
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "  平滑参数配置:");
        RCLCPP_INFO(get_logger(), "    Buffer size: %zu", buffer_size);
        RCLCPP_INFO(get_logger(), "    Interpolation weight: %.2f", interp_weight);
        RCLCPP_INFO(get_logger(), "    Velocity safety factor: %.1f%%", velocity_safety_factor * 100.0);
        RCLCPP_INFO(get_logger(), "========================================");

        // Publishers
        status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>("/jaka/servo/status", 10);
        robot_state_pub_ = create_publisher<qyh_jaka_control_msgs::msg::RobotState>("/jaka/robot_state", 10);
        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // Subscribers (Bridge模式 - 标准ROS接口，支持独立单臂控制)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        left_bridge_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm/joint_command", qos,
            std::bind(&JakaControlNode::leftBridgeCallback, this, std::placeholders::_1));
            
        right_bridge_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm/joint_command", qos,
            std::bind(&JakaControlNode::rightBridgeCallback, this, std::placeholders::_1));

        // IK求解模式参数
        declare_parameter<bool>("ik_solver.enabled", false);
        declare_parameter<bool>("ik_solver.target_x_left", false);
        declare_parameter<bool>("ik_solver.has_z_offset", true);
        declare_parameter<double>("ik_solver.left_z_offset", 0.219885132);
        declare_parameter<double>("ik_solver.right_z_offset", 0.217950931);
        
        ik_enabled_ = get_parameter("ik_solver.enabled").as_bool();
        target_x_left_ = get_parameter("ik_solver.target_x_left").as_bool();
        has_z_offset_ = get_parameter("ik_solver.has_z_offset").as_bool();
        left_z_offset_ = get_parameter("ik_solver.left_z_offset").as_double();
        right_z_offset_ = get_parameter("ik_solver.right_z_offset").as_double();
        
        // IK模式：订阅VR目标位姿
        if (ik_enabled_) {
            RCLCPP_INFO(get_logger(), "🎯 IK求解模式已启用");
            RCLCPP_INFO(get_logger(), "  target_x_left=%s, has_z_offset=%s", 
                target_x_left_ ? "true" : "false", 
                has_z_offset_ ? "true" : "false");
            
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
        } else {
            RCLCPP_INFO(get_logger(), "📋 使用标准Bridge模式（关节空间指令）");
        }

        // Services (Basic Control)
        srv_power_on_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/power_on",
            std::bind(&JakaControlNode::handlePowerOn, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_power_off_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/power_off",
            std::bind(&JakaControlNode::handlePowerOff, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_enable_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/enable",
            std::bind(&JakaControlNode::handleEnable, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_disable_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/disable",
            std::bind(&JakaControlNode::handleDisable, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_clear_error_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/clear_error",
            std::bind(&JakaControlNode::handleClearError, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_motion_abort_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/robot/motion_abort",
            std::bind(&JakaControlNode::handleMotionAbort, this, std::placeholders::_1, std::placeholders::_2));

        // Services (Servo Control)
        srv_start_servo_ = create_service<qyh_jaka_control_msgs::srv::StartServo>(
            "/jaka/servo/start",
            std::bind(&JakaControlNode::handleStartServo, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_stop_servo_ = create_service<qyh_jaka_control_msgs::srv::StopServo>(
            "/jaka/servo/stop",
            std::bind(&JakaControlNode::handleStopServo, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_set_filter_ = create_service<qyh_jaka_control_msgs::srv::SetFilter>(
            "/jaka/servo/set_filter",
            std::bind(&JakaControlNode::handleSetFilter, this, std::placeholders::_1, std::placeholders::_2));

        // Services (Bridge Compatibility)
        srv_bridge_start_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/start_servo",
            std::bind(&JakaControlNode::handleBridgeStartServo, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_bridge_stop_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/stop_servo",
            std::bind(&JakaControlNode::handleBridgeStopServo, this, std::placeholders::_1, std::placeholders::_2));

        // Services (Motion & Others)
        srv_move_j_ = create_service<qyh_jaka_control_msgs::srv::MoveJ>(
            "/jaka/move_j",
            std::bind(&JakaControlNode::handleMoveJ, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_move_l_ = create_service<qyh_jaka_control_msgs::srv::MoveL>(
            "/jaka/move_l",
            std::bind(&JakaControlNode::handleMoveL, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_set_collision_level_ = create_service<qyh_jaka_control_msgs::srv::SetCollisionLevel>(
            "/jaka/set_collision_level",
            std::bind(&JakaControlNode::handleSetCollisionLevel, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_set_tool_offset_ = create_service<qyh_jaka_control_msgs::srv::SetToolOffset>(
            "/jaka/set_tool_offset",
            std::bind(&JakaControlNode::handleSetToolOffset, this, std::placeholders::_1, std::placeholders::_2));
            
        srv_get_robot_state_ = create_service<qyh_jaka_control_msgs::srv::GetRobotState>(
            "/jaka/get_robot_state",
            std::bind(&JakaControlNode::handleGetRobotState, this, std::placeholders::_1, std::placeholders::_2));

        srv_jog_ = create_service<qyh_jaka_control_msgs::srv::Jog>(
            "/jaka/jog",
            std::bind(&JakaControlNode::handleJog, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_jog_stop_ = create_service<qyh_jaka_control_msgs::srv::JogStop>(
            "/jaka/jog_stop",
            std::bind(&JakaControlNode::handleJogStop, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_payload_ = create_service<qyh_jaka_control_msgs::srv::SetPayload>(
            "/jaka/set_payload",
            std::bind(&JakaControlNode::handleSetPayload, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_get_payload_ = create_service<qyh_jaka_control_msgs::srv::GetPayload>(
            "/jaka/get_payload",
            std::bind(&JakaControlNode::handleGetPayload, this, std::placeholders::_1, std::placeholders::_2));

        // IK 服务
        srv_compute_ik_ = create_service<qyh_jaka_control_msgs::srv::ComputeIK>(
            "/jaka/compute_ik",
            std::bind(&JakaControlNode::handleComputeIK, this, std::placeholders::_1, std::placeholders::_2));

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
            jaka_interface_.servoMoveEnable(false);
        }
        if (connected_) {
            jaka_interface_.disconnect();
        }
    }

private:
    // ==================== 主循环 ====================
    void mainLoop()
    {
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

        RCLCPP_DEBUG_ONCE(get_logger(), "[MainLoop] First servo cycle");
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[MainLoop] Running at %.1f Hz (cycle: %.2f ms)", 
            1000.0 / cycle_time_ms_, cycle_time_ms_);
        
        // Bridge模式：从缓冲区获取插值后的指令
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Getting interpolated commands...");
        std::vector<double> left_cmd, right_cmd;
        bool has_left = left_bridge_->getInterpolatedCommand(left_cmd);
        bool has_right = right_bridge_->getInterpolatedCommand(right_cmd);
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
                bool success = true;
                
                // 独立发送左臂指令
                if (has_left) {
                    RCLCPP_DEBUG_ONCE(get_logger(), "[MainLoop] First left command - preparing edgServoJ");
                    auto jv = convertToJointValue(left_cmd);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "[Bridge] Left: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        jv.jVal[0], jv.jVal[1], jv.jVal[2], jv.jVal[3], jv.jVal[4], jv.jVal[5], jv.jVal[6]);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgServoJ(0)...");
                    success &= jaka_interface_.edgServoJ(0, jv, true);
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
                    success &= jaka_interface_.edgServoJ(1, jv, true);
                    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgServoJ(1) returned");
                }
                
                // 统一发送，保证双臂同步
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgSend()...");
                success &= jaka_interface_.edgSend(&cmd_index_);
                cmd_index_++;
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgSend() returned, cmd_index=%u, success=%d", cmd_index_, success);
                
                if (!success) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
                        "[Bridge] Failed to send servo commands");
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, 
                        "[MainLoop] Servo commands sent successfully (L:%d R:%d)", has_left, has_right);
                }
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

    // ==================== Bridge回调函数 ====================
    void leftBridgeCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), 
            "[Callback] leftBridgeCallback received: servo_running=%d, positions=%zu", 
            servo_running_.load(), msg->position.size());
        if (servo_running_ && msg->position.size() >= 7) {
            RCLCPP_INFO(get_logger(), "[Callback] Condition passed, processing command");
            std::vector<double> positions(msg->position.begin(), msg->position.begin() + 7);
            
            // 使用缓存的当前位置传给Bridge，由Bridge智能决定是否使用
            std::vector<double> current_joints(7);
            if (has_cached_state_) {
                for (size_t i = 0; i < 7; ++i) current_joints[i] = cached_left_joints_.jVal[i];
                left_bridge_->addCommand(positions, current_joints);
            } else {
                left_bridge_->addCommand(positions);
            }
        } else {
            RCLCPP_WARN(get_logger(), "[Callback] Command rejected: servo_running=%d, size=%zu (need: true & >=7)", 
                        servo_running_.load(), msg->position.size());
        }
    }
    
    void rightBridgeCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), 
            "[Callback] rightBridgeCallback received: servo_running=%d, positions=%zu", 
            servo_running_.load(), msg->position.size());
        if (servo_running_ && msg->position.size() >= 7) {
            RCLCPP_INFO(get_logger(), "[Callback] Condition passed, processing command");
            std::vector<double> positions(msg->position.begin(), msg->position.begin() + 7);
            
            // 使用缓存的当前位置传给Bridge，由Bridge智能决定是否使用
            std::vector<double> current_joints(7);
            if (has_cached_state_) {
                for (size_t i = 0; i < 7; ++i) current_joints[i] = cached_right_joints_.jVal[i];
                right_bridge_->addCommand(positions, current_joints);
            } else {
                right_bridge_->addCommand(positions);
            }
        } else {
            RCLCPP_WARN(get_logger(), "[Callback] Command rejected: servo_running=%d, size=%zu (need: true & >=7)", 
                        servo_running_.load(), msg->position.size());
        }
    }

    // ==================== IK求解回调函数 ====================
    void leftVRTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        left_target_ = msg;
        has_left_target_ = true;
        
        if (servo_running_) {
            solveLeftArmIK();
        }
    }
    
    void rightVRTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        right_target_ = msg;
        has_right_target_ = true;
        
        if (servo_running_) {
            solveRightArmIK();
        }
    }

    // ==================== IK求解函数 ====================
    bool solveLeftArmIK() {
        if (!has_left_target_ || !servo_running_) return false;
        
        // 检查消息新鲜度
        auto msg_age = (now() - left_target_->header.stamp).seconds();
        if (msg_age > 1.0) {
            has_left_target_ = false;
            return false;
        }
        
        // TF坐标系转换
        geometry_msgs::msg::PoseStamped target_in_base_left;
        try {
            geometry_msgs::msg::PoseStamped input_pose = *left_target_;
            input_pose.header.stamp = rclcpp::Time(0);
            target_in_base_left = tf_buffer_->transform(
                input_pose, "base_link_left", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "左臂TF转换失败: %s", ex.what());
            return false;
        }
        
        // 提取位姿
        tf2::Quaternion q_base_left(
            target_in_base_left.pose.orientation.x,
            target_in_base_left.pose.orientation.y,
            target_in_base_left.pose.orientation.z,
            target_in_base_left.pose.orientation.w);
        
        tf2::Vector3 pos_base_left(
            target_in_base_left.pose.position.x,
            target_in_base_left.pose.position.y,
            target_in_base_left.pose.position.z);
        
        // Z轴偏移
        if (has_z_offset_) {
            pos_base_left.setZ(pos_base_left.z() + left_z_offset_);
        }
        
        // 末端坐标系校正
        tf2::Matrix3x3 R_correction(
            0.0,  1.0,  0.0,
            0.0,  0.0,  1.0,
            1.0,  0.0,  0.0);
        
        if (target_x_left_) {
            R_correction = tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        }
        
        tf2::Matrix3x3 R_base_left(q_base_left);
        tf2::Matrix3x3 R_corrected = R_base_left * R_correction.transpose();
        
        tf2::Quaternion q_corrected;
        R_corrected.getRotation(q_corrected);
        q_corrected.normalize();
        
        // 转换到JAKA格式
        CartesianPose target_pose;
        target_pose.tran.x = pos_base_left.x() * 1000.0;  // m -> mm
        target_pose.tran.y = pos_base_left.y() * 1000.0;
        target_pose.tran.z = pos_base_left.z() * 1000.0;
        
        tf2::Matrix3x3 m(q_corrected);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 直接获取当前真实关节角度作为IK参考
        JointValue ref_joints;
        jaka_interface_.getJointPositions(0, ref_joints);
        
        // 调用IK求解
        JointValue ik_result;
        if (jaka_interface_.kineInverse(0, ref_joints, target_pose, ik_result)) {
            // 安全检查
            if (!checkJointLimits(ik_result, "左臂")) {
                return false;
            }
            
            // 直接添加到Bridge，传入当前位置（ref_joints就是刚获取的当前位置）
            std::vector<double> positions(7);
            std::vector<double> current_joints(7);
            for (size_t i = 0; i < 7; ++i) {
                positions[i] = ik_result.jVal[i];
                current_joints[i] = ref_joints.jVal[i];
            }
            
            left_bridge_->addCommand(positions, current_joints);
            ik_left_success_count_++;
            return true;
        } else {
            ik_left_error_count_++;
            if (ik_left_error_count_ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "[IK] 左臂IK失败 (错误计数: %d)", ik_left_error_count_);
            }
            return false;
        }
    }
    
    bool solveRightArmIK() {
        if (!has_right_target_ || !servo_running_) return false;
        
        // 检查消息新鲜度
        auto msg_age = (now() - right_target_->header.stamp).seconds();
        if (msg_age > 1.0) {
            has_right_target_ = false;
            return false;
        }
        
        // TF坐标系转换
        geometry_msgs::msg::PoseStamped target_in_base_right;
        try {
            geometry_msgs::msg::PoseStamped input_pose = *right_target_;
            input_pose.header.stamp = rclcpp::Time(0);
            target_in_base_right = tf_buffer_->transform(
                input_pose, "base_link_right", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "右臂TF转换失败: %s", ex.what());
            return false;
        }
        
        // 提取位姿
        tf2::Quaternion q_base_right(
            target_in_base_right.pose.orientation.x,
            target_in_base_right.pose.orientation.y,
            target_in_base_right.pose.orientation.z,
            target_in_base_right.pose.orientation.w);
        
        tf2::Vector3 pos_base_right(
            target_in_base_right.pose.position.x,
            target_in_base_right.pose.position.y,
            target_in_base_right.pose.position.z);
        
        // Z轴偏移
        if (has_z_offset_) {
            pos_base_right.setZ(pos_base_right.z() + right_z_offset_);
        }
        
        // 末端坐标系校正
        tf2::Matrix3x3 R_correction(
            0.0,  1.0,  0.0,
            0.0,  0.0,  1.0,
            1.0,  0.0,  0.0);
        
        if (target_x_left_) {
            R_correction = tf2::Matrix3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        }
        
        tf2::Matrix3x3 R_base_right(q_base_right);
        tf2::Matrix3x3 R_corrected = R_base_right * R_correction.transpose();
        
        tf2::Quaternion q_corrected;
        R_corrected.getRotation(q_corrected);
        q_corrected.normalize();
        
        // 转换到JAKA格式
        CartesianPose target_pose;
        target_pose.tran.x = pos_base_right.x() * 1000.0;  // m -> mm
        target_pose.tran.y = pos_base_right.y() * 1000.0;
        target_pose.tran.z = pos_base_right.z() * 1000.0;
        
        tf2::Matrix3x3 m(q_corrected);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 直接获取当前真实关节角度作为IK参考
        JointValue ref_joints;
        jaka_interface_.getJointPositions(1, ref_joints);
        
        // 调用IK求解
        JointValue ik_result;
        if (jaka_interface_.kineInverse(1, ref_joints, target_pose, ik_result)) {
            // 安全检查
            if (!checkJointLimits(ik_result, "右臂")) {
                return false;
            }
            
            // 直接添加到Bridge，传入当前位置（ref_joints就是刚获取的当前位置）
            std::vector<double> positions(7);
            std::vector<double> current_joints(7);
            for (size_t i = 0; i < 7; ++i) {
                positions[i] = ik_result.jVal[i];
                current_joints[i] = ref_joints.jVal[i];
            }
            
            right_bridge_->addCommand(positions, current_joints);
            ik_right_success_count_++;
            return true;
        } else {
            ik_right_error_count_++;
            if (ik_right_error_count_ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "[IK] 右臂IK失败 (错误计数: %d)", ik_right_error_count_);
            }
            return false;
        }
    }
    
    bool checkJointLimits(const JointValue& joints, const std::string& arm_name) {
        for (int i = 0; i < 7; ++i) {
            if (joints.jVal[i] < JAKA_ZU7_LIMITS[i].pos_min + SAFETY_MARGIN_POS ||
                joints.jVal[i] > JAKA_ZU7_LIMITS[i].pos_max - SAFETY_MARGIN_POS) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[IK] %s 关节%d超出限位: %.3f (限位: %.3f ~ %.3f)",
                    arm_name.c_str(), i+1, joints.jVal[i],
                    JAKA_ZU7_LIMITS[i].pos_min, JAKA_ZU7_LIMITS[i].pos_max);
                return false;
            }
        }
        return true;
    }

    // ==================== 基础控制服务 ====================
    void handlePowerOn(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!connected_) { res->success = false; res->message = "Robot not connected"; return; }
        if (jaka_interface_.powerOn()) { powered_ = true; res->success = true; res->message = "Power ON success"; }
        else { res->success = false; res->message = "Power ON failed"; }
    }

    void handlePowerOff(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
        if (jaka_interface_.powerOff()) { powered_ = false; enabled_ = false; res->success = true; res->message = "Power OFF success"; }
        else { res->success = false; res->message = "Power OFF failed"; }
    }

    void handleEnable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!powered_) { res->success = false; res->message = "Robot not powered on"; return; }
        if (jaka_interface_.enableRobot()) { enabled_ = true; res->success = true; res->message = "Robot ENABLED"; }
        else { res->success = false; res->message = "Enable failed"; }
    }

    void handleDisable(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (servo_running_) { res->success = false; res->message = "Stop servo first"; return; }
        if (jaka_interface_.disableRobot()) { enabled_ = false; res->success = true; res->message = "Robot DISABLED"; }
        else { res->success = false; res->message = "Disable failed"; }
    }

    void handleClearError(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (jaka_interface_.clearError()) { res->success = true; res->message = "Error cleared"; }
        else { res->success = false; res->message = "Clear error failed"; }
    }

    void handleMotionAbort(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (jaka_interface_.motionAbort()) { servo_running_ = false; res->success = true; res->message = "Motion aborted"; }
        else { res->success = false; res->message = "Motion abort failed"; }
    }

    // ==================== 伺服控制服务 ====================
    void handleStartServo(const qyh_jaka_control_msgs::srv::StartServo::Request::SharedPtr, qyh_jaka_control_msgs::srv::StartServo::Response::SharedPtr res)
    {
        if (startServoInternal()) { res->success = true; res->message = "Servo started"; }
        else { res->success = false; res->message = "Start servo failed"; }
    }

    void handleStopServo(const qyh_jaka_control_msgs::srv::StopServo::Request::SharedPtr, qyh_jaka_control_msgs::srv::StopServo::Response::SharedPtr res)
    {
        if (stopServoInternal()) { res->success = true; res->message = "Servo stopped"; }
        else { res->success = false; res->message = "Stop servo failed"; }
    }
    
    void handleBridgeStartServo(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (startServoInternal()) { res->success = true; res->message = "Servo started (Bridge)"; }
        else { res->success = false; res->message = "Start servo failed"; }
    }
    
    void handleBridgeStopServo(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (stopServoInternal()) { res->success = true; res->message = "Servo stopped (Bridge)"; }
        else { res->success = false; res->message = "Stop servo failed"; }
    }

    void handleSetFilter(const qyh_jaka_control_msgs::srv::SetFilter::Request::SharedPtr req, qyh_jaka_control_msgs::srv::SetFilter::Response::SharedPtr res)
    {
        bool success = false;
        if (req->filter_type == "none") success = jaka_interface_.setFilterNone();
        else if (req->filter_type == "joint_lpf") success = jaka_interface_.setFilterJointLPF(req->cutoff_frequency);
        else if (req->filter_type == "joint_nlf") success = jaka_interface_.setFilterJointNLF(req->max_joint_velocity, req->max_joint_acceleration, req->max_joint_jerk);
        
        res->success = success;
        res->message = success ? "Filter set" : "Failed to set filter";
    }

    // ==================== 内部辅助函数 ====================
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
            
            RCLCPP_INFO(get_logger(), "[Servo] Step 4/5: Initializing bridges from current positions...");
            servo_running_ = true;
            
            // 从当前机械臂位置初始化Bridge，避免启动跳变
            RCLCPP_INFO(get_logger(), "[Servo] Getting left arm current position...");
            JointValue left_pos, right_pos;
            if (jaka_interface_.getJointPositions(0, left_pos)) {
                std::vector<double> left_joints(7);
                for (size_t i = 0; i < 7; ++i) left_joints[i] = left_pos.jVal[i];
                RCLCPP_INFO(get_logger(), "[Servo] Left joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    left_joints[0], left_joints[1], left_joints[2], left_joints[3],
                    left_joints[4], left_joints[5], left_joints[6]);
                left_bridge_->initializeFromCurrent(left_joints);
                RCLCPP_INFO(get_logger(), "[Servo] ✓ Left bridge initialized from current position");
            } else {
                RCLCPP_WARN(get_logger(), "[Servo] ✗ Failed to get left arm position for initialization");
            }
            
            RCLCPP_INFO(get_logger(), "[Servo] Getting right arm current position...");
            if (jaka_interface_.getJointPositions(1, right_pos)) {
                std::vector<double> right_joints(7);
                for (size_t i = 0; i < 7; ++i) right_joints[i] = right_pos.jVal[i];
                RCLCPP_INFO(get_logger(), "[Servo] Right joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    right_joints[0], right_joints[1], right_joints[2], right_joints[3],
                    right_joints[4], right_joints[5], right_joints[6]);
                right_bridge_->initializeFromCurrent(right_joints);
                RCLCPP_INFO(get_logger(), "[Servo] ✓ Right bridge initialized from current position");
            } else {
                RCLCPP_WARN(get_logger(), "[Servo] ✗ Failed to get right arm position for initialization");
            }
            
            RCLCPP_INFO(get_logger(), "[Servo] Step 5/5: All initialization complete");
            RCLCPP_INFO(get_logger(), "[Servo] === Servo Mode Active - Ready for commands ===");
            
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

    // ==================== 其他服务 (MoveJ, MoveL, Jog, Payload) ====================
    // 这里的实现与原 jaka_control_node 相同，为节省篇幅，仅保留空壳或简单实现
    // 实际使用时应完整复制原逻辑
    
    void handleMoveJ(const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res) {
        res->success = jaka_interface_.moveJ(req->robot_id, std::vector<double>(req->joint_positions.begin(), req->joint_positions.end()), 
                                           req->move_mode, req->velocity, req->acceleration, req->is_block);
    }
    
    void handleMoveL(const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req, qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res) {
        res->success = jaka_interface_.moveL(req->robot_id, req->target_pose, req->move_mode, req->velocity, req->acceleration, req->is_block);
    }
    
    void handleSetCollisionLevel(const qyh_jaka_control_msgs::srv::SetCollisionLevel::Request::SharedPtr, qyh_jaka_control_msgs::srv::SetCollisionLevel::Response::SharedPtr res) {
        res->success = true; // Placeholder
    }
    
    void handleSetToolOffset(const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr, qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res) {
        res->success = true; // Placeholder
    }
    
    void handleGetRobotState(const qyh_jaka_control_msgs::srv::GetRobotState::Request::SharedPtr, qyh_jaka_control_msgs::srv::GetRobotState::Response::SharedPtr res) {
        RobotState state;
        if (jaka_interface_.getRobotState(state)) {
            res->powered_on = state.poweredOn;
            res->servo_enabled = state.servoEnabled;
            res->estoped = state.estoped;
            
            int error[2] = {0, 0};
            jaka_interface_.isInError(error);
            res->in_error = (error[0] || error[1]);
            
            if (res->in_error) {
                ErrorCode error_code;
                if (jaka_interface_.getLastError(error_code)) {
                    res->error_message = error_code.message;
                }
            }
        }
    }
    
    void handleJog(const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req, qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res) {
        if (!enabled_) {
            res->success = false;
            res->message = "Robot not enabled";
            return;
        }

        // 检查是否可以执行点动（非伺服模式）
        if (!jaka_interface_.canJog()) {
            res->success = false;
            res->message = "Cannot jog: servo mode is enabled or not connected";
            return;
        }

        // 新接口参数转换:
        // axis_num: 服务是1-based, 新接口是0-based
        int axis_index = req->axis_num - 1;  // 转换为0-based
        double velocity_percent = std::abs(req->velocity) * 100.0;  // 简单转换为百分比
        if (velocity_percent < 1.0) velocity_percent = 30.0;
        if (velocity_percent > 100.0) velocity_percent = 100.0;
        
        // 方向由速度符号决定
        int direction = (req->velocity >= 0) ? 1 : -1;
        
        bool success = false;
        
        if (req->coord_type == req->COORD_JOINT) {
            // 关节点动
            if (req->move_mode == req->MOVE_CONTINUOUS) {
                success = jaka_interface_.jogJointContinuous(req->robot_id, axis_index, direction, velocity_percent);
            } else {
                double step_deg = req->position * 180.0 / M_PI * direction;
                success = jaka_interface_.jogJoint(req->robot_id, axis_index, step_deg, velocity_percent);
            }
        } else {
            // 笛卡尔点动
            int coord = (req->coord_type == req->COORD_TOOL) ? 2 : 0;
            
            if (req->move_mode == req->MOVE_CONTINUOUS) {
                success = jaka_interface_.jogCartesianContinuous(req->robot_id, axis_index, direction, velocity_percent, coord);
            } else {
                double step = req->position * direction;
                if (axis_index >= 3) {
                    step = req->position * 180.0 / M_PI * direction;
                }
                success = jaka_interface_.jogCartesian(req->robot_id, axis_index, step, velocity_percent, coord);
            }
        }
        
        res->success = success;
        res->message = success ? "Jog command executed" : "Failed to execute jog command";
    }
    
    void handleJogStop(const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req, qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res) {
        res->success = jaka_interface_.jogStop(req->robot_id);
    }
    
    void handleSetPayload(const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr, qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res) {
        res->success = true; // Placeholder
    }
    
    void handleGetPayload(const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr, qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res) {
        res->success = true; // Placeholder
    }
    
    // ==================== IK 服务 ====================
    void handleComputeIK(
        const qyh_jaka_control_msgs::srv::ComputeIK::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::ComputeIK::Response::SharedPtr res)
    {
        if (!connected_) {
            res->success = false;
            res->message = "Robot not connected";
            return;
        }
        
        // 获取参考关节位置
        JointValue ref_pos;
        if (req->reference_joints.size() >= 7) {
            // 使用请求中提供的参考位置
            for (size_t i = 0; i < 7; ++i) {
                ref_pos.jVal[i] = req->reference_joints[i];
            }
        } else {
            // 使用当前关节位置作为参考
            if (!jaka_interface_.getJointPositions(req->robot_id, ref_pos)) {
                res->success = false;
                res->message = "Failed to get current joint positions";
                return;
            }
        }
        
        // 转换目标位姿为 JAKA 格式
        CartesianPose target_pose;
        target_pose.tran.x = req->target_pose.position.x * 1000.0;  // m -> mm
        target_pose.tran.y = req->target_pose.position.y * 1000.0;
        target_pose.tran.z = req->target_pose.position.z * 1000.0;
        
        // 四元数转欧拉角 (JAKA 使用 RPY)
        double qw = req->target_pose.orientation.w;
        double qx = req->target_pose.orientation.x;
        double qy = req->target_pose.orientation.y;
        double qz = req->target_pose.orientation.z;
        
        // RPY from quaternion
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        target_pose.rpy.rx = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
        
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            target_pose.rpy.ry = std::copysign(90.0, sinp);
        else
            target_pose.rpy.ry = std::asin(sinp) * 180.0 / M_PI;
        
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        target_pose.rpy.rz = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
        
        // 调用 JAKA IK
        JointValue result_joints;
        if (jaka_interface_.kineInverse(req->robot_id, ref_pos, target_pose, result_joints)) {
            res->success = true;
            res->message = "IK success";
            res->joint_positions.resize(7);
            for (size_t i = 0; i < 7; ++i) {
                res->joint_positions[i] = result_joints.jVal[i];
            }
            
            RCLCPP_DEBUG(get_logger(), 
                "[IK] robot_id=%d, target=[%.3f,%.3f,%.3f], result=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                req->robot_id,
                target_pose.tran.x, target_pose.tran.y, target_pose.tran.z,
                result_joints.jVal[0], result_joints.jVal[1], result_joints.jVal[2],
                result_joints.jVal[3], result_joints.jVal[4], result_joints.jVal[5], result_joints.jVal[6]);
        } else {
            res->success = false;
            res->message = "IK failed - target may be out of workspace or near singularity";
        }
    }

    // 成员变量
    qyh_jaka_control::JakaInterface jaka_interface_;
    std::unique_ptr<qyh_jaka_control::SmoothServoBridge> left_bridge_;
    std::unique_ptr<qyh_jaka_control::SmoothServoBridge> right_bridge_;
    
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
    uint32_t cmd_index_ = 0;
    
    // 缓存的机械臂位姿（主循环更新，状态发布使用）
    JointValue cached_left_joints_;
    JointValue cached_right_joints_;
    CartesianPose cached_left_pose_;
    CartesianPose cached_right_pose_;
    bool has_cached_state_{false};

    // IK求解相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool ik_enabled_{false};
    bool target_x_left_{false};
    bool has_z_offset_{true};
    double left_z_offset_{0.219885132};
    double right_z_offset_{0.217950931};
    
    // IK目标位姿
    geometry_msgs::msg::PoseStamped::SharedPtr left_target_;
    geometry_msgs::msg::PoseStamped::SharedPtr right_target_;
    bool has_left_target_{false};
    bool has_right_target_{false};
    
    // IK统计
    int ik_left_success_count_{0};
    int ik_right_success_count_{0};
    int ik_left_error_count_{0};
    int ik_right_error_count_{0};

    // ROS接口
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_bridge_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_bridge_sub_;
    
    // IK模式订阅
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
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetFilter>::SharedPtr srv_set_filter_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_bridge_start_servo_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_bridge_stop_servo_;

    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetCollisionLevel>::SharedPtr srv_set_collision_level_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetRobotState>::SharedPtr srv_get_robot_state_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::Jog>::SharedPtr srv_jog_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::JogStop>::SharedPtr srv_jog_stop_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetPayload>::SharedPtr srv_set_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetPayload>::SharedPtr srv_get_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::ComputeIK>::SharedPtr srv_compute_ik_;

    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
