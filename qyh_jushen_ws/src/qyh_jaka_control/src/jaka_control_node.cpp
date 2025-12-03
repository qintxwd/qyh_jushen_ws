#include "qyh_jaka_control/jaka_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_joint_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_cartesian_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_servo_status.hpp>
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>
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
#include <chrono>
#include <atomic>
#include <mutex>
#include <set>
#include <map>
#include <array>

using namespace std::chrono_literals;

/**
 * @brief JAKA双臂机器人控制节点
 * 
 * 功能：
 * 1. 基础控制：上电、下电、使能、去使能、清除错误、急停
 * 2. 伺服模式：125Hz实时控制、关节空间、笛卡尔空间
 * 3. 接收命令：从遥操作控制器接收关节/笛卡尔命令
 * 4. 轨迹平滑：缓冲和插值（由 smooth_servo_bridge 处理）
 * 
 * 注意：VR 数据处理和坐标变换由 qyh_vr_calibration 和 qyh_teleoperation_controller 负责
 */
class JakaControlNode : public rclcpp::Node
{
public:
    JakaControlNode() 
        : Node("jaka_control_node"),
          jaka_interface_(this->get_logger()),
          connected_(false),
          powered_(false),
          enabled_(false),
          servo_running_(false)
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("cycle_time_ms", 8.0);  // 125Hz
        declare_parameter<bool>("use_cartesian", false);
        declare_parameter<bool>("auto_connect", true);
        declare_parameter<bool>("auto_initialize", false);
        
        // 初始化姿态参数
        declare_parameter<std::vector<double>>("open_pose_joints", 
            std::vector<double>{0.0, -0.5, 0.0, -1.57, 0.0, 1.07, 0.0,
                                0.0, -0.5, 0.0, -1.57, 0.0, 1.07, 0.0});
        declare_parameter<std::vector<double>>("home_pose_joints",
            std::vector<double>{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,
                                0.0, -0.785, 0.0, -2.356, 0.0, 1.571, -0.785});
        declare_parameter<double>("init_joint_velocity", 0.5);
        declare_parameter<double>("init_joint_acceleration", 0.3);
        declare_parameter<std::string>("default_filter_type", "joint_lpf");
        declare_parameter<double>("default_filter_cutoff", 5.0);
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();
        use_cartesian_ = get_parameter("use_cartesian").as_bool();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        auto_power_on_ = false;  // 废弃，使用auto_initialize
        auto_enable_ = false;     // 废弃，使用auto_initialize

        // Publishers
        status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>(
            "/jaka/servo/status", 10);
        robot_state_pub_ = create_publisher<qyh_jaka_control_msgs::msg::RobotState>(
            "/jaka/robot_state", 10);
        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        // Subscribers
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        joint_sub_ = create_subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>(
            "/jaka/servo/joint_cmd", qos,
            std::bind(&JakaControlNode::jointCmdCallback, this, std::placeholders::_1));
        
        cartesian_sub_ = create_subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>(
            "/jaka/servo/cartesian_cmd", qos,
            std::bind(&JakaControlNode::cartesianCmdCallback, this, std::placeholders::_1));

        // 基础控制服务
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

        // 伺服控制服务
        srv_start_servo_ = create_service<qyh_jaka_control_msgs::srv::StartServo>(
            "/jaka/servo/start",
            std::bind(&JakaControlNode::handleStartServo, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_servo_ = create_service<qyh_jaka_control_msgs::srv::StopServo>(
            "/jaka/servo/stop",
            std::bind(&JakaControlNode::handleStopServo, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_set_filter_ = create_service<qyh_jaka_control_msgs::srv::SetFilter>(
            "/jaka/servo/set_filter",
            std::bind(&JakaControlNode::handleSetFilter, this, std::placeholders::_1, std::placeholders::_2));

        // 点到点运动服务
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

        // 点动控制服务 (Jog control)
        srv_jog_ = create_service<qyh_jaka_control_msgs::srv::Jog>(
            "/jaka/jog",
            std::bind(&JakaControlNode::handleJog, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_jog_stop_ = create_service<qyh_jaka_control_msgs::srv::JogStop>(
            "/jaka/jog_stop",
            std::bind(&JakaControlNode::handleJogStop, this, std::placeholders::_1, std::placeholders::_2));

        // 负载管理服务 (Payload)
        srv_set_payload_ = create_service<qyh_jaka_control_msgs::srv::SetPayload>(
            "/jaka/set_payload",
            std::bind(&JakaControlNode::handleSetPayload, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_get_payload_ = create_service<qyh_jaka_control_msgs::srv::GetPayload>(
            "/jaka/get_payload",
            std::bind(&JakaControlNode::handleGetPayload, this, std::placeholders::_1, std::placeholders::_2));

        // 自动连接和初始化
        if (auto_connect_) {
            if (jaka_interface_.connect(robot_ip_)) {
                connected_ = true;
                RCLCPP_INFO(get_logger(), "✓ Connected to robot at %s", robot_ip_.c_str());
                
                // 自动初始化流程
                bool auto_initialize = get_parameter("auto_initialize").as_bool();
                if (auto_initialize) {
                    RCLCPP_INFO(get_logger(), "Starting auto-initialization sequence...");
                    
                    if (autoInitialize()) {
                        RCLCPP_INFO(get_logger(), "✓ Auto-initialization completed successfully");
                    } else {
                        RCLCPP_ERROR(get_logger(), "✗ Auto-initialization failed");
                    }
                }
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to connect to robot at %s", robot_ip_.c_str());
            }
        }

        // 主循环定时器
        auto period = std::chrono::microseconds(static_cast<int>(cycle_time_ms_ * 1000));
        main_timer_ = create_wall_timer(period, std::bind(&JakaControlNode::mainLoop, this));

        // 状态发布定时器（10Hz）
        status_timer_ = create_wall_timer(100ms, std::bind(&JakaControlNode::publishStatus, this));

        RCLCPP_INFO(get_logger(), "=== JAKA Control Node Initialized ===");
        RCLCPP_INFO(get_logger(), "Cycle time: %.2f ms (%.1f Hz)", cycle_time_ms_, 1000.0/cycle_time_ms_);
        RCLCPP_INFO(get_logger(), "Connected: %s, Powered: %s, Enabled: %s",
                    connected_ ? "YES" : "NO",
                    powered_ ? "YES" : "NO",
                    enabled_ ? "YES" : "NO");
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
        if (!servo_running_) {
            return;
        }

        auto start = std::chrono::high_resolution_clock::now();

        // 关节指令模式
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto now = this->now();
        
        // 超时保护（500ms无指令则停止）
        if (last_cmd_time_.seconds() > 0 && 
            (now - last_cmd_time_).seconds() > 0.5) {
            // 超时，不发送指令
            return;
        }

        // 发送关节或笛卡尔指令
        if (last_joint_cmd_) {
            std::vector<double> positions(last_joint_cmd_->positions.begin(), 
                                         last_joint_cmd_->positions.end());
            jaka_interface_.servoJ(-1, positions, last_joint_cmd_->is_abs);
            
            // Send command to robot (CRITICAL!)
            jaka_interface_.edgSend();
        }
        else if (last_cartesian_cmd_) {
            // 笛卡尔控制（待实现双臂分别控制）
        }

        auto end = std::chrono::high_resolution_clock::now();
        last_cycle_duration_us_ = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

    void publishStatus()
    {
        // 伺服状态
        auto servo_msg = qyh_jaka_control_msgs::msg::JakaServoStatus();
        // 根据 servo_running_ 状态设置 mode
        if (servo_running_) {
            servo_msg.mode = use_cartesian_ ? "cartesian" : "joint";
        } else {
            servo_msg.mode = "idle";
        }
        servo_msg.is_abs = true;
        servo_msg.cycle_time_ns = static_cast<int32_t>(cycle_time_ms_ * 1e6);
        servo_msg.publish_rate_hz = servo_running_ ? (1000.0 / cycle_time_ms_) : 0.0;
        servo_msg.latency_ms = last_cycle_duration_us_ / 1000.0;
        servo_msg.packet_loss_rate = 0.0;
        servo_msg.error_code = 0;
        status_pub_->publish(servo_msg);

        // 机器人状态
        auto robot_state_msg = qyh_jaka_control_msgs::msg::RobotState();
        robot_state_msg.header.stamp = this->now();
        robot_state_msg.header.frame_id = "world";
        robot_state_msg.connected = connected_;
        robot_state_msg.robot_ip = robot_ip_;

        RobotState state;
        if (jaka_interface_.getRobotState(state)) {
            robot_state_msg.powered_on = state.poweredOn;
            robot_state_msg.enabled = state.servoEnabled;
            robot_state_msg.in_estop = state.estoped;
            robot_state_msg.servo_mode_enabled = servo_running_;  // 伺服模式状态

            // Only query detailed status if powered on
            if (state.poweredOn) {
                int error[2] = {0, 0};
                // Try to get error state
                if (jaka_interface_.isInError(error)) {
                    robot_state_msg.in_error = (error[0] || error[1]);
                }

                // Only query pose if not in error (to avoid SDK errors)
                if (!robot_state_msg.in_error) {
                    int inpos[2] = {0, 0};
                    if (jaka_interface_.isInPosition(inpos)) {
                        robot_state_msg.left_in_position = inpos[0];
                        robot_state_msg.right_in_position = inpos[1];
                    }

                    // 获取关节位置
                    JointValue left_joint, right_joint;
                    std::array<double, 7> left_joint_positions{};
                    std::array<double, 7> right_joint_positions{};

                    if (jaka_interface_.getJointPositions(0, left_joint)) {
                        for (size_t i = 0; i < left_joint_positions.size(); ++i) {
                            left_joint_positions[i] = left_joint.jVal[i];
                        }
                    }
                    if (jaka_interface_.getJointPositions(1, right_joint)) {
                        for (size_t i = 0; i < right_joint_positions.size(); ++i) {
                            right_joint_positions[i] = right_joint.jVal[i];
                        }
                    }

                    robot_state_msg.left_joint_positions = left_joint_positions;
                    robot_state_msg.right_joint_positions = right_joint_positions;

                    // 获取笛卡尔位姿
                    CartesianPose left_pose, right_pose;
                    if (jaka_interface_.getCartesianPose(0, left_pose)) {
                        robot_state_msg.left_cartesian_pose = jaka_interface_.jakaPoseToRos(left_pose);
                    }
                    if (jaka_interface_.getCartesianPose(1, right_pose)) {
                        robot_state_msg.right_cartesian_pose = jaka_interface_.jakaPoseToRos(right_pose);
                    }
                } else {
                    // 错误信息
                    ErrorCode error_code;
                    if (jaka_interface_.getLastError(error_code)) {
                        robot_state_msg.error_message = error_code.message;
                    }
                }
            }
        }

        robot_state_pub_->publish(robot_state_msg);

        // 发布标准JointState消息（用于RViz）
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = "world";
        
        // 左臂关节名称
        for (int i = 1; i <= 7; ++i) {
            joint_state_msg.name.push_back("left_joint_" + std::to_string(i));
        }
        // 右臂关节名称
        for (int i = 1; i <= 7; ++i) {
            joint_state_msg.name.push_back("right_joint_" + std::to_string(i));
        }
        
        joint_state_msg.position.assign(
            robot_state_msg.left_joint_positions.begin(),
            robot_state_msg.left_joint_positions.end());
        joint_state_msg.position.insert(
            joint_state_msg.position.end(),
            robot_state_msg.right_joint_positions.begin(),
            robot_state_msg.right_joint_positions.end());
        
        joint_states_pub_->publish(joint_state_msg);
    }

    // ==================== 回调函数 ====================
    void jointCmdCallback(const qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_joint_cmd_ = msg;
        last_cmd_time_ = this->now();
    }

    void cartesianCmdCallback(const qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cartesian_cmd_ = msg;
        last_cmd_time_ = this->now();
    }

    // ==================== 基础控制服务 ====================
    void handlePowerOn(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!connected_) {
            res->success = false;
            res->message = "Robot not connected";
            return;
        }

        if (jaka_interface_.powerOn()) {
            powered_ = true;
            res->success = true;
            res->message = "Robot powered on successfully";
            RCLCPP_INFO(get_logger(), "✓ Power ON");
        } else {
            res->success = false;
            res->message = "Failed to power on robot";
            RCLCPP_ERROR(get_logger(), "✗ Power ON failed");
        }
    }

    void handlePowerOff(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (servo_running_) {
            res->success = false;
            res->message = "Stop servo mode first";
            return;
        }

        if (jaka_interface_.powerOff()) {
            powered_ = false;
            enabled_ = false;
            res->success = true;
            res->message = "Robot powered off successfully";
            RCLCPP_INFO(get_logger(), "✓ Power OFF");
        } else {
            res->success = false;
            res->message = "Failed to power off robot";
        }
    }

    void handleEnable(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (!powered_) {
            res->success = false;
            res->message = "Robot not powered on";
            return;
        }

        if (jaka_interface_.enableRobot()) {
            enabled_ = true;
            res->success = true;
            res->message = "Robot enabled successfully";
            RCLCPP_INFO(get_logger(), "✓ Robot ENABLED");
        } else {
            res->success = false;
            res->message = "Failed to enable robot";
        }
    }

    void handleDisable(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (servo_running_) {
            res->success = false;
            res->message = "Stop servo mode first";
            return;
        }

        if (jaka_interface_.disableRobot()) {
            enabled_ = false;
            res->success = true;
            res->message = "Robot disabled successfully";
            RCLCPP_INFO(get_logger(), "✓ Robot DISABLED");
        } else {
            res->success = false;
            res->message = "Failed to disable robot";
        }
    }

    void handleClearError(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (jaka_interface_.clearError()) {
            res->success = true;
            res->message = "Error cleared successfully";
            RCLCPP_INFO(get_logger(), "✓ Error CLEARED");
        } else {
            res->success = false;
            res->message = "Failed to clear error";
        }
    }

    void handleMotionAbort(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (jaka_interface_.motionAbort()) {
            servo_running_ = false;
            res->success = true;
            res->message = "Motion aborted successfully";
            RCLCPP_WARN(get_logger(), "⚠ Motion ABORTED");
        } else {
            res->success = false;
            res->message = "Failed to abort motion";
        }
    }

    // ==================== 伺服控制服务 ====================
    void handleStartServo(
        const qyh_jaka_control_msgs::srv::StartServo::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StartServo::Response::SharedPtr res)
    {
        if (!enabled_) {
            res->success = false;
            res->message = "Robot not enabled";
            return;
        }

        // Apply default filter to handle jitter from non-realtime sources (like Python)
        std::string filter_type = get_parameter("default_filter_type").as_string();
        double cutoff = get_parameter("default_filter_cutoff").as_double();
        
        if (filter_type == "joint_lpf") {
            jaka_interface_.setFilterJointLPF(cutoff);
            RCLCPP_INFO(get_logger(), "Applied default filter: Joint LPF (%.1f Hz)", cutoff);
        } else if (filter_type == "none") {
            jaka_interface_.setFilterNone();
            RCLCPP_INFO(get_logger(), "Applied default filter: None");
        }

        if (jaka_interface_.servoMoveEnable(true)) {
            servo_running_ = true;
            res->success = true;
            res->message = "Servo mode started";
            RCLCPP_INFO(get_logger(), "✓ Servo mode STARTED");
        } else {
            res->success = false;
            res->message = "Failed to start servo mode";
        }
    }

    void handleStopServo(
        const qyh_jaka_control_msgs::srv::StopServo::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StopServo::Response::SharedPtr res)
    {
        if (jaka_interface_.servoMoveEnable(false)) {
            servo_running_ = false;
            res->success = true;
            res->message = "Servo mode stopped";
            RCLCPP_INFO(get_logger(), "✓ Servo mode STOPPED");
        } else {
            res->success = false;
            res->message = "Failed to stop servo mode";
        }
    }

    void handleSetFilter(
        const qyh_jaka_control_msgs::srv::SetFilter::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::SetFilter::Response::SharedPtr res)
    {
        bool success = false;
        
        if (req->filter_type == "none") {
            success = jaka_interface_.setFilterNone();
        } else if (req->filter_type == "joint_lpf") {
            success = jaka_interface_.setFilterJointLPF(req->cutoff_frequency);
        } else if (req->filter_type == "joint_nlf") {
            success = jaka_interface_.setFilterJointNLF(
                req->max_joint_velocity, req->max_joint_acceleration, req->max_joint_jerk);
        }
        
        res->success = success;
        res->message = success ? "Filter set successfully" : "Failed to set filter";
    }

    // ==================== 自动初始化流程 ====================
    bool autoInitialize()
    {
        // 步骤1: 上电
        RCLCPP_INFO(get_logger(), "[1/6] Powering on robot...");
        if (!jaka_interface_.powerOn()) {
            RCLCPP_ERROR(get_logger(), "Failed to power on robot");
            return false;
        }
        powered_ = true;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(get_logger(), "✓ Robot powered on");

        // 步骤2: 使能
        RCLCPP_INFO(get_logger(), "[2/6] Enabling robot...");
        if (!jaka_interface_.enableRobot()) {
            RCLCPP_ERROR(get_logger(), "Failed to enable robot");
            return false;
        }
        enabled_ = true;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(get_logger(), "✓ Robot enabled");

        // 步骤3: 移动到张开姿态
        RCLCPP_INFO(get_logger(), "[3/6] Moving to open pose...");
        auto open_pose = get_parameter("open_pose_joints").as_double_array();
        double vel = get_parameter("init_joint_velocity").as_double();
        double acc = get_parameter("init_joint_acceleration").as_double();
        
        if (!jaka_interface_.moveJ(-1, open_pose, false, vel, acc, true)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to open pose");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(get_logger(), "✓ Moved to open pose");

        // 步骤4: 移动到初始位置
        RCLCPP_INFO(get_logger(), "[4/6] Moving to home pose...");
        auto home_pose = get_parameter("home_pose_joints").as_double_array();
        
        if (!jaka_interface_.moveJ(-1, home_pose, false, vel, acc, true)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to home pose");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(get_logger(), "✓ Moved to home pose");

        // 步骤5: 启动伺服模式
        RCLCPP_INFO(get_logger(), "[5/6] Starting servo mode...");
        if (!jaka_interface_.servoMoveEnable(true, -1)) {
            RCLCPP_ERROR(get_logger(), "Failed to start servo mode");
            return false;
        }
        servo_running_ = true;
        RCLCPP_INFO(get_logger(), "✓ Servo mode started");

        return true;
    }

    // ==================== 点到点运动服务 ====================
    void handleMoveJ(
        const qyh_jaka_control_msgs::srv::MoveJ::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::MoveJ::Response::SharedPtr res)
    {
        if (!enabled_) {
            res->success = false;
            res->message = "Robot not enabled";
            return;
        }

        std::vector<double> positions(req->joint_positions.begin(), req->joint_positions.end());
        res->success = jaka_interface_.moveJ(
            req->robot_id, positions, req->move_mode,
            req->velocity, req->acceleration, req->is_block);
        res->message = res->success ? "MoveJ executed successfully" : "Failed to execute MoveJ";
    }

    void handleMoveL(
        const qyh_jaka_control_msgs::srv::MoveL::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::MoveL::Response::SharedPtr res)
    {
        if (!enabled_) {
            res->success = false;
            res->message = "Robot not enabled";
            return;
        }

        res->success = jaka_interface_.moveL(
            req->robot_id, req->target_pose, req->move_mode,
            req->velocity, req->acceleration, req->is_block);
        res->message = res->success ? "MoveL executed successfully" : "Failed to execute MoveL";
    }

    void handleSetCollisionLevel(
        const qyh_jaka_control_msgs::srv::SetCollisionLevel::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::SetCollisionLevel::Response::SharedPtr res)
    {
        res->success = jaka_interface_.setCollisionLevel(req->robot_id, req->level);
        res->message = res->success ? "Collision level set successfully" : "Failed to set collision level";
    }

    void handleSetToolOffset(
        const qyh_jaka_control_msgs::srv::SetToolOffset::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::SetToolOffset::Response::SharedPtr res)
    {
        res->success = jaka_interface_.setToolOffset(req->robot_id, req->tool_offset);
        res->message = res->success ? "Tool offset set successfully" : "Failed to set tool offset";
    }

    void handleGetRobotState(
        const qyh_jaka_control_msgs::srv::GetRobotState::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::GetRobotState::Response::SharedPtr res)
    {
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

    // ==================== 点动控制服务 ====================
    void handleJog(
        const qyh_jaka_control_msgs::srv::Jog::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::Jog::Response::SharedPtr res)
    {
        if (!enabled_) {
            res->success = false;
            res->message = "Robot not enabled";
            return;
        }

        // 停止伺服模式下的jog可能会影响已有的servo命令
        // 对于步进模式，直接执行
        // 对于连续模式，需要持续发送命令（由调用方负责）
        
        res->success = jaka_interface_.jog(
            req->robot_id,
            req->axis_num,
            req->move_mode,
            req->coord_type,
            req->velocity,
            req->position
        );
        res->message = res->success ? "Jog command executed" : "Failed to execute jog command";
        
        if (res->success) {
            RCLCPP_INFO(get_logger(), "Jog: robot=%d, axis=%d, mode=%d, coord=%d, vel=%.3f, pos=%.3f",
                req->robot_id, req->axis_num, req->move_mode, req->coord_type, 
                req->velocity, req->position);
        }
    }

    void handleJogStop(
        const qyh_jaka_control_msgs::srv::JogStop::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::JogStop::Response::SharedPtr res)
    {
        res->success = jaka_interface_.jogStop(req->robot_id, req->axis_num);
        res->message = res->success ? "Jog stopped" : "Failed to stop jog";
        
        if (res->success) {
            RCLCPP_INFO(get_logger(), "Jog stopped: robot=%d, axis=%d", req->robot_id, req->axis_num);
        }
    }

    void handleSetPayload(
        const qyh_jaka_control_msgs::srv::SetPayload::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::SetPayload::Response::SharedPtr res)
    {
        // 质心固定为 x=150mm (末端向前15cm)
        res->success = jaka_interface_.setPayload(req->robot_id, req->mass, 150.0);
        res->message = res->success ? 
            "Payload set: " + std::to_string(req->mass) + " kg" : 
            "Failed to set payload";
        
        if (res->success) {
            RCLCPP_INFO(get_logger(), "Payload set: robot=%d, mass=%.2f kg", 
                req->robot_id, req->mass);
        }
    }

    void handleGetPayload(
        const qyh_jaka_control_msgs::srv::GetPayload::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::GetPayload::Response::SharedPtr res)
    {
        double mass, cx, cy, cz;
        res->success = jaka_interface_.getPayload(req->robot_id, mass, cx, cy, cz);
        
        if (res->success) {
            res->mass = mass;
            res->centroid_x = cx;
            res->centroid_y = cy;
            res->centroid_z = cz;
            res->message = "Payload retrieved";
            RCLCPP_INFO(get_logger(), "Payload: robot=%d, mass=%.2f kg, centroid=(%.1f, %.1f, %.1f) mm",
                req->robot_id, mass, cx, cy, cz);
        } else {
            res->message = "Failed to get payload";
        }
    }

    // ==================== 成员变量 ====================
    qyh_jaka_control::JakaInterface jaka_interface_;
    
    // 参数
    std::string robot_ip_;
    double cycle_time_ms_;
    bool use_cartesian_;
    bool auto_connect_;
    bool auto_power_on_;
    bool auto_enable_;

    // 状态标志
    std::atomic<bool> connected_;
    std::atomic<bool> powered_;
    std::atomic<bool> enabled_;
    std::atomic<bool> servo_running_;
    int64_t last_cycle_duration_us_ = 0;

    // 指令缓存
    std::mutex cmd_mutex_;
    qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr last_joint_cmd_;
    qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr last_cartesian_cmd_;
    rclcpp::Time last_cmd_time_;

    // ROS接口
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>::SharedPtr joint_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>::SharedPtr cartesian_sub_;

    // 基础控制服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_on_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_power_off_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_disable_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_error_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_motion_abort_;
    
    // 伺服控制服务
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartServo>::SharedPtr srv_start_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopServo>::SharedPtr srv_stop_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetFilter>::SharedPtr srv_set_filter_;

    // 点到点运动服务
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetCollisionLevel>::SharedPtr srv_set_collision_level_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetRobotState>::SharedPtr srv_get_robot_state_;

    // 点动控制服务 (Jog control)
    rclcpp::Service<qyh_jaka_control_msgs::srv::Jog>::SharedPtr srv_jog_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::JogStop>::SharedPtr srv_jog_stop_;

    // 负载管理服务 (Payload)
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetPayload>::SharedPtr srv_set_payload_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetPayload>::SharedPtr srv_get_payload_;

    // 定时器
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
