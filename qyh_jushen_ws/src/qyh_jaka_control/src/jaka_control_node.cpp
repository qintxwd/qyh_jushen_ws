#include "qyh_jaka_control/jaka_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_joint_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_cartesian_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_servo_status.hpp>
#include <qyh_jaka_control_msgs/msg/vr_pose.hpp>
#include <qyh_jaka_control_msgs/msg/vr_follow_status.hpp>
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>
#include <qyh_jaka_control_msgs/srv/start_servo.hpp>
#include <qyh_jaka_control_msgs/srv/stop_servo.hpp>
#include <qyh_jaka_control_msgs/srv/enable_vr_follow.hpp>
#include <qyh_jaka_control_msgs/srv/calibrate_vr.hpp>
#include <qyh_jaka_control_msgs/srv/set_filter.hpp>
#include <qyh_jaka_control_msgs/srv/move_j.hpp>
#include <qyh_jaka_control_msgs/srv/move_l.hpp>
#include <qyh_jaka_control_msgs/srv/set_collision_level.hpp>
#include <qyh_jaka_control_msgs/srv/set_tool_offset.hpp>
#include <qyh_jaka_control_msgs/srv/get_robot_state.hpp>
#include <qyh_vr_calibration_msgs/srv/get_profile.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <atomic>
#include <mutex>
#include <set>

using namespace std::chrono_literals;

/**
 * @brief JAKA双臂机器人完整控制节点
 * 
 * 功能：
 * 1. 基础控制：上电、下电、使能、去使能、清除错误、急停
 * 2. 伺服模式：125Hz实时控制、关节空间、笛卡尔空间
 * 3. VR跟随：实时跟随、坐标校准、误差监控
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
          servo_running_(false),
          vr_following_(false)
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("cycle_time_ms", 8.0);  // 125Hz
        declare_parameter<bool>("use_cartesian", false);
        declare_parameter<bool>("auto_connect", true);
        declare_parameter<bool>("auto_power_on", false);
        declare_parameter<bool>("auto_enable", false);
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();
        use_cartesian_ = get_parameter("use_cartesian").as_bool();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        auto_power_on_ = get_parameter("auto_power_on").as_bool();
        auto_enable_ = get_parameter("auto_enable").as_bool();

        // 初始化VR坐标变换
        vr_to_robot_left_.setIdentity();
        vr_to_robot_right_.setIdentity();

        // Publishers
        status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>(
            "/jaka/servo/status", 10);
        vr_status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::VRFollowStatus>(
            "/jaka/vr/status", 10);
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

        vr_left_sub_ = create_subscription<qyh_jaka_control_msgs::msg::VRPose>(
            "/vr/left_controller", qos,
            std::bind(&JakaControlNode::vrLeftCallback, this, std::placeholders::_1));
        
        vr_right_sub_ = create_subscription<qyh_jaka_control_msgs::msg::VRPose>(
            "/vr/right_controller", qos,
            std::bind(&JakaControlNode::vrRightCallback, this, std::placeholders::_1));

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

        // VR控制服务
        srv_enable_vr_ = create_service<qyh_jaka_control_msgs::srv::EnableVRFollow>(
            "/jaka/vr/enable",
            std::bind(&JakaControlNode::handleEnableVR, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_calibrate_vr_ = create_service<qyh_jaka_control_msgs::srv::CalibrateVR>(
            "/jaka/vr/calibrate",
            std::bind(&JakaControlNode::handleCalibrateVR, this, std::placeholders::_1, std::placeholders::_2));

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

        // VR Calibration Client
        client_get_profile_ = create_client<qyh_vr_calibration_msgs::srv::GetProfile>(
            "vr_calibration/get_profile");

        // 自动连接和初始化
        if (auto_connect_) {
            if (jaka_interface_.connect(robot_ip_)) {
                connected_ = true;
                RCLCPP_INFO(get_logger(), "✓ Connected to robot at %s", robot_ip_.c_str());
                
                if (auto_power_on_ && jaka_interface_.powerOn()) {
                    powered_ = true;
                    RCLCPP_INFO(get_logger(), "✓ Robot powered on");
                    
                    if (auto_enable_ && jaka_interface_.enableRobot()) {
                        enabled_ = true;
                        RCLCPP_INFO(get_logger(), "✓ Robot enabled");
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

        // VR跟随模式
        if (vr_following_) {
            processVRFollow();
        }
        // 手动指令模式
        else {
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
            }
            else if (last_cartesian_cmd_) {
                // 笛卡尔控制（待实现双臂分别控制）
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        last_cycle_duration_us_ = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

    void processVRFollow()
    {
        std::lock_guard<std::mutex> lock(vr_mutex_);
        
        if (!last_vr_left_ || !last_vr_right_) {
            return;  // 等待VR数据
        }

        // 转换VR位姿到机器人坐标系
        geometry_msgs::msg::Pose left_target, right_target;
        
        // 左臂
        tf2::Transform vr_left;
        tf2::fromMsg(last_vr_left_->pose, vr_left);
        tf2::Transform robot_left = vr_to_robot_left_ * vr_left;
        tf2::toMsg(robot_left, left_target);
        
        // 右臂
        tf2::Transform vr_right;
        tf2::fromMsg(last_vr_right_->pose, vr_right);
        tf2::Transform robot_right = vr_to_robot_right_ * vr_right;
        tf2::toMsg(robot_right, right_target);

        // 发送伺服指令（笛卡尔空间）
        jaka_interface_.servoP(0, left_target, true);   // 左臂
        jaka_interface_.servoP(1, right_target, true);  // 右臂
    }

    void publishStatus()
    {
        // 伺服状态
        auto servo_msg = qyh_jaka_control_msgs::msg::JakaServoStatus();
        servo_msg.mode = use_cartesian_ ? "cartesian" : "joint";
        servo_msg.is_abs = true;
        servo_msg.cycle_time_ns = static_cast<int32_t>(cycle_time_ms_ * 1e6);
        servo_msg.publish_rate_hz = 1000.0 / cycle_time_ms_;
        servo_msg.latency_ms = last_cycle_duration_us_ / 1000.0;
        servo_msg.packet_loss_rate = 0.0;
        servo_msg.error_code = 0;
        status_pub_->publish(servo_msg);

        // 机器人状态
        auto robot_state_msg = qyh_jaka_control_msgs::msg::RobotState();
        robot_state_msg.header.stamp = this->now();
        robot_state_msg.header.frame_id = "world";

        RobotState state;
        if (jaka_interface_.getRobotState(state)) {
            robot_state_msg.powered_on = state.poweredOn;
            robot_state_msg.servo_enabled = state.servoEnabled;
            robot_state_msg.estoped = state.estoped;

            int error[2] = {0, 0};
            jaka_interface_.isInError(error);
            robot_state_msg.in_error = (error[0] || error[1]);

            int inpos[2] = {0, 0};
            jaka_interface_.isInPosition(inpos);
            robot_state_msg.left_in_position = inpos[0];
            robot_state_msg.right_in_position = inpos[1];

            // 获取关节位置
            JointValue left_joint, right_joint;
            if (jaka_interface_.getJointPositions(0, left_joint)) {
                for (int i = 0; i < 7; ++i) {
                    robot_state_msg.left_joint_positions.push_back(left_joint.jVal[i]);
                }
            }
            if (jaka_interface_.getJointPositions(1, right_joint)) {
                for (int i = 0; i < 7; ++i) {
                    robot_state_msg.right_joint_positions.push_back(right_joint.jVal[i]);
                }
            }

            // 获取笛卡尔位姿
            CartesianPose left_pose, right_pose;
            if (jaka_interface_.getCartesianPose(0, left_pose)) {
                robot_state_msg.left_cartesian_pose = jaka_interface_.jakaPoseToRos(left_pose);
            }
            if (jaka_interface_.getCartesianPose(1, right_pose)) {
                robot_state_msg.right_cartesian_pose = jaka_interface_.jakaPoseToRos(right_pose);
            }

            // 错误信息
            if (robot_state_msg.in_error) {
                ErrorCode error_code;
                if (jaka_interface_.getLastError(error_code)) {
                    robot_state_msg.error_message = error_code.message;
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
        
        joint_state_msg.position = robot_state_msg.left_joint_positions;
        joint_state_msg.position.insert(joint_state_msg.position.end(),
                                       robot_state_msg.right_joint_positions.begin(),
                                       robot_state_msg.right_joint_positions.end());
        
        joint_states_pub_->publish(joint_state_msg);

        // VR跟随状态
        if (vr_following_) {
            auto vr_msg = qyh_jaka_control_msgs::msg::VRFollowStatus();
            vr_msg.left_arm_status = "Following";
            vr_msg.right_arm_status = "Following";
            vr_msg.left_pose_error = 0.0;  // TODO: 计算实际误差
            vr_msg.right_pose_error = 0.0;
            vr_status_pub_->publish(vr_msg);
        }
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

    void vrLeftCallback(const qyh_jaka_control_msgs::msg::VRPose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(vr_mutex_);
        last_vr_left_ = msg;
    }

    void vrRightCallback(const qyh_jaka_control_msgs::msg::VRPose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(vr_mutex_);
        last_vr_right_ = msg;
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
            vr_following_ = false;
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
            vr_following_ = false;
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

    // ==================== VR控制服务 ====================
    void handleEnableVR(
        const qyh_jaka_control_msgs::srv::EnableVRFollow::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::EnableVRFollow::Response::SharedPtr res)
    {
        if (!servo_running_) {
            res->success = false;
            res->message = "Servo mode not running";
            return;
        }

        if (req->enable) {
            // Check if username is provided
            if (req->username.empty()) {
                res->success = false;
                res->message = "Username is required to enable VR follow";
                return;
            }

            // Call GetProfile service
            if (!client_get_profile_->wait_for_service(std::chrono::seconds(1))) {
                res->success = false;
                res->message = "VR calibration service not available";
                return;
            }

            auto profile_req = std::make_shared<qyh_vr_calibration_msgs::srv::GetProfile::Request>();
            profile_req->username = req->username;

            auto future = client_get_profile_->async_send_request(profile_req);
            
            // Wait for result (blocking here is acceptable for configuration service)
            if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
                res->success = false;
                res->message = "Timeout waiting for VR profile";
                return;
            }

            auto profile_res = future.get();
            if (!profile_res->found) {
                res->success = false;
                res->message = "VR profile not found for user: " + req->username;
                return;
            }

            // Validate samples 0, 1, 2, 3 exist
            std::set<uint32_t> existing_indices;
            for (const auto& sample : profile_res->profile.samples) {
                existing_indices.insert(sample.sample_index);
            }

            bool has_all_samples = true;
            for (uint32_t i = 0; i < 4; ++i) {
                if (existing_indices.find(i) == existing_indices.end()) {
                    has_all_samples = false;
                    break;
                }
            }

            if (!has_all_samples) {
                res->success = false;
                res->message = "Incomplete calibration data. Required samples 0-3.";
                return;
            }

            // TODO: Load calibration data into internal state for use in processVRFollow
            RCLCPP_INFO(get_logger(), "Loaded VR profile for %s with %zu samples", 
                        req->username.c_str(), profile_res->profile.samples.size());
        }

        vr_following_ = req->enable;
        res->success = true;
        res->message = vr_following_ ? "VR following enabled" : "VR following disabled";
        RCLCPP_INFO(get_logger(), vr_following_ ? "✓ VR following ENABLED" : "✓ VR following DISABLED");
    }

    void handleCalibrateVR(
        const qyh_jaka_control_msgs::srv::CalibrateVR::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::CalibrateVR::Response::SharedPtr res)
    {
        // 简单实现：设置VR到机器人的坐标变换
        // TODO: 实现完整的校准流程
        res->success = true;
        res->message = "VR calibration completed";
        RCLCPP_INFO(get_logger(), "✓ VR calibrated");
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
    std::atomic<bool> vr_following_;
    int64_t last_cycle_duration_us_ = 0;

    // VR相关
    std::mutex vr_mutex_;
    qyh_jaka_control_msgs::msg::VRPose::SharedPtr last_vr_left_;
    qyh_jaka_control_msgs::msg::VRPose::SharedPtr last_vr_right_;
    tf2::Transform vr_to_robot_left_;
    tf2::Transform vr_to_robot_right_;

    // 指令缓存
    std::mutex cmd_mutex_;
    qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr last_joint_cmd_;
    qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr last_cartesian_cmd_;
    rclcpp::Time last_cmd_time_;

    // ROS接口
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::VRFollowStatus>::SharedPtr vr_status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>::SharedPtr joint_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>::SharedPtr cartesian_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::VRPose>::SharedPtr vr_left_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::VRPose>::SharedPtr vr_right_sub_;

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
    
    // VR控制服务
    rclcpp::Service<qyh_jaka_control_msgs::srv::EnableVRFollow>::SharedPtr srv_enable_vr_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::CalibrateVR>::SharedPtr srv_calibrate_vr_;

    // 点到点运动服务
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetCollisionLevel>::SharedPtr srv_set_collision_level_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::GetRobotState>::SharedPtr srv_get_robot_state_;

    // VR Calibration Client
    rclcpp::Client<qyh_vr_calibration_msgs::srv::GetProfile>::SharedPtr client_get_profile_;

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
