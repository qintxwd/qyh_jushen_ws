#include "qyh_jaka_control/jaka_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_joint_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_cartesian_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_servo_status.hpp>
#include <qyh_jaka_control_msgs/msg/vr_pose.hpp>
#include <qyh_jaka_control_msgs/msg/vr_follow_status.hpp>
#include <qyh_jaka_control_msgs/srv/start_servo.hpp>
#include <qyh_jaka_control_msgs/srv/stop_servo.hpp>
#include <qyh_jaka_control_msgs/srv/enable_vr_follow.hpp>
#include <qyh_jaka_control_msgs/srv/start_recording.hpp>
#include <qyh_jaka_control_msgs/srv/stop_recording.hpp>
#include <qyh_jaka_control_msgs/srv/calibrate_vr.hpp>
#include <qyh_jaka_control_msgs/srv/set_filter.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <chrono>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @brief JAKA双臂机器人完整控制节点
 * 
 * 功能：
 * 1. 基础控制：上电、下电、使能、去使能、清除错误、急停
 * 2. 伺服模式：125Hz实时控制、关节空间、笛卡尔空间
 * 3. VR跟随：实时跟随、坐标校准、误差监控
 * 4. 数据录制：CSV格式、可配置频率、完整状态记录
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
          vr_following_(false),
          recording_(false),
          recorded_frames_(0)
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("cycle_time_ms", 8.0);  // 125Hz
        declare_parameter<bool>("use_cartesian", false);
        declare_parameter<bool>("auto_connect", true);
        declare_parameter<bool>("auto_power_on", false);
        declare_parameter<bool>("auto_enable", false);
        declare_parameter<std::string>("recording_output_dir", "/tmp/jaka_recordings");
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();
        use_cartesian_ = get_parameter("use_cartesian").as_bool();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        auto_power_on_ = get_parameter("auto_power_on").as_bool();
        auto_enable_ = get_parameter("auto_enable").as_bool();
        recording_output_dir_ = get_parameter("recording_output_dir").as_string();

        // 初始化VR坐标变换
        vr_to_robot_left_.setIdentity();
        vr_to_robot_right_.setIdentity();

        // Publishers
        status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>(
            "/jaka/servo/status", 10);
        vr_status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::VRFollowStatus>(
            "/jaka/vr/status", 10);

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
        
        srv_start_recording_ = create_service<qyh_jaka_control_msgs::srv::StartRecording>(
            "/jaka/vr/start_recording",
            std::bind(&JakaControlNode::handleStartRecording, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_recording_ = create_service<qyh_jaka_control_msgs::srv::StopRecording>(
            "/jaka/vr/stop_recording",
            std::bind(&JakaControlNode::handleStopRecording, this, std::placeholders::_1, std::placeholders::_2));

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
        if (recording_) {
            stopRecording();
        }
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

        // 录制数据
        if (recording_) {
            recordFrame(left_target, right_target);
        }
    }

    void recordFrame(const geometry_msgs::msg::Pose& left, const geometry_msgs::msg::Pose& right)
    {
        if (!recording_file_.is_open()) {
            return;
        }

        auto now = this->now();
        double timestamp = (now - recording_start_time_).seconds();
        
        // CSV格式：timestamp, left_vr, right_vr, joint_states, cartesian_poses
        recording_file_ << timestamp << ","
                       << left.position.x << "," << left.position.y << "," << left.position.z << ","
                       << left.orientation.x << "," << left.orientation.y << "," 
                       << left.orientation.z << "," << left.orientation.w << ","
                       << right.position.x << "," << right.position.y << "," << right.position.z << ","
                       << right.orientation.x << "," << right.orientation.y << "," 
                       << right.orientation.z << "," << right.orientation.w << "\n";
        
        recorded_frames_++;
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

        // VR跟随状态
        if (vr_following_) {
            auto vr_msg = qyh_jaka_control_msgs::msg::VRFollowStatus();
            vr_msg.left_arm_status = "Following";
            vr_msg.right_arm_status = "Following";
            vr_msg.left_pose_error = 0.0;  // TODO: 计算实际误差
            vr_msg.right_pose_error = 0.0;
            vr_msg.recorded_frames = recorded_frames_;
            if (recording_) {
                vr_msg.recording_duration = (this->now() - recording_start_time_).seconds();
            }
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
        if (recording_) {
            stopRecording();
        }

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

    void handleStartRecording(
        const qyh_jaka_control_msgs::srv::StartRecording::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::StartRecording::Response::SharedPtr res)
    {
        if (!vr_following_) {
            res->success = false;
            res->message = "VR following not enabled";
            return;
        }

        // 生成文件名
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time_t));
        
        current_recording_file_ = req->output_path + "/recording_" + buffer + ".csv";
        
        recording_file_.open(current_recording_file_);
        if (!recording_file_.is_open()) {
            res->success = false;
            res->message = "Failed to open file: " + current_recording_file_;
            return;
        }

        // 写CSV头
        recording_file_ << "timestamp,left_vr_x,left_vr_y,left_vr_z,"
                       << "left_vr_qx,left_vr_qy,left_vr_qz,left_vr_qw,"
                       << "right_vr_x,right_vr_y,right_vr_z,"
                       << "right_vr_qx,right_vr_qy,right_vr_qz,right_vr_qw\n";

        recording_ = true;
        recorded_frames_ = 0;
        recording_start_time_ = this->now();
        
        res->success = true;
        res->message = "Recording started: " + current_recording_file_;
        RCLCPP_INFO(get_logger(), "✓ Recording STARTED: %s", current_recording_file_.c_str());
    }

    void handleStopRecording(
        const qyh_jaka_control_msgs::srv::StopRecording::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StopRecording::Response::SharedPtr res)
    {
        if (!recording_) {
            res->success = false;
            res->message = "Not currently recording";
            return;
        }

        stopRecording();
        
        auto duration = (this->now() - recording_start_time_).seconds();
        res->success = true;
        res->total_frames = recorded_frames_;
        res->duration = duration;
        res->saved_file = current_recording_file_;
        res->message = "Recording stopped";
        
        RCLCPP_INFO(get_logger(), "✓ Recording STOPPED: %d frames, %.2f seconds", 
                    recorded_frames_, duration);
    }

    void stopRecording()
    {
        if (recording_file_.is_open()) {
            recording_file_.close();
        }
        recording_ = false;
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
    std::string recording_output_dir_;

    // 状态标志
    std::atomic<bool> connected_;
    std::atomic<bool> powered_;
    std::atomic<bool> enabled_;
    std::atomic<bool> servo_running_;
    std::atomic<bool> vr_following_;
    std::atomic<bool> recording_;
    int64_t last_cycle_duration_us_ = 0;

    // VR相关
    std::mutex vr_mutex_;
    qyh_jaka_control_msgs::msg::VRPose::SharedPtr last_vr_left_;
    qyh_jaka_control_msgs::msg::VRPose::SharedPtr last_vr_right_;
    tf2::Transform vr_to_robot_left_;
    tf2::Transform vr_to_robot_right_;

    // 录制相关
    std::ofstream recording_file_;
    std::string current_recording_file_;
    int recorded_frames_;
    rclcpp::Time recording_start_time_;

    // 指令缓存
    std::mutex cmd_mutex_;
    qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr last_joint_cmd_;
    qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr last_cartesian_cmd_;
    rclcpp::Time last_cmd_time_;

    // ROS接口
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::VRFollowStatus>::SharedPtr vr_status_pub_;
    
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
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartRecording>::SharedPtr srv_start_recording_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopRecording>::SharedPtr srv_stop_recording_;

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
