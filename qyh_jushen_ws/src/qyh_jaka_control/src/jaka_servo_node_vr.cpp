#include "qyh_jaka_control/jaka_interface.hpp"
#include <rclcpp/rclcpp.hpp>
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

class JakaServoNode : public rclcpp::Node
{
public:
    JakaServoNode() 
        : Node("jaka_servo_node"),
          jaka_interface_(this->get_logger()),
          servo_running_(false),
          vr_following_(false),
          recording_(false),
          recorded_frames_(0)
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("cycle_time_ms", 8.0);  // 125Hz默认
        declare_parameter<bool>("use_cartesian", false);
        declare_parameter<bool>("auto_connect", true);
        declare_parameter<std::string>("recording_output_dir", "/tmp/jaka_recordings");
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();
        use_cartesian_ = get_parameter("use_cartesian").as_bool();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        recording_output_dir_ = get_parameter("recording_output_dir").as_string();

        // 初始化VR坐标变换（单位矩阵）
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
            std::bind(&JakaServoNode::jointCmdCallback, this, std::placeholders::_1));
        
        cartesian_sub_ = create_subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>(
            "/jaka/servo/cartesian_cmd", qos,
            std::bind(&JakaServoNode::cartesianCmdCallback, this, std::placeholders::_1));

        vr_left_sub_ = create_subscription<qyh_jaka_control_msgs::msg::VRPose>(
            "/vr/left_controller", qos,
            std::bind(&JakaServoNode::vrLeftCallback, this, std::placeholders::_1));
        
        vr_right_sub_ = create_subscription<qyh_jaka_control_msgs::msg::VRPose>(
            "/vr/right_controller", qos,
            std::bind(&JakaServoNode::vrRightCallback, this, std::placeholders::_1));

        // Services
        srv_start_servo_ = create_service<qyh_jaka_control_msgs::srv::StartServo>(
            "/jaka/servo/start",
            std::bind(&JakaServoNode::handleStartServo, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_servo_ = create_service<qyh_jaka_control_msgs::srv::StopServo>(
            "/jaka/servo/stop",
            std::bind(&JakaServoNode::handleStopServo, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_enable_vr_ = create_service<qyh_jaka_control_msgs::srv::EnableVRFollow>(
            "/jaka/vr/enable",
            std::bind(&JakaServoNode::handleEnableVR, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_start_recording_ = create_service<qyh_jaka_control_msgs::srv::StartRecording>(
            "/jaka/vr/start_recording",
            std::bind(&JakaServoNode::handleStartRecording, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stop_recording_ = create_service<qyh_jaka_control_msgs::srv::StopRecording>(
            "/jaka/vr/stop_recording",
            std::bind(&JakaServoNode::handleStopRecording, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_calibrate_vr_ = create_service<qyh_jaka_control_msgs::srv::CalibrateVR>(
            "/jaka/vr/calibrate",
            std::bind(&JakaServoNode::handleCalibrateVR, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_set_filter_ = create_service<qyh_jaka_control_msgs::srv::SetFilter>(
            "/jaka/servo/set_filter",
            std::bind(&JakaServoNode::handleSetFilter, this, std::placeholders::_1, std::placeholders::_2));

        // 连接到机器人
        if (auto_connect_) {
            if (jaka_interface_.connect(robot_ip_)) {
                RCLCPP_INFO(get_logger(), "Connected to robot at %s", robot_ip_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to connect to robot at %s", robot_ip_.c_str());
            }
        }

        // 主循环定时器
        auto period = std::chrono::microseconds(static_cast<int>(cycle_time_ms_ * 1000));
        main_timer_ = create_wall_timer(period, std::bind(&JakaServoNode::mainLoop, this));

        RCLCPP_INFO(get_logger(), "JAKA Servo Node initialized (cycle: %.2f ms)", cycle_time_ms_);
    }

    ~JakaServoNode()
    {
        if (recording_) {
            stopRecording();
        }
        if (servo_running_) {
            jaka_interface_.servoMoveEnable(false);
        }
    }

private:
    void mainLoop()
    {
        auto start_time = std::chrono::steady_clock::now();

        // 发布伺服状态
        publishServoStatus();

        // 如果伺服模式运行中
        if (servo_running_) {
            // VR跟随模式
            if (vr_following_) {
                processVRFollow();
            }
            // 否则处理普通伺服指令（由回调函数接收）
        }

        // 发布VR跟随状态
        if (vr_following_) {
            publishVRStatus();
        }

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        last_cycle_duration_us_ = duration.count();
    }

    void processVRFollow()
    {
        std::lock_guard<std::mutex> lock(vr_mutex_);

        if (!last_vr_left_ || !last_vr_right_) {
            return;  // 等待VR数据
        }

        // 检查VR数据是否过时
        auto now = this->now();
        if ((now - last_vr_left_->header.stamp).seconds() > 0.1 ||
            (now - last_vr_right_->header.stamp).seconds() > 0.1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "VR data is stale");
            return;
        }

        // 将VR位姿转换到机器人坐标系
        tf2::Transform vr_left_tf;
        tf2::fromMsg(last_vr_left_->pose, vr_left_tf);
        tf2::Transform robot_left_tf = vr_to_robot_left_ * vr_left_tf;

        tf2::Transform vr_right_tf;
        tf2::fromMsg(last_vr_right_->pose, vr_right_tf);
        tf2::Transform robot_right_tf = vr_to_robot_right_ * vr_right_tf;

        // 构建笛卡尔伺服指令
        geometry_msgs::msg::Pose left_target = tf2::toMsg(robot_left_tf);
        geometry_msgs::msg::Pose right_target = tf2::toMsg(robot_right_tf);

        // 发送到机器人（这里需要根据实际SDK调整）
        // 由于JAKA SDK的edg_servo_p针对单臂，需要分别发送
        CartesianPose left_pose = jaka_interface_.rosPoseToJaka(left_target);
        CartesianPose right_pose = jaka_interface_.rosPoseToJaka(right_target);

        // TODO: 根据SDK实际接口调整，可能需要使用edg_recv/edg_servo_p/edg_send循环
        // jaka_interface_.edgServoP(0, left_pose, true);  // 左臂
        // jaka_interface_.edgServoP(1, right_pose, true);  // 右臂

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
        double timestamp = now.seconds();

        // CSV格式：timestamp, lx, ly, lz, lqx, lqy, lqz, lqw, rx, ry, rz, rqx, rqy, rqz, rqw
        recording_file_ << std::fixed << std::setprecision(6)
                       << timestamp << ","
                       << left.position.x << "," << left.position.y << "," << left.position.z << ","
                       << left.orientation.x << "," << left.orientation.y << "," 
                       << left.orientation.z << "," << left.orientation.w << ","
                       << right.position.x << "," << right.position.y << "," << right.position.z << ","
                       << right.orientation.x << "," << right.orientation.y << "," 
                       << right.orientation.z << "," << right.orientation.w << "\n";

        recorded_frames_++;
    }

    void publishServoStatus()
    {
        auto msg = qyh_jaka_control_msgs::msg::JakaServoStatus();
        msg.mode = use_cartesian_ ? "cartesian" : "joint";
        msg.is_abs = true;
        msg.cycle_time_ns = static_cast<int32_t>(cycle_time_ms_ * 1e6);
        msg.publish_rate_hz = 1000.0 / cycle_time_ms_;
        msg.latency_ms = last_cycle_duration_us_ / 1000.0;
        msg.packet_loss_rate = 0.0;
        msg.error_code = 0;
        status_pub_->publish(msg);
    }

    void publishVRStatus()
    {
        auto msg = qyh_jaka_control_msgs::msg::VRFollowStatus();
        msg.header.stamp = this->now();
        msg.following_enabled = vr_following_;
        msg.recording_enabled = recording_;
        msg.left_arm_status = "following";
        msg.right_arm_status = "following";
        msg.left_pose_error = 0.0;  // TODO: 计算实际误差
        msg.right_pose_error = 0.0;
        msg.recorded_frames = recorded_frames_;
        msg.recording_duration = recording_start_time_.seconds() > 0 
            ? (this->now() - recording_start_time_).seconds() : 0.0;
        vr_status_pub_->publish(msg);
    }

    // 回调函数
    void jointCmdCallback(const qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr msg)
    {
        if (!servo_running_ || vr_following_) {
            return;  // VR跟随模式时忽略手动指令
        }
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_joint_cmd_ = msg;
        last_cmd_time_ = this->now();
        
        // 发送关节伺服指令
        jaka_interface_.servoJ(-1, msg->positions, msg->is_abs);
    }

    void cartesianCmdCallback(const qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr msg)
    {
        if (!servo_running_ || vr_following_) {
            return;
        }
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_cartesian_cmd_ = msg;
        last_cmd_time_ = this->now();

        // 发送笛卡尔伺服指令
        jaka_interface_.servoP(0, msg->left_pose, msg->is_abs);  // 左臂
        jaka_interface_.servoP(1, msg->right_pose, msg->is_abs);  // 右臂
    }

    void vrLeftCallback(const qyh_jaka_control_msgs::msg::VRPose::SharedPtr msg)
    {
        if (!vr_following_) return;
        std::lock_guard<std::mutex> lock(vr_mutex_);
        last_vr_left_ = msg;
    }

    void vrRightCallback(const qyh_jaka_control_msgs::msg::VRPose::SharedPtr msg)
    {
        if (!vr_following_) return;
        std::lock_guard<std::mutex> lock(vr_mutex_);
        last_vr_right_ = msg;
    }

    // 服务处理函数
    void handleStartServo(
        const qyh_jaka_control_msgs::srv::StartServo::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StartServo::Response::SharedPtr res)
    {
        if (servo_running_) {
            res->success = true;
            res->message = "Servo already running";
            return;
        }

        if (!jaka_interface_.isConnected()) {
            res->success = false;
            res->message = "Robot not connected";
            return;
        }

        if (jaka_interface_.servoMoveEnable(true)) {
            servo_running_ = true;
            res->success = true;
            res->message = "Servo mode started";
            RCLCPP_INFO(get_logger(), "Servo mode started");
        } else {
            res->success = false;
            res->message = "Failed to enable servo mode";
        }
    }

    void handleStopServo(
        const qyh_jaka_control_msgs::srv::StopServo::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StopServo::Response::SharedPtr res)
    {
        if (!servo_running_) {
            res->success = true;
            res->message = "Servo not running";
            return;
        }

        // 停止VR跟随和录制
        vr_following_ = false;
        if (recording_) {
            stopRecording();
        }

        if (jaka_interface_.servoMoveEnable(false)) {
            servo_running_ = false;
            res->success = true;
            res->message = "Servo mode stopped";
            RCLCPP_INFO(get_logger(), "Servo mode stopped");
        } else {
            res->success = false;
            res->message = "Failed to disable servo mode";
        }
    }

    void handleEnableVR(
        const qyh_jaka_control_msgs::srv::EnableVRFollow::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::EnableVRFollow::Response::SharedPtr res)
    {
        if (req->enable) {
            if (!servo_running_) {
                res->success = false;
                res->message = "Servo mode must be started first";
                return;
            }
            vr_following_ = true;
            res->success = true;
            res->message = "VR following enabled";
            RCLCPP_INFO(get_logger(), "VR following enabled");
        } else {
            vr_following_ = false;
            if (recording_) {
                stopRecording();
            }
            res->success = true;
            res->message = "VR following disabled";
            RCLCPP_INFO(get_logger(), "VR following disabled");
        }
    }

    void handleStartRecording(
        const qyh_jaka_control_msgs::srv::StartRecording::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::StartRecording::Response::SharedPtr res)
    {
        if (!vr_following_) {
            res->success = false;
            res->message = "VR following must be enabled first";
            return;
        }

        if (recording_) {
            res->success = false;
            res->message = "Recording already in progress";
            return;
        }

        // 生成文件名
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << recording_output_dir_ << "/recording_" 
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
        current_recording_file_ = ss.str();

        recording_file_.open(current_recording_file_);
        if (!recording_file_.is_open()) {
            res->success = false;
            res->message = "Failed to open recording file";
            return;
        }

        // 写CSV头
        recording_file_ << "timestamp,left_x,left_y,left_z,left_qx,left_qy,left_qz,left_qw,"
                       << "right_x,right_y,right_z,right_qx,right_qy,right_qz,right_qw\n";

        recording_ = true;
        recorded_frames_ = 0;
        recording_start_time_ = this->now();

        res->success = true;
        res->message = "Recording started: " + current_recording_file_;
        RCLCPP_INFO(get_logger(), "Recording started: %s", current_recording_file_.c_str());
    }

    void handleStopRecording(
        const qyh_jaka_control_msgs::srv::StopRecording::Request::SharedPtr,
        qyh_jaka_control_msgs::srv::StopRecording::Response::SharedPtr res)
    {
        if (!recording_) {
            res->success = false;
            res->message = "No recording in progress";
            return;
        }

        stopRecording();

        res->success = true;
        res->message = "Recording stopped";
        res->total_frames = recorded_frames_;
        res->duration = (this->now() - recording_start_time_).seconds();
        res->saved_file = current_recording_file_;

        RCLCPP_INFO(get_logger(), "Recording stopped: %d frames, %.2f sec, saved to %s",
                    recorded_frames_, res->duration, current_recording_file_.c_str());
    }

    void handleCalibrateVR(
        const qyh_jaka_control_msgs::srv::CalibrateVR::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::CalibrateVR::Response::SharedPtr res)
    {
        // TODO: 实现VR坐标系校准
        // 这里需要根据实际的VR设备和机器人布局来设计校准流程
        // 简单实现：直接使用提供的变换
        tf2::fromMsg(req->base_to_world, vr_to_robot_left_);
        vr_to_robot_right_ = vr_to_robot_left_;  // 假设两臂使用相同的变换

        res->success = true;
        res->message = "VR coordinate system calibrated";
        RCLCPP_INFO(get_logger(), "VR coordinate system calibrated");
    }

    void handleSetFilter(
        const qyh_jaka_control_msgs::srv::SetFilter::Request::SharedPtr req,
        qyh_jaka_control_msgs::srv::SetFilter::Response::SharedPtr res)
    {
        bool success = false;
        
        if (req->filter_type == "none") {
            success = jaka_interface_.setFilterNone();
        } else if (req->filter_type == "joint_lpf") {
            success = jaka_interface_.setFilterJointLPF(req->param1);
        } else if (req->filter_type == "joint_nlf") {
            success = jaka_interface_.setFilterJointNLF(req->param1, req->param2, req->param3);
        } else if (req->filter_type == "carte_nlf") {
            success = jaka_interface_.setFilterCarteNLF(
                req->param1, req->param2, req->param3, req->param4, req->param5, req->param6);
        } else {
            res->success = false;
            res->message = "Unknown filter type: " + req->filter_type;
            return;
        }

        res->success = success;
        res->message = success ? "Filter set successfully" : "Failed to set filter";
    }

    void stopRecording()
    {
        if (recording_file_.is_open()) {
            recording_file_.close();
        }
        recording_ = false;
    }

    // 成员变量
    qyh_jaka_control::JakaInterface jaka_interface_;
    
    // 参数
    std::string robot_ip_;
    double cycle_time_ms_;
    bool use_cartesian_;
    bool auto_connect_;
    std::string recording_output_dir_;

    // 状态
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

    // Publishers & Subscribers
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<qyh_jaka_control_msgs::msg::VRFollowStatus>::SharedPtr vr_status_pub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>::SharedPtr joint_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>::SharedPtr cartesian_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::VRPose>::SharedPtr vr_left_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::VRPose>::SharedPtr vr_right_sub_;

    // Services
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartServo>::SharedPtr srv_start_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopServo>::SharedPtr srv_stop_servo_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::EnableVRFollow>::SharedPtr srv_enable_vr_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StartRecording>::SharedPtr srv_start_recording_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::StopRecording>::SharedPtr srv_stop_recording_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::CalibrateVR>::SharedPtr srv_calibrate_vr_;
    rclcpp::Service<qyh_jaka_control_msgs::srv::SetFilter>::SharedPtr srv_set_filter_;

    // Timers
    rclcpp::TimerBase::SharedPtr main_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
