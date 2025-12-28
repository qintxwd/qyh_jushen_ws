#include "jaka_control_node.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
using namespace std::chrono_literals;
namespace fs = std::filesystem;

// helper: degrees -> radians
static inline double deg2rad(double d) { return d * M_PI / 180.0; }
static inline double rad2deg(double d) { return d * 180.0 / M_PI; }

JakaControlNode::JakaControlNode() 
    : Node("jaka_control_node")
{
    // 参数声明
    declare_parameter<std::string>("robot_ip", "192.168.2.200");
    declare_parameter<double>("cycle_time", 0.008); // 默认 125Hz
    // declare_parameter<double>("jog_time", 0.1); // 默认100ms
    // 获取参数
    robot_ip_ = get_parameter("robot_ip").as_string();
    cycle_time_ = get_parameter("cycle_time").as_double();
    // jog_time_ = get_parameter("jog_time").as_double();

    for (int i = 0; i < 7; ++i) {
        left_joint_names_.push_back("left_joint" + std::to_string(i + 1));
        right_joint_names_.push_back("right_joint" + std::to_string(i + 1));
    }

    // Publishers
    robot_state_pub_ = create_publisher<qyh_jaka_control_msgs::msg::RobotState>("/jaka/robot_state", 10);

    left_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/left_arm/joint_states", 10);
    right_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/right_arm/joint_states", 10);

    left_tcp_pose_pub_ = create_publisher<sensor_msgs::msg::JointState>("/left_arm/tcp_pose", 10);
    right_tcp_pose_pub_ = create_publisher<sensor_msgs::msg::JointState>("/right_arm/tcp_pose", 10);

    left_torque_sensor_pub_ = create_publisher<sensor_msgs::msg::JointState>("/left_arm/torque_sensor", 10);
    right_torque_sensor_pub_ = create_publisher<sensor_msgs::msg::JointState>("/right_arm/torque_sensor", 10);

    dual_arm_command_p_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/dual_arm/command_servo_p", 10);

    // 订阅VR目标位姿
    left_vr_servo_p_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/teleop/left/servo_p", 10,
        std::bind(&JakaControlNode::leftServoPCallback, this, std::placeholders::_1));
    
    right_vr_servo_p_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/teleop/right/servo_p", 10,
        std::bind(&JakaControlNode::rightServoPCallback, this, std::placeholders::_1));

    // 订阅自定义合并后的servo p指令
    dual_arm_command_p_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
        "/dual_arm/command_servo_p", 10,
        std::bind(&JakaControlNode::command_p_callback, this, std::placeholders::_1));

    robot_ = std::make_shared<JAKAZuRobot>();
    
    // 创建服务处理器（负责所有服务回调）
    service_handlers_ = std::make_unique<qyh_jaka_control::JakaServiceHandlers>(
        this,
        robot_,
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


    has_left_cached_state_ = false;
    has_right_cached_state_ = false;

    // 按照官方示例程序的标准初始化流程
    RCLCPP_INFO(get_logger(), "Connecting to robot at %s...", robot_ip_.c_str());
    if (robot_->login_in(robot_ip_.c_str()) == ERR_SUCC) {
        connected_ = true;
        RCLCPP_INFO(get_logger(), "✓ Connected to robot");
        
        // 1. 清除错误
        RCLCPP_INFO(get_logger(), "Clearing errors...");
        robot_->clear_error();
        
        // 2. 关闭伺服模式（确保从干净状态开始）
        RCLCPP_INFO(get_logger(), "Disabling servo mode...");
        robot_->servo_move_enable(false, -1);
        
        // 3. 设置滤波器为低通
        RCLCPP_INFO(get_logger(), "Setting filter to low pass...");
        robot_->servo_move_use_joint_LPF(1.0); // 0=none, 0.5=low
        
        // 🔧 新增：在上电前设置负载
        RCLCPP_INFO(get_logger(), "Loading and setting payload configuration...");
        loadAndSetPayloadFromConfig();
        
        // 4. 上电
        RCLCPP_INFO(get_logger(), "Powering on...");
        if (robot_->power_on() == ERR_SUCC) {
            powered_ = true;
            RCLCPP_INFO(get_logger(), "✓ Powered on");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to power on");
        }
        
        // 5. 使能
        RCLCPP_INFO(get_logger(), "Enabling robot...");
        if (robot_->enable_robot() == ERR_SUCC) {
            enabled_ = true;
            RCLCPP_INFO(get_logger(), "✓ Robot enabled");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to enable robot");
        }
        
        // 6. 停止当前动作
        RCLCPP_INFO(get_logger(), "Aborting any motion...");
        robot_->motion_abort();
        
        // 7. 等待5秒让机器人稳定
        RCLCPP_INFO(get_logger(), "Waiting 5 seconds for robot to stabilize...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        RCLCPP_INFO(get_logger(), "✓ Robot initialization complete and ready");
    } else {
        RCLCPP_ERROR(get_logger(), "✗ Failed to connect to robot at %s", robot_ip_.c_str());
    }

    // Timers
    auto period = std::chrono::milliseconds(static_cast<int>(cycle_time_ * 1000));
    left_timer_ = create_wall_timer(period, std::bind(&JakaControlNode::left_timer_callback, this));
    right_timer_ = create_wall_timer(period, std::bind(&JakaControlNode::right_timer_callback, this));
    command_p_timer_ = create_wall_timer(period, std::bind(&JakaControlNode::command_p_timer_callback, this));
    status_timer_ = create_wall_timer(200ms, std::bind(&JakaControlNode::publishStatus, this));
    // jog_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(jog_time_ * 1000)),std::bind(&JakaControlNode::jog_timer_callback, this));
    
    RCLCPP_INFO(get_logger(), "=== JAKA Unified Control Node Initialized ===");
}

JakaControlNode::~JakaControlNode()
{
    if (servo_running_) {
        stopServoInternal();
    }
    if (connected_) {
        robot_->login_out();
    }
}

void JakaControlNode::left_timer_callback()
{
    sensor_msgs::msg::JointState msg_joint_state;
    sensor_msgs::msg::JointState msg_tcp_pose;
    sensor_msgs::msg::JointState msg_raw_torque;

    auto current_time = this->get_clock()->now();
    msg_joint_state.header.stamp = current_time;
    msg_tcp_pose.header.stamp = current_time;
    msg_raw_torque.header.stamp = current_time;

    msg_joint_state.name.assign(left_joint_names_.begin(), left_joint_names_.end());
    msg_raw_torque.name.assign(left_joint_names_.begin(), left_joint_names_.end());

    // be careful! cartesian read rpy in degrees, but requires radius to control servo p
    // edg_get_stat 单位mm和rad
    if (robot_->edg_get_stat(0, &cached_left_joints_, &cached_left_pose_) == ERR_SUCC) // 0 for left arm
    {
        cached_left_pose_.rpy.rx = deg2rad(cached_left_pose_.rpy.rx);
        cached_left_pose_.rpy.ry = deg2rad(cached_left_pose_.rpy.ry);
        cached_left_pose_.rpy.rz = deg2rad(cached_left_pose_.rpy.rz);
        // joint vals
        msg_joint_state.position.assign(cached_left_joints_.jVal, cached_left_joints_.jVal + 7);
        tf2::Quaternion quaternion;
        quaternion.setRPY(cached_left_pose_.rpy.rx, cached_left_pose_.rpy.ry, cached_left_pose_.rpy.rz);

        msg_tcp_pose.position.resize(10);
        msg_tcp_pose.position[0] = cached_left_pose_.tran.x;
        msg_tcp_pose.position[1] = cached_left_pose_.tran.y;
        msg_tcp_pose.position[2] = cached_left_pose_.tran.z;
        msg_tcp_pose.position[3] = quaternion.w();
        msg_tcp_pose.position[4] = quaternion.x();
        msg_tcp_pose.position[5] = quaternion.y();
        msg_tcp_pose.position[6] = quaternion.z();
        // 新增：附加 rpy 信息（rad）
        msg_tcp_pose.position[7] = cached_left_pose_.rpy.rx;
        msg_tcp_pose.position[8] = cached_left_pose_.rpy.ry;
        msg_tcp_pose.position[9] = cached_left_pose_.rpy.rz;
    }
    

    if (msg_pre_left_joint_state.size() != 0)
    {
        for (int i = 0; i < 7; i++)
        {
            msg_joint_state.velocity.push_back((msg_joint_state.position[i] - msg_pre_left_joint_state[i]) / cycle_time_);
        }
    }else{
        for (int i = 0; i < 7; i++)
        {
            msg_joint_state.velocity.push_back(0.0);
            msg_pre_left_joint_state.push_back(0.0);
        }
    }

    for (int i = 0; i < 7; i++)
    {
        msg_pre_left_joint_state[i] = msg_joint_state.position[i];
    }

    left_joint_state_pub_->publish(msg_joint_state);
    left_tcp_pose_pub_->publish(msg_tcp_pose);
    // 每隔1秒输出一次msg_tcp_pose内容，方便调试
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Left TCP Pose: [x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f]",
        msg_tcp_pose.position[0], msg_tcp_pose.position[1], msg_tcp_pose.position[2],
        rad2deg(cached_left_pose_.rpy.rx), rad2deg(cached_left_pose_.rpy.ry), rad2deg(cached_left_pose_.rpy.rz));

    int status[2] = {-3, -3};
    int errorcode[2] = {-1, -1};
    double ft_original[6] = {};
    double ft_actual[6] = {};
    int ret = robot_->robot_get_ftsensor_stat(LEFT, 1, status, errorcode, ft_original, ft_actual);
    if (ret != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "robot_get_ftsensor_stat failed!");
    }
    else
    {
        msg_raw_torque.position.assign(ft_original, ft_original + 6);
        left_torque_sensor_pub_->publish(msg_raw_torque);
    }
}
void JakaControlNode::right_timer_callback()
{
    sensor_msgs::msg::JointState msg_joint_state;
    sensor_msgs::msg::JointState msg_tcp_pose;
    sensor_msgs::msg::JointState msg_raw_torque;

    auto current_time = this->get_clock()->now();
    msg_joint_state.header.stamp = current_time;
    msg_tcp_pose.header.stamp = current_time;
    msg_raw_torque.header.stamp = current_time;

    msg_joint_state.name.assign(right_joint_names_.begin(), right_joint_names_.end());
    msg_raw_torque.name.assign(right_joint_names_.begin(), right_joint_names_.end());

    // be careful! cartesian read rpy in degrees, but requires radius to control servo p
    if (robot_->edg_get_stat(1, &cached_right_joints_, &cached_right_pose_) == ERR_SUCC) // 1 for right arm
    {
        cached_right_pose_.rpy.rx = deg2rad(cached_right_pose_.rpy.rx);
        cached_right_pose_.rpy.ry = deg2rad(cached_right_pose_.rpy.ry);
        cached_right_pose_.rpy.rz = deg2rad(cached_right_pose_.rpy.rz);
        // joint vals
        msg_joint_state.position.assign(cached_right_joints_.jVal, cached_right_joints_.jVal + 7);
        tf2::Quaternion quaternion;
        quaternion.setRPY(cached_right_pose_.rpy.rx, cached_right_pose_.rpy.ry, cached_right_pose_.rpy.rz);

        msg_tcp_pose.position.resize(10);
        msg_tcp_pose.position[0] = cached_right_pose_.tran.x;
        msg_tcp_pose.position[1] = cached_right_pose_.tran.y;
        msg_tcp_pose.position[2] = cached_right_pose_.tran.z;
        msg_tcp_pose.position[3] = quaternion.w();
        msg_tcp_pose.position[4] = quaternion.x();
        msg_tcp_pose.position[5] = quaternion.y();
        msg_tcp_pose.position[6] = quaternion.z();
        
        // 新增：附加 rpy 信息（rad）
        msg_tcp_pose.position[7] = cached_right_pose_.rpy.rx;
        msg_tcp_pose.position[8] = cached_right_pose_.rpy.ry;
        msg_tcp_pose.position[9] = cached_right_pose_.rpy.rz;
    }

    if (msg_pre_right_joint_state.size() != 0)
    {
        for (int i = 0; i < 7; i++)
        {
            msg_joint_state.velocity.push_back((msg_joint_state.position[i] - msg_pre_right_joint_state[i]) / cycle_time_);
        }
    }else{
        for (int i = 0; i < 7; i++)
        {
            msg_joint_state.velocity.push_back(0.0);
            msg_pre_right_joint_state.push_back(0.0);
        }
    }

    for (int i = 0; i < 7; i++)
    {
        msg_pre_right_joint_state[i] = msg_joint_state.position[i];
    }

    right_joint_state_pub_->publish(msg_joint_state);
    right_tcp_pose_pub_->publish(msg_tcp_pose);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
    //     "Right TCP Pose: [x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f]",
    //     msg_tcp_pose.position[0], msg_tcp_pose.position[1], msg_tcp_pose.position[2],
    //     rad2deg(cached_right_pose_.rpy.rx), rad2deg(cached_right_pose_.rpy.ry), rad2deg(cached_right_pose_.rpy.rz));

    int status[2] = {-3, -3};
    int errorcode[2] = {-1, -1};
    double ft_original[6] = {};
    double ft_actual[6] = {};
    int ret = robot_->robot_get_ftsensor_stat(RIGHT, 1, status, errorcode, ft_original, ft_actual);
    if (ret != 0)
    {
        printf("robot_get_ftsensor_stat failed!\n");
    }
    else
    {
        msg_raw_torque.position.assign(ft_original, ft_original + 6);
        right_torque_sensor_pub_->publish(msg_raw_torque);
    }
}

void JakaControlNode::command_p_timer_callback()
{
    if (!connected_ || !powered_ || !enabled_ || !servo_running_) {
        return;
    }
    auto current_time = this->get_clock()->now();

    //状态评估
    if (left_input_state_ != ServoInputState::NEVER_RECEIVED) {
        rclcpp::Duration left_age = current_time - left_last_command_p_time_;

        if (left_input_state_ == ServoInputState::ACTIVE) {
            if (left_age > command_timeout_) {
                left_input_state_ = ServoInputState::TIMEOUT;
            } else if (left_age > active_window_) {
                left_input_state_ = ServoInputState::HOLD;
            }
        }
        else if (left_input_state_ == ServoInputState::HOLD) {
            if (left_age > command_timeout_) {
                left_input_state_ = ServoInputState::TIMEOUT;
            }
        }
    }

    if (right_input_state_ != ServoInputState::NEVER_RECEIVED) {
        rclcpp::Duration right_age = current_time - right_last_command_p_time_;

        if (right_input_state_ == ServoInputState::ACTIVE) {
            if (right_age > command_timeout_) {
                right_input_state_ = ServoInputState::TIMEOUT;
            } else if (right_age > active_window_) {
                right_input_state_ = ServoInputState::HOLD;
            }
        }
        else if (right_input_state_ == ServoInputState::HOLD) {
            if (right_age > command_timeout_) {
                right_input_state_ = ServoInputState::TIMEOUT;
            }
        }
    }

    // 3. 目标位姿决策（NEVER_RECEIVED/ACTIVE/HOLD/TIMEOUT）
    CartesianPose target_left;
    CartesianPose target_right;

    // LEFT
    switch (left_input_state_) {
        case ServoInputState::NEVER_RECEIVED:
            target_left = cached_left_pose_; // 保持当前
            break;
        case ServoInputState::ACTIVE:
            target_left = left_command_servo_p_val;
            left_last_target_pose_ = target_left;
            break;
        case ServoInputState::HOLD:
            target_left = left_last_target_pose_;
            break;
        case ServoInputState::TIMEOUT:
            target_left = left_last_target_pose_;  // 保持最后一次目标
            break;
    }

    // RIGHT
    switch (right_input_state_) {
        case ServoInputState::NEVER_RECEIVED:
            target_right = cached_right_pose_;
            break;
        case ServoInputState::ACTIVE:
            target_right = right_command_servo_p_val;
            right_last_target_pose_ = target_right;
            break;
        case ServoInputState::HOLD:
            target_right = right_last_target_pose_;
            break;
        case ServoInputState::TIMEOUT:
            target_right = right_last_target_pose_; // 保持最后一次目标
            break;
    }

    // 4. 按照职责分工：Timer -> 生成合并的12维命令并发布，实际发送由订阅者 `command_p_callback` 负责执行
    sensor_msgs::msg::JointState dual_msg;
    dual_msg.header.stamp = current_time;
    dual_msg.position.resize(12);
    dual_msg.position[0] = target_left.tran.x;
    dual_msg.position[1] = target_left.tran.y;
    dual_msg.position[2] = target_left.tran.z;
    dual_msg.position[3] = target_left.rpy.rx;
    dual_msg.position[4] = target_left.rpy.ry;
    dual_msg.position[5] = target_left.rpy.rz;
    dual_msg.position[6] = target_right.tran.x;
    dual_msg.position[7] = target_right.tran.y;
    dual_msg.position[8] = target_right.tran.z;
    dual_msg.position[9] = target_right.rpy.rx;
    dual_msg.position[10] = target_right.rpy.ry;
    dual_msg.position[11] = target_right.rpy.rz;

    // 输出合并命令，便于排查数据流（200ms节流）
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "[CommandTimer] publishing dual command: [lx=%.3f,ly=%.3f,lz=%.3f,lr=%.3f,lp=%.3f,lyw=%.3f, rx=%.3f,ry=%.3f,rz=%.3f,rr=%.3f,rp=%.3f,ryw=%.3f]",
        dual_msg.position[0], dual_msg.position[1], dual_msg.position[2], dual_msg.position[3], dual_msg.position[4], dual_msg.position[5],
        dual_msg.position[6], dual_msg.position[7], dual_msg.position[8], dual_msg.position[9], dual_msg.position[10], dual_msg.position[11]);

    dual_arm_command_p_publisher_->publish(dual_msg);
}

// 接收合并后的12维消息并原子地发送到机器人
void JakaControlNode::command_p_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg || !connected_ || !powered_ || !enabled_ || !servo_running_) {
        return;
    }

    // 检查长度
    if (msg->position.size() != 12) {
        RCLCPP_ERROR(this->get_logger(), "command_p_callback: expected 12 positions, got %zu", msg->position.size());
        return;
    }

    try {
        CartesianPose pose_left;
        pose_left.tran.x = msg->position[0];
        pose_left.tran.y = msg->position[1];
        pose_left.tran.z = msg->position[2];
        pose_left.rpy.rx = msg->position[3];
        pose_left.rpy.ry = msg->position[4];
        pose_left.rpy.rz = msg->position[5];

        CartesianPose pose_right;
        pose_right.tran.x = msg->position[6];
        pose_right.tran.y = msg->position[7];
        pose_right.tran.z = msg->position[8];
        pose_right.rpy.rx = msg->position[9];
        pose_right.rpy.ry = msg->position[10];
        pose_right.rpy.rz = msg->position[11];

        // 原子发送：先将两个edg_servo_p压入缓冲，然后一次edg_send
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
            "[CommandCallback] sending to robot: left rpy=[%.6f, %.6f, %.6f], right rpy=[%.6f, %.6f, %.6f]",
            pose_left.rpy.rx, pose_left.rpy.ry, pose_left.rpy.rz,
            pose_right.rpy.rx, pose_right.rpy.ry, pose_right.rpy.rz);

        robot_->edg_servo_p(0, &pose_left, ABS, 1);
        robot_->edg_servo_p(1, &pose_right, ABS, 1);
        int ret = robot_->edg_send();
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "edg_send failed with code %d", ret);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "edg_send OK");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in command_p_callback: %s", e.what());
    }
}


void JakaControlNode::publishStatus()
{
    auto now = this->now();
    
    // ==================== 2. 机器人状态 ====================
    auto robot_state_msg = qyh_jaka_control_msgs::msg::RobotState();
    robot_state_msg.header.stamp = now;
    robot_state_msg.header.frame_id = "base_link";
    robot_state_msg.connected = connected_;
    robot_state_msg.robot_ip = robot_ip_;
    robot_state_msg.servo_mode_enabled = servo_running_;
    
    // 🔧 实时查询机器人状态（而不是使用缓存的powered_/enabled_变量）
    if (connected_) {
        RobotState state;
        if (robot_->get_robot_state(&state) == ERR_SUCC) {
            robot_state_msg.powered_on = state.poweredOn;
            robot_state_msg.enabled = state.servoEnabled;
            robot_state_msg.in_estop = state.estoped;
            
            // 同步更新缓存变量（供服务回调使用）
            powered_ = state.poweredOn;
            enabled_ = state.servoEnabled;
        } else {
            // 查询失败时使用缓存值
            robot_state_msg.powered_on = powered_;
            robot_state_msg.enabled = enabled_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[Status] Failed to query robot state, using cached values");
        }
    } else {
        robot_state_msg.powered_on = false;
        robot_state_msg.enabled = false;
    }
    
    // ⚡ 始终使用缓存数据（mainLoop现在总是更新缓存）
    if (has_left_cached_state_) {        
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
    }
    if (has_right_cached_state_) {  
        // 关节位置  
        for (size_t i = 0; i < 7; ++i) {
            robot_state_msg.right_joint_positions[i] = cached_right_joints_.jVal[i];
        }   
         
        // TCP位姿
        tf2::Quaternion q_right;
        q_right.setRPY(cached_right_pose_.rpy.rx, cached_right_pose_.rpy.ry, cached_right_pose_.rpy.rz);
        robot_state_msg.right_cartesian_pose.orientation = tf2::toMsg(q_right);
        
        robot_state_msg.right_cartesian_pose.position.x = cached_right_pose_.tran.x;
        robot_state_msg.right_cartesian_pose.position.y = cached_right_pose_.tran.y;
        robot_state_msg.right_cartesian_pose.position.z = cached_right_pose_.tran.z;
    }
    // 检查错误状态
    int error[2] = {0, 0};
    if (robot_->robot_is_in_error(error)) {
        robot_state_msg.in_error = (error[0] || error[1]);
        if (robot_state_msg.in_error) {
            ErrorCode error_code;
            if (robot_->get_last_error(&error_code) == ERR_SUCC) {
                robot_state_msg.error_message = error_code.message;
                if (error_code.code != 0) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                        "[Status] Robot in error state: Code %ld, Message: %s",
                        error_code.code, error_code.message);
                }
            }
        }
    }
    
    robot_state_pub_->publish(robot_state_msg);
}

void JakaControlNode::leftServoPCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(!connected_ || !powered_ || !enabled_ || !servo_running_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[LeftServoP] Robot not ready to receive commands");
        return;
    }
    if (msg->position.size() != 7) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[LeftServoP] invalid position size: %zu", msg->position.size());
        return;
    }

    left_command_servo_p_val.tran.x = msg->position[0];
    left_command_servo_p_val.tran.y = msg->position[1];
    left_command_servo_p_val.tran.z = msg->position[2];
    left_command_servo_p_val.rpy.rx = new_rpy_left[3];
    left_command_servo_p_val.rpy.ry = new_rpy_left[4];
    left_command_servo_p_val.rpy.rz = new_rpy_left[5];
    
    left_last_command_p_time_ = this->now();
    left_input_state_ = ServoInputState::ACTIVE;
    left_last_target_pose_ = left_command_servo_p_val;  // 同步一次

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "[LeftServoP] recv pos: x=%.3f y=%.3f z=%.3f rpy=[%.6f, %.6f, %.6f]",
        left_command_servo_p_val.tran.x, left_command_servo_p_val.tran.y, left_command_servo_p_val.tran.z,
        left_command_servo_p_val.rpy.rx, left_command_servo_p_val.rpy.ry, left_command_servo_p_val.rpy.rz);
}
    
void JakaControlNode::rightServoPCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if(!connected_ || !powered_ || !enabled_ || !servo_running_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[LeftServoP] Robot not ready to receive commands");
        return;
    }
    if (msg->position.size() != 7) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[RightServoP] invalid position size: %zu", msg->position.size());
        return;
    }
    right_command_servo_p_val.tran.x = msg->position[0];
    right_command_servo_p_val.tran.y = msg->position[1];
    right_command_servo_p_val.tran.z = msg->position[2];
    right_command_servo_p_val.rpy.rx = new_rpy_right[3];
    right_command_servo_p_val.rpy.ry = new_rpy_right[4];
    right_command_servo_p_val.rpy.rz = new_rpy_right[5];

    right_last_command_p_time_ = this->now();
    right_input_state_ = ServoInputState::ACTIVE;
    right_last_target_pose_ = right_command_servo_p_val;  // 同步一次

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "[RightServoP] recv pos: x=%.3f y=%.3f z=%.3f rpy=[%.6f, %.6f, %.6f]",
        right_command_servo_p_val.tran.x, right_command_servo_p_val.tran.y, right_command_servo_p_val.tran.z,
        right_command_servo_p_val.rpy.rx, right_command_servo_p_val.rpy.ry, right_command_servo_p_val.rpy.rz);
}


// ==================== 负载配置加载 ====================
/**
 * @brief 从YAML配置文件加载并设置夹爪负载
 * @return true if successful
 */
bool JakaControlNode::loadAndSetPayloadFromConfig() {
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

        PayLoad left_payload;
        left_payload.mass = left_mass;
        left_payload.centroid.x = 150.0;
        left_payload.centroid.y = 0.0;
        left_payload.centroid.z = 0.0;
        errno_t lret = robot_->robot_set_tool_payload(0, &left_payload);
        bool left_success = (lret == ERR_SUCC);
        if (left_success) {
            RCLCPP_INFO(get_logger(), "  ✓ Left arm payload set: %.2f kg", left_mass);
        } else {
            RCLCPP_ERROR(get_logger(), "  ✗ Failed to set left arm payload (err=%d)", lret);
        }

        PayLoad right_payload;
        right_payload.mass = right_mass;
        right_payload.centroid.x = 150.0;
        right_payload.centroid.y = 0.0;
        right_payload.centroid.z = 0.0;
        errno_t rret = robot_->robot_set_tool_payload(1, &right_payload);
        bool right_success = (rret == ERR_SUCC);
        if (right_success) {
            RCLCPP_INFO(get_logger(), "  ✓ Right arm payload set: %.2f kg", right_mass);
        } else {
            RCLCPP_ERROR(get_logger(), "  ✗ Failed to set right arm payload (err=%d)", rret);
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

// ==================== 内部辅助函数（供 JakaServiceHandlers 调用）====================
bool JakaControlNode::startServoInternal() {
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
    errno_t ret = true;
    robot_->servo_move_use_joint_LPF(1.0); // 0=none, 0.5=low
    ret = robot_->servo_move_enable(true, 0);
    if (ret != ERR_SUCC) {
        //获取错误码并打印
        RCLCPP_ERROR(get_logger(), "[Servo] Error enabling left arm servo (err=%d)", ret);
        RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable left arm servo!");
        return false;
    }
    RCLCPP_INFO(get_logger(), "[Servo] Left arm servo enabled successfully");
    
    RCLCPP_INFO(get_logger(), "[Servo] Step 3/5: Enabling servo for right arm (id=1)...");
    ret = robot_->servo_move_enable(true, 1);
    if (ret != ERR_SUCC) {
        RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable right arm servo!");
        robot_->servo_move_enable(false, 0);  // 回滚左臂
        return false;
    }
    RCLCPP_INFO(get_logger(), "[Servo] Right arm servo enabled successfully");
    
    if (ret == ERR_SUCC) {

        RCLCPP_INFO(get_logger(), "[Servo] Step 4/5: Initializing controllers from current positions...");
        
        left_command_servo_p_val = cached_left_pose_;
        right_command_servo_p_val = cached_right_pose_;

        left_input_state_ = ServoInputState::NEVER_RECEIVED;
        right_input_state_ = ServoInputState::NEVER_RECEIVED;

        // 初始化 last_target 为当前缓存位姿，便于进入 HOLD 时保持
        left_last_target_pose_ = cached_left_pose_;
        right_last_target_pose_ = cached_right_pose_;

        servo_running_ = true;

        return true;
    }
    // 如果失败，尝试回滚
    RCLCPP_ERROR(get_logger(), "[Servo] Failed to enable servo, rolling back...");
    robot_->servo_move_enable(false, 0);
    robot_->servo_move_enable(false, 1);
    RCLCPP_ERROR(get_logger(), "[Servo] Servo start failed!");
    return false;
}

bool JakaControlNode::stopServoInternal() {
    RCLCPP_INFO(get_logger(), "[Servo] === Stopping Servo Mode ===");
    servo_running_ = false;
    
    RCLCPP_INFO(get_logger(), "[Servo] Disabling left arm servo...");
    errno_t ret = true;
    ret = robot_->servo_move_enable(false, 0);
    RCLCPP_INFO(get_logger(), "[Servo] Disabling right arm servo...");
    ret = robot_->servo_move_enable(false, 1);
    
    if (ret == ERR_SUCC) {
        RCLCPP_INFO(get_logger(), "[Servo] === Servo Mode Stopped ===");
    } else {
        RCLCPP_ERROR(get_logger(), "[Servo] Failed to stop servo cleanly");
    }
    return ret == ERR_SUCC;
}
    

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaControlNode>();
    // initVelocityControllers removed (not present in this class)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
