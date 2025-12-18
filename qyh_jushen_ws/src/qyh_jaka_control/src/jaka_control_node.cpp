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
        declare_parameter<bool>("ik_solver.enabled", true);
        declare_parameter<bool>("ik_solver.has_z_offset", true);
        declare_parameter<double>("ik_solver.left_z_offset", 0.219885132);
        declare_parameter<double>("ik_solver.right_z_offset", 0.217950931);
        
        ik_enabled_ = get_parameter("ik_solver.enabled").as_bool();
        has_z_offset_ = get_parameter("ik_solver.has_z_offset").as_bool();
        left_z_offset_ = get_parameter("ik_solver.left_z_offset").as_double();
        right_z_offset_ = get_parameter("ik_solver.right_z_offset").as_double();
        
        // 初始化速度控制器
        if (ik_enabled_) {
            RCLCPP_INFO(get_logger(), "🎯 速度积分控制模式已启用");
            
            std::string urdf_path = "d:/work/yc/qyh_jushen_ws/qyh_jushen_ws/src/qyh_dual_arms_description/urdf/dual_arms.urdf";
            
            left_vel_controller_ = std::make_unique<qyh_jaka_control::VelocityServoController>(shared_from_this(), "left");
            if (!left_vel_controller_->initialize(urdf_path, "base_link_left", "forward_lt")) {
                RCLCPP_ERROR(get_logger(), "Failed to initialize left velocity controller");
            }
            
            right_vel_controller_ = std::make_unique<qyh_jaka_control::VelocityServoController>(shared_from_this(), "right");
            if (!right_vel_controller_->initialize(urdf_path, "base_link_right", "forward_rt")) {
                RCLCPP_ERROR(get_logger(), "Failed to initialize right velocity controller");
            }
            
            RCLCPP_INFO(get_logger(), "✓ 速度伺服控制器已初始化");
            RCLCPP_INFO(get_logger(), "  has_z_offset=%s", has_z_offset_ ? "true" : "false");
            
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
            RCLCPP_WARN(get_logger(), "⚠️ IK求解模式未启用，节点将不工作");
        }

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
                bool success = true;
                
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
                
                // 统一发送，保证双臂同步
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] Calling edgSend()...");
                if (!jaka_interface_.edgSend(&cmd_index_)) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
                        "[MainLoop] Failed to send servo commands via edgSend");
                    return;  // 发送失败，停止本周期
                }
                cmd_index_++;
                RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "[MainLoop] edgSend() returned, cmd_index=%u", cmd_index_);
                
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
        
        try {
            geometry_msgs::msg::PoseStamped input_pose = *msg;
            input_pose.header.stamp = now();  // 使用当前时间，保证 TF 查找成功
            geometry_msgs::msg::PoseStamped target_in_base = tf_buffer_->transform(
                input_pose, "base_link_left", tf2::durationFromSec(0.1));
            
            if (has_z_offset_) {
                target_in_base.pose.position.z += left_z_offset_;
            }
            
            left_vel_controller_->setTargetPose(target_in_base);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Left TF Error: %s", ex.what());
        }
    }
    
    void rightVRTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!servo_running_ || !right_vel_controller_) return;
        
        try {
            geometry_msgs::msg::PoseStamped input_pose = *msg;
            input_pose.header.stamp = now();  // 使用当前时间，保证 TF 查找成功
            geometry_msgs::msg::PoseStamped target_in_base = tf_buffer_->transform(
                input_pose, "base_link_right", tf2::durationFromSec(0.1));
            
            if (has_z_offset_) {
                target_in_base.pose.position.z += right_z_offset_;
            }
            
            right_vel_controller_->setTargetPose(target_in_base);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Right TF Error: %s", ex.what());
        }
    }

    // ==================== 关节限位检查 ====================
    bool checkJointLimits(const JointValue& joints, const std::string& arm_name) {
        for (int i = 0; i < 7; ++i) {
            if (joints.jVal[i] < JAKA_ZU7_LIMITS[i].pos_min + SAFETY_MARGIN_POS ||
                joints.jVal[i] > JAKA_ZU7_LIMITS[i].pos_max - SAFETY_MARGIN_POS) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[Safety] %s 关节%d超出限位: %.3f (限位: %.3f ~ %.3f)",
                    arm_name.c_str(), i+1, joints.jVal[i],
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
                RCLCPP_INFO(get_logger(), "[Servo] Left joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
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
                RCLCPP_INFO(get_logger(), "[Servo] Right joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    right_joints[0], right_joints[1], right_joints[2], right_joints[3],
                    right_joints[4], right_joints[5], right_joints[6]);
                right_vel_controller_->updateRobotState(right_joints);
                right_vel_controller_->reset();
                right_init_success = true;
                RCLCPP_INFO(get_logger(), "[Servo] ✓ Right controller initialized from current position");
            } else {
                RCLCPP_WARN(get_logger(), "[Servo] ✗ Failed to initialize right controller");
            }
            
            // ★★★ 初始化完成后再允许主循环执行伺服 ★★★
            if (left_init_success || right_init_success) {
                servo_running_ = true;
                RCLCPP_INFO(get_logger(), "[Servo] Step 5/5: Controllers initialized, servo mode active");
                RCLCPP_INFO(get_logger(), "[Servo] === Servo Mode Active - Ready for commands ===");
            } else {
                RCLCPP_ERROR(get_logger(), "[Servo] Both controllers failed to initialize, rolling back...");
                jaka_interface_.servoMoveEnable(false, 0);
                jaka_interface_.servoMoveEnable(false, 1);
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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
