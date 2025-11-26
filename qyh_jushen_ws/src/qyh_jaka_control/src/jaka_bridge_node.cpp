#include "qyh_jaka_control/jaka_interface.hpp"
#include "qyh_jaka_control/smooth_servo_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief JAKA机器人桥接节点 - 用于遥操作控制
 * 
 * 订阅来自teleoperation_controller的平滑关节命令
 * 通过smooth_servo_bridge进一步缓冲和平滑
 * 以125Hz频率发送给真实JAKA机器人
 */
class JakaBridgeNode : public rclcpp::Node
{
public:
    JakaBridgeNode()
        : Node("jaka_bridge_node"),
          jaka_interface_(this->get_logger()),
          connected_(false),
          servo_enabled_(false)
    {
        // 参数
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("servo_frequency_hz", 125.0);
        declare_parameter<int>("buffer_size", 10);
        declare_parameter<double>("interpolation_weight", 0.5);
        declare_parameter<bool>("enable_interpolation", true);
        declare_parameter<bool>("auto_connect", true);
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        servo_frequency_ = get_parameter("servo_frequency_hz").as_double();
        buffer_size_ = static_cast<size_t>(get_parameter("buffer_size").as_int());
        
        // 创建平滑伺服桥接
        left_bridge_ = std::make_unique<qyh_jaka_control::SmoothServoBridge>(
            get_logger(), buffer_size_, servo_frequency_
        );
        right_bridge_ = std::make_unique<qyh_jaka_control::SmoothServoBridge>(
            get_logger(), buffer_size_, servo_frequency_
        );
        
        // 配置插值
        double interp_weight = get_parameter("interpolation_weight").as_double();
        bool enable_interp = get_parameter("enable_interpolation").as_bool();
        
        left_bridge_->setInterpolationWeight(interp_weight);
        left_bridge_->enableInterpolation(enable_interp);
        right_bridge_->setInterpolationWeight(interp_weight);
        right_bridge_->enableInterpolation(enable_interp);
        
        // 订阅关节命令 (来自teleoperation_controller)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        
        left_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm/joint_command", qos,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if (servo_enabled_ && msg->position.size() >= 7) {
                    std::vector<double> positions(msg->position.begin(), 
                                                 msg->position.begin() + 7);
                    left_bridge_->addCommand(positions);
                }
            }
        );
        
        right_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm/joint_command", qos,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                if (servo_enabled_ && msg->position.size() >= 7) {
                    std::vector<double> positions(msg->position.begin(),
                                                 msg->position.begin() + 7);
                    right_bridge_->addCommand(positions);
                }
            }
        );
        
        // 发布关节状态
        joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10
        );
        
        // 服务
        srv_start_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/start_servo",
            std::bind(&JakaBridgeNode::handleStartServo, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        srv_stop_servo_ = create_service<std_srvs::srv::Trigger>(
            "/jaka/bridge/stop_servo",
            std::bind(&JakaBridgeNode::handleStopServo, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 自动连接
        if (get_parameter("auto_connect").as_bool()) {
            if (jaka_interface_.connect(robot_ip_)) {
                connected_ = true;
                RCLCPP_INFO(get_logger(), "✓ Connected to robot at %s", robot_ip_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "✗ Failed to connect to robot");
            }
        }
        
        // 创建125Hz定时器（仅在伺服启动后激活）
        double period_ms = 1000.0 / servo_frequency_;
        servo_timer_ = create_wall_timer(
            std::chrono::duration<double, std::milli>(period_ms),
            std::bind(&JakaBridgeNode::servoCallback, this)
        );
        
        RCLCPP_INFO(get_logger(), "JAKA Bridge Node initialized");
        RCLCPP_INFO(get_logger(), "  Servo frequency: %.1f Hz", servo_frequency_);
        RCLCPP_INFO(get_logger(), "  Buffer size: %zu", buffer_size_);
    }
    
    ~JakaBridgeNode()
    {
        if (servo_enabled_) {
            stopServo();
        }
        if (connected_) {
            jaka_interface_.disconnect();
        }
    }

private:
    void servoCallback()
    {
        if (!servo_enabled_ || !connected_) {
            return;
        }
        
        std::vector<double> left_cmd, right_cmd;
        
        // 从桥接器获取插值命令
        bool left_ok = left_bridge_->getInterpolatedCommand(left_cmd);
        bool right_ok = right_bridge_->getInterpolatedCommand(right_cmd);
        
        if (!left_ok || !right_ok) {
            // 缓冲器为空，保持不动
            return;
        }
        
        // 发送到JAKA机器人
        // 注意：robot_id: 0=左臂, 1=右臂
        bool success = true;
        
        // 使用EtherCAT同步模式
        success &= jaka_interface_.edgServoJ(0, convertToJointValue(left_cmd), true);
        success &= jaka_interface_.edgServoJ(1, convertToJointValue(right_cmd), true);
        success &= jaka_interface_.edgSend();  // 同步发送
        
        if (!success) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Failed to send servo commands");
        }
        
        // 发布关节状态（用于监控）
        publishJointStates(left_cmd, right_cmd);
    }
    
    void handleStartServo(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (!connected_) {
            response->success = false;
            response->message = "Not connected to robot";
            return;
        }
        
        if (servo_enabled_) {
            response->success = true;
            response->message = "Servo already running";
            return;
        }
        
        if (startServo()) {
            response->success = true;
            response->message = "Servo started successfully";
        } else {
            response->success = false;
            response->message = "Failed to start servo";
        }
    }
    
    void handleStopServo(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        
        if (stopServo()) {
            response->success = true;
            response->message = "Servo stopped successfully";
        } else {
            response->success = false;
            response->message = "Failed to stop servo";
        }
    }
    
    bool startServo()
    {
        RCLCPP_INFO(get_logger(), "Starting servo mode...");
        
        // 启用两个机器人的伺服模式
        bool success = true;
        success &= jaka_interface_.servoMoveEnable(true, 0);  // 左臂
        success &= jaka_interface_.servoMoveEnable(true, 1);  // 右臂
        
        if (success) {
            servo_enabled_ = true;
            
            // 清空缓冲器
            left_bridge_->clearBuffer();
            right_bridge_->clearBuffer();
            left_bridge_->resetPerformanceStats();
            right_bridge_->resetPerformanceStats();
            
            RCLCPP_INFO(get_logger(), "✓ Servo mode started");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to start servo mode");
        }
        
        return success;
    }
    
    bool stopServo()
    {
        RCLCPP_INFO(get_logger(), "Stopping servo mode...");
        
        servo_enabled_ = false;
        
        // 禁用伺服模式
        bool success = true;
        success &= jaka_interface_.servoMoveEnable(false, 0);
        success &= jaka_interface_.servoMoveEnable(false, 1);
        
        // 打印性能统计
        auto left_stats = left_bridge_->getPerformanceStats();
        auto right_stats = right_bridge_->getPerformanceStats();
        
        RCLCPP_INFO(get_logger(), "Left arm stats:");
        RCLCPP_INFO(get_logger(), "  Frequency: %.1f Hz", left_stats.average_frequency);
        RCLCPP_INFO(get_logger(), "  Latency: %.2f ms", left_stats.average_latency_ms);
        RCLCPP_INFO(get_logger(), "  Errors: %zu", left_stats.error_count);
        RCLCPP_INFO(get_logger(), "  Buffer overflows: %zu", left_stats.buffer_overflow_count);
        
        RCLCPP_INFO(get_logger(), "Right arm stats:");
        RCLCPP_INFO(get_logger(), "  Frequency: %.1f Hz", right_stats.average_frequency);
        RCLCPP_INFO(get_logger(), "  Latency: %.2f ms", right_stats.average_latency_ms);
        RCLCPP_INFO(get_logger(), "  Errors: %zu", right_stats.error_count);
        RCLCPP_INFO(get_logger(), "  Buffer overflows: %zu", right_stats.buffer_overflow_count);
        
        if (success) {
            RCLCPP_INFO(get_logger(), "✓ Servo mode stopped");
        } else {
            RCLCPP_ERROR(get_logger(), "✗ Failed to stop servo mode");
        }
        
        return success;
    }
    
    JointValue convertToJointValue(const std::vector<double>& joints)
    {
        JointValue jv;
        for (size_t i = 0; i < std::min(joints.size(), size_t(7)); ++i) {
            jv.jVal[i] = joints[i];
        }
        return jv;
    }
    
    void publishJointStates(const std::vector<double>& left, const std::vector<double>& right)
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        
        // 左臂关节名
        for (int i = 1; i <= 7; ++i) {
            msg.name.push_back("left_joint_" + std::to_string(i));
        }
        // 右臂关节名
        for (int i = 1; i <= 7; ++i) {
            msg.name.push_back("right_joint_" + std::to_string(i));
        }
        
        // 位置
        msg.position.insert(msg.position.end(), left.begin(), left.end());
        msg.position.insert(msg.position.end(), right.begin(), right.end());
        
        joint_states_pub_->publish(msg);
    }
    
    // 成员变量
    qyh_jaka_control::JakaInterface jaka_interface_;
    std::unique_ptr<qyh_jaka_control::SmoothServoBridge> left_bridge_;
    std::unique_ptr<qyh_jaka_control::SmoothServoBridge> right_bridge_;
    
    std::string robot_ip_;
    double servo_frequency_;
    size_t buffer_size_;
    bool connected_;
    bool servo_enabled_;
    
    // ROS接口
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_cmd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_servo_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_servo_;
    
    rclcpp::TimerBase::SharedPtr servo_timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<JakaBridgeNode>();
    
    RCLCPP_INFO(node->get_logger(), "JAKA Bridge Node spinning...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
