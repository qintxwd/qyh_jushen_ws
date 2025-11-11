#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "jaka_control/jaka_robot_interface.hpp"

// Include generated service headers
#include "jaka_control/srv/enable_robot.hpp"
#include "jaka_control/srv/disable_robot.hpp"
#include "jaka_control/srv/power_on.hpp"
#include "jaka_control/srv/power_off.hpp"
#include "jaka_control/srv/move_j.hpp"
#include "jaka_control/srv/move_l.hpp"
#include "jaka_control/srv/set_collision_level.hpp"
#include "jaka_control/srv/set_tool_offset.hpp"
#include "jaka_control/srv/get_robot_state.hpp"
#include "jaka_control/srv/clear_error.hpp"
#include "jaka_control/srv/motion_abort.hpp"
#include "jaka_control/srv/servo_move_enable.hpp"
#include "jaka_control/srv/servo_j.hpp"
#include "jaka_control/srv/servo_p.hpp"
#include "jaka_control/srv/set_servo_filter.hpp"

// Include generated message headers
#include "jaka_control/msg/joint_position.hpp"
#include "jaka_control/msg/cartesian_pose.hpp"
#include "jaka_control/msg/robot_status.hpp"

using namespace std::chrono_literals;

namespace jaka_control
{

/**
 * @brief JAKA机器人ROS 2控制节点
 * 
 * 该节点提供了：
 * - 服务接口：用于控制机器人（移动、使能、配置等）
 * - 话题发布：定期发布机器人状态信息
 */
class JakaRobotNode : public rclcpp::Node
{
public:
    JakaRobotNode()
        : Node("jaka_robot_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("robot_ip", "192.168.2.200");
        this->declare_parameter<double>("status_publish_rate", 10.0);  // Hz
        this->declare_parameter<bool>("auto_connect", true);
        this->declare_parameter<bool>("auto_power_on", false);
        this->declare_parameter<bool>("auto_enable", false);

        // 获取参数
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        double publish_rate = this->get_parameter("status_publish_rate").as_double();
        bool auto_connect = this->get_parameter("auto_connect").as_bool();
        bool auto_power_on = this->get_parameter("auto_power_on").as_bool();
        bool auto_enable = this->get_parameter("auto_enable").as_bool();

        // 创建机器人接口
        robot_interface_ = std::make_shared<JakaRobotInterface>(this->get_logger());

        // 自动连接
        if (auto_connect) {
            if (robot_interface_->connect(robot_ip_)) {
                RCLCPP_INFO(this->get_logger(), "Robot connected successfully");
                
                // 自动上电
                if (auto_power_on) {
                    if (robot_interface_->powerOn()) {
                        RCLCPP_INFO(this->get_logger(), "Robot powered on");
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        
                        // 自动使能
                        if (auto_enable) {
                            if (robot_interface_->enableRobot()) {
                                RCLCPP_INFO(this->get_logger(), "Robot enabled");
                            } else {
                                RCLCPP_ERROR(this->get_logger(), "Failed to enable robot");
                            }
                        }
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to power on robot");
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot at %s", robot_ip_.c_str());
            }
        }

        // 创建服务
        createServices();

        // 创建发布器
        createPublishers();

        // 创建定时器用于发布状态
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        status_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JakaRobotNode::publishStatus, this));

        RCLCPP_INFO(this->get_logger(), "JAKA Robot Node initialized");
    }

    ~JakaRobotNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down JAKA Robot Node");
    }

private:
    void createServices()
    {
        // 基础控制服务
        srv_enable_ = this->create_service<jaka_control::srv::EnableRobot>(
            "~/enable_robot",
            std::bind(&JakaRobotNode::handleEnableRobot, this, std::placeholders::_1, std::placeholders::_2));

        srv_disable_ = this->create_service<jaka_control::srv::DisableRobot>(
            "~/disable_robot",
            std::bind(&JakaRobotNode::handleDisableRobot, this, std::placeholders::_1, std::placeholders::_2));

        srv_power_on_ = this->create_service<jaka_control::srv::PowerOn>(
            "~/power_on",
            std::bind(&JakaRobotNode::handlePowerOn, this, std::placeholders::_1, std::placeholders::_2));

        srv_power_off_ = this->create_service<jaka_control::srv::PowerOff>(
            "~/power_off",
            std::bind(&JakaRobotNode::handlePowerOff, this, std::placeholders::_1, std::placeholders::_2));

        srv_clear_error_ = this->create_service<jaka_control::srv::ClearError>(
            "~/clear_error",
            std::bind(&JakaRobotNode::handleClearError, this, std::placeholders::_1, std::placeholders::_2));

        srv_motion_abort_ = this->create_service<jaka_control::srv::MotionAbort>(
            "~/motion_abort",
            std::bind(&JakaRobotNode::handleMotionAbort, this, std::placeholders::_1, std::placeholders::_2));

        // 运动控制服务
        srv_move_j_ = this->create_service<jaka_control::srv::MoveJ>(
            "~/move_j",
            std::bind(&JakaRobotNode::handleMoveJ, this, std::placeholders::_1, std::placeholders::_2));

        srv_move_l_ = this->create_service<jaka_control::srv::MoveL>(
            "~/move_l",
            std::bind(&JakaRobotNode::handleMoveL, this, std::placeholders::_1, std::placeholders::_2));

        // 配置服务
        srv_set_collision_level_ = this->create_service<jaka_control::srv::SetCollisionLevel>(
            "~/set_collision_level",
            std::bind(&JakaRobotNode::handleSetCollisionLevel, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_tool_offset_ = this->create_service<jaka_control::srv::SetToolOffset>(
            "~/set_tool_offset",
            std::bind(&JakaRobotNode::handleSetToolOffset, this, std::placeholders::_1, std::placeholders::_2));

        // 状态查询服务
        srv_get_robot_state_ = this->create_service<jaka_control::srv::GetRobotState>(
            "~/get_robot_state",
            std::bind(&JakaRobotNode::handleGetRobotState, this, std::placeholders::_1, std::placeholders::_2));

        // 伺服控制服务
        srv_servo_enable_ = this->create_service<jaka_control::srv::ServoMoveEnable>(
            "~/servo_move_enable",
            std::bind(&JakaRobotNode::handleServoMoveEnable, this, std::placeholders::_1, std::placeholders::_2));

        srv_servo_j_ = this->create_service<jaka_control::srv::ServoJ>(
            "~/servo_j",
            std::bind(&JakaRobotNode::handleServoJ, this, std::placeholders::_1, std::placeholders::_2));

        srv_servo_p_ = this->create_service<jaka_control::srv::ServoP>(
            "~/servo_p",
            std::bind(&JakaRobotNode::handleServoP, this, std::placeholders::_1, std::placeholders::_2));

        srv_set_servo_filter_ = this->create_service<jaka_control::srv::SetServoFilter>(
            "~/set_servo_filter",
            std::bind(&JakaRobotNode::handleSetServoFilter, this, std::placeholders::_1, std::placeholders::_2));
    }

    void createPublishers()
    {
        // 机器人状态发布器
        pub_robot_status_ = this->create_publisher<jaka_control::msg::RobotStatus>(
            "~/robot_status", 10);

        // 关节状态发布器（兼容ROS标准）
        pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        // 左臂笛卡尔位姿发布器
        pub_left_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/left_arm/cartesian_pose", 10);

        // 右臂笛卡尔位姿发布器
        pub_right_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/right_arm/cartesian_pose", 10);
    }

    // 服务回调函数
    void handleEnableRobot(
        const std::shared_ptr<jaka_control::srv::EnableRobot::Request> request,
        std::shared_ptr<jaka_control::srv::EnableRobot::Response> response)
    {
        (void)request;
        response->success = robot_interface_->enableRobot();
        response->message = response->success ? "Robot enabled successfully" : "Failed to enable robot";
    }

    void handleDisableRobot(
        const std::shared_ptr<jaka_control::srv::DisableRobot::Request> request,
        std::shared_ptr<jaka_control::srv::DisableRobot::Response> response)
    {
        (void)request;
        response->success = robot_interface_->disableRobot();
        response->message = response->success ? "Robot disabled successfully" : "Failed to disable robot";
    }

    void handlePowerOn(
        const std::shared_ptr<jaka_control::srv::PowerOn::Request> request,
        std::shared_ptr<jaka_control::srv::PowerOn::Response> response)
    {
        (void)request;
        response->success = robot_interface_->powerOn();
        response->message = response->success ? "Robot powered on successfully" : "Failed to power on robot";
    }

    void handlePowerOff(
        const std::shared_ptr<jaka_control::srv::PowerOff::Request> request,
        std::shared_ptr<jaka_control::srv::PowerOff::Response> response)
    {
        (void)request;
        response->success = robot_interface_->powerOff();
        response->message = response->success ? "Robot powered off successfully" : "Failed to power off robot";
    }

    void handleClearError(
        const std::shared_ptr<jaka_control::srv::ClearError::Request> request,
        std::shared_ptr<jaka_control::srv::ClearError::Response> response)
    {
        (void)request;
        response->success = robot_interface_->clearError();
        response->message = response->success ? "Error cleared successfully" : "Failed to clear error";
    }

    void handleMotionAbort(
        const std::shared_ptr<jaka_control::srv::MotionAbort::Request> request,
        std::shared_ptr<jaka_control::srv::MotionAbort::Response> response)
    {
        (void)request;
        response->success = robot_interface_->motionAbort();
        response->message = response->success ? "Motion aborted successfully" : "Failed to abort motion";
    }

    void handleMoveJ(
        const std::shared_ptr<jaka_control::srv::MoveJ::Request> request,
        std::shared_ptr<jaka_control::srv::MoveJ::Response> response)
    {
        response->success = robot_interface_->moveJ(
            request->robot_id,
            request->joint_positions,
            request->move_mode,
            request->velocity,
            request->acceleration,
            request->is_block
        );
        response->message = response->success ? "MoveJ executed successfully" : "Failed to execute MoveJ";
    }

    void handleMoveL(
        const std::shared_ptr<jaka_control::srv::MoveL::Request> request,
        std::shared_ptr<jaka_control::srv::MoveL::Response> response)
    {
        response->success = robot_interface_->moveL(
            request->robot_id,
            request->target_pose,
            request->move_mode,
            request->velocity,
            request->acceleration,
            request->is_block
        );
        response->message = response->success ? "MoveL executed successfully" : "Failed to execute MoveL";
    }

    void handleSetCollisionLevel(
        const std::shared_ptr<jaka_control::srv::SetCollisionLevel::Request> request,
        std::shared_ptr<jaka_control::srv::SetCollisionLevel::Response> response)
    {
        response->success = robot_interface_->setCollisionLevel(request->robot_id, request->level);
        response->message = response->success ? 
            "Collision level set successfully" : "Failed to set collision level";
    }

    void handleSetToolOffset(
        const std::shared_ptr<jaka_control::srv::SetToolOffset::Request> request,
        std::shared_ptr<jaka_control::srv::SetToolOffset::Response> response)
    {
        response->success = robot_interface_->setToolOffset(request->robot_id, request->tool_offset);
        response->message = response->success ? "Tool offset set successfully" : "Failed to set tool offset";
    }

    void handleGetRobotState(
        const std::shared_ptr<jaka_control::srv::GetRobotState::Request> request,
        std::shared_ptr<jaka_control::srv::GetRobotState::Response> response)
    {
        (void)request;
        RobotState state;
        if (robot_interface_->getRobotState(state)) {
            response->powered_on = state.poweredOn;
            response->servo_enabled = state.servoEnabled;
            response->estoped = state.estoped;
            
            int error[2] = {0, 0};
            robot_interface_->isInError(error);
            response->in_error = (error[0] || error[1]);
            
            if (response->in_error) {
                ErrorCode error_code;
                if (robot_interface_->getLastError(error_code)) {
                    response->error_message = error_code.message;
                }
            }
        }
    }

    void handleServoMoveEnable(
        const std::shared_ptr<jaka_control::srv::ServoMoveEnable::Request> request,
        std::shared_ptr<jaka_control::srv::ServoMoveEnable::Response> response)
    {
        response->success = robot_interface_->servoMoveEnable(request->enable, request->robot_id);
        response->message = response->success ? 
            (request->enable ? "Servo mode enabled" : "Servo mode disabled") : 
            "Failed to set servo mode";
    }

    void handleServoJ(
        const std::shared_ptr<jaka_control::srv::ServoJ::Request> request,
        std::shared_ptr<jaka_control::srv::ServoJ::Response> response)
    {
        response->success = robot_interface_->servoJ(
            request->robot_id,
            request->joint_positions,
            request->move_mode
        );
        response->message = response->success ? "ServoJ executed successfully" : "Failed to execute ServoJ";
    }

    void handleServoP(
        const std::shared_ptr<jaka_control::srv::ServoP::Request> request,
        std::shared_ptr<jaka_control::srv::ServoP::Response> response)
    {
        response->success = robot_interface_->servoP(
            request->robot_id,
            request->target_pose,
            request->move_mode
        );
        response->message = response->success ? "ServoP executed successfully" : "Failed to execute ServoP";
    }

    void handleSetServoFilter(
        const std::shared_ptr<jaka_control::srv::SetServoFilter::Request> request,
        std::shared_ptr<jaka_control::srv::SetServoFilter::Response> response)
    {
        bool success = false;
        
        switch (request->filter_type) {
            case 0: // 不使用滤波器
                success = robot_interface_->servoMoveUseNoneFilter();
                break;
            case 1: // 关节LPF
                success = robot_interface_->servoMoveUseJointLPF(request->cutoff_freq);
                break;
            case 2: // 关节NLF
                if (request->nlf_params.size() >= 3) {
                    success = robot_interface_->servoMoveUseJointNLF(
                        request->nlf_params[3], 
                        request->nlf_params[4], 
                        request->nlf_params[5]
                    );
                }
                break;
            case 3: // 笛卡尔NLF
                if (request->nlf_params.size() >= 6) {
                    success = robot_interface_->servoMoveUseCarteNLF(
                        request->nlf_params[0], 
                        request->nlf_params[1], 
                        request->nlf_params[2],
                        request->nlf_params[3], 
                        request->nlf_params[4], 
                        request->nlf_params[5]
                    );
                }
                break;
            default:
                response->message = "Invalid filter type";
                response->success = false;
                return;
        }
        
        response->success = success;
        response->message = success ? "Filter set successfully" : "Failed to set filter";
    }

    void publishStatus()
    {
        // 获取机器人状态
        RobotState state;
        if (!robot_interface_->getRobotState(state)) {
            return;
        }

        // 发布RobotStatus消息
        auto status_msg = jaka_control::msg::RobotStatus();
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = "world";
        status_msg.powered_on = state.poweredOn;
        status_msg.servo_enabled = state.servoEnabled;
        status_msg.estoped = state.estoped;

        int error[2] = {0, 0};
        robot_interface_->isInError(error);
        status_msg.in_error = (error[0] || error[1]);

        int inpos[2] = {0, 0};
        robot_interface_->isInPosition(inpos);
        status_msg.left_in_position = inpos[0];
        status_msg.right_in_position = inpos[1];

        // TODO: 获取关节位置和笛卡尔位姿
        // 这需要从SDK获取当前位置信息

        pub_robot_status_->publish(status_msg);

        // 发布JointState消息（用于RViz显示）
        // TODO: 实现从SDK读取当前关节位置
    }

    // 成员变量
    std::string robot_ip_;
    std::shared_ptr<JakaRobotInterface> robot_interface_;

    // 服务服务器
    rclcpp::Service<jaka_control::srv::EnableRobot>::SharedPtr srv_enable_;
    rclcpp::Service<jaka_control::srv::DisableRobot>::SharedPtr srv_disable_;
    rclcpp::Service<jaka_control::srv::PowerOn>::SharedPtr srv_power_on_;
    rclcpp::Service<jaka_control::srv::PowerOff>::SharedPtr srv_power_off_;
    rclcpp::Service<jaka_control::srv::ClearError>::SharedPtr srv_clear_error_;
    rclcpp::Service<jaka_control::srv::MotionAbort>::SharedPtr srv_motion_abort_;
    rclcpp::Service<jaka_control::srv::MoveJ>::SharedPtr srv_move_j_;
    rclcpp::Service<jaka_control::srv::MoveL>::SharedPtr srv_move_l_;
    rclcpp::Service<jaka_control::srv::SetCollisionLevel>::SharedPtr srv_set_collision_level_;
    rclcpp::Service<jaka_control::srv::SetToolOffset>::SharedPtr srv_set_tool_offset_;
    rclcpp::Service<jaka_control::srv::GetRobotState>::SharedPtr srv_get_robot_state_;
    rclcpp::Service<jaka_control::srv::ServoMoveEnable>::SharedPtr srv_servo_enable_;
    rclcpp::Service<jaka_control::srv::ServoJ>::SharedPtr srv_servo_j_;
    rclcpp::Service<jaka_control::srv::ServoP>::SharedPtr srv_servo_p_;
    rclcpp::Service<jaka_control::srv::SetServoFilter>::SharedPtr srv_set_servo_filter_;

    // 发布器
    rclcpp::Publisher<jaka_control::msg::RobotStatus>::SharedPtr pub_robot_status_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_pose_;

    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace jaka_control

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jaka_control::JakaRobotNode>());
    rclcpp::shutdown();
    return 0;
}
