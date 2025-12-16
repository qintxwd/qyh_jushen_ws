/**
 * @file dual_arm_ik_solver_node.cpp
 * @brief 双臂IK求解节点 - VR遥操作专用
 * 
 * 功能：
 * - 作为第二个客户端连接到JAKA控制器
 * - 订阅VR目标位姿：/teleop/left_hand/target, /teleop/right_hand/target
 * - 高频调用IK求解（100Hz+）
 * - 发布关节指令供伺服控制使用
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <JAKAZuRobot.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <array>

using namespace std::chrono_literals;

class DualArmIKSolverNode : public rclcpp::Node
{
public:
    DualArmIKSolverNode() : Node("dual_arm_ik_solver_node")
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("ik_rate", 125.0);  // 125Hz匹配伺服频率
        declare_parameter<bool>("auto_connect", true);
        declare_parameter<bool>("use_tf_lookup", false);  // 是否使用TF查询
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        ik_rate_ = get_parameter("ik_rate").as_double();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        use_tf_lookup_ = get_parameter("use_tf_lookup").as_bool();
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "  双臂IK求解节点启动");
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "控制器IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_logger(), "IK求解频率: %.1f Hz", ik_rate_);
        RCLCPP_INFO(get_logger(), "使用TF查询: %s", use_tf_lookup_ ? "是" : "否");
        
        // 初始化JAKA SDK
        robot_ = std::make_unique<JAKAZuRobot>();
        
        // TF监听器（如果需要）
        if (use_tf_lookup_) {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        }
        
        // 订阅VR目标位姿
        left_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/left_hand/target", 10,
            std::bind(&DualArmIKSolverNode::leftTargetCallback, this, std::placeholders::_1));
        
        right_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/right_hand/target", 10,
            std::bind(&DualArmIKSolverNode::rightTargetCallback, this, std::placeholders::_1));
        
        // 发布关节指令
        left_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/left_arm/joint_command", 10);
        
        right_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/right_arm/joint_command", 10);
        
        // 发布IK状态
        ik_status_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/ik_solver/status", 10);
        
        // 自动连接
        if (auto_connect_) {
            connectToRobot();
        }
        
        // 定时器 - 高频IK求解
        auto period = std::chrono::duration<double>(1.0 / ik_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DualArmIKSolverNode::ikSolverCallback, this));
        
        RCLCPP_INFO(get_logger(), "✓ IK求解节点初始化完成");
        RCLCPP_INFO(get_logger(), "等待VR目标位姿输入...");
    }
    
    ~DualArmIKSolverNode()
    {
        if (connected_) {
            RCLCPP_INFO(get_logger(), "断开JAKA连接...");
            robot_->login_out();
        }
    }

private:
    void connectToRobot()
    {
        RCLCPP_INFO(get_logger(), "连接到JAKA控制器 %s (第二个客户端)...", robot_ip_.c_str());
        
        errno_t ret = robot_->login_in(robot_ip_.c_str());
        if (ret == ERR_SUCC) {
            connected_ = true;
            RCLCPP_INFO(get_logger(), "✅ IK求解节点成功连接！");
            RCLCPP_INFO(get_logger(), "📊 多客户端连接已验证");
            
            // 初始化参考关节位置（零位）
            initReferenceJoints();
        } else {
            connected_ = false;
            RCLCPP_ERROR(get_logger(), "❌ 连接失败！错误码: %d", ret);
            RCLCPP_ERROR(get_logger(), "请确认qyh_jaka_control已运行");
        }
    }
    
    void initReferenceJoints()
    {
        // 左右臂零位参考（全零或张开姿态）
        for (int i = 0; i < 7; i++) {
            ref_left_joints_.jVal[i] = 0.0;
            ref_right_joints_.jVal[i] = 0.0;
        }
        // 可选：使用张开姿态作为参考
        ref_left_joints_.jVal[1] = -1.0472;  // -60度
        ref_right_joints_.jVal[1] = -1.0472;
        
        RCLCPP_INFO(get_logger(), "✓ 参考关节位置初始化完成");
    }
    
    void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        left_target_ = msg;
        has_left_target_ = true;
    }
    
    void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        right_target_ = msg;
        has_right_target_ = true;
    }
    
    void ikSolverCallback()
    {
        if (!connected_) {
            return;
        }
        
        bool left_success = false;
        bool right_success = false;
        
        // 求解左臂IK
        if (has_left_target_) {
            left_success = solveLeftArmIK();
        }
        
        // 求解右臂IK
        if (has_right_target_) {
            right_success = solveRightArmIK();
        }
        
        // 发布状态
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = (left_success || right_success);
        ik_status_pub_->publish(status_msg);
        
        // 统计
        if (left_success) left_success_count_++;
        if (right_success) right_success_count_++;
        
        solve_count_++;
        
        // 每秒打印一次统计
        if (solve_count_ % static_cast<int>(ik_rate_) == 0) {
            double left_rate = 100.0 * left_success_count_ / solve_count_;
            double right_rate = 100.0 * right_success_count_ / solve_count_;
            RCLCPP_INFO(get_logger(), 
                "📊 IK统计: 左臂成功率=%.1f%%, 右臂成功率=%.1f%%, 总计=%d次",
                left_rate, right_rate, solve_count_);
        }
    }
    
    bool solveLeftArmIK()
    {
        // 转换目标位姿到JAKA格式
        CartesianPose target_pose;
        target_pose.tran.x = left_target_->pose.position.x * 1000.0;  // m -> mm
        target_pose.tran.y = left_target_->pose.position.y * 1000.0;
        target_pose.tran.z = left_target_->pose.position.z * 1000.0;
        
        // 四元数转欧拉角（RPY，弧度）
        tf2::Quaternion q(
            left_target_->pose.orientation.x,
            left_target_->pose.orientation.y,
            left_target_->pose.orientation.z,
            left_target_->pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 调用IK求解 - robot_id=0表示左臂
        JointValue ik_result;
        errno_t ret = robot_->kine_inverse(0, &ref_left_joints_, &target_pose, &ik_result);
        
        if (ret == ERR_SUCC) {
            // 更新参考位置（用于下次求解）
            ref_left_joints_ = ik_result;
            
            // 发布关节指令
            publishJointCommand(ik_result, left_joint_pub_);
            return true;
        } else {
            if (left_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "左臂IK失败 (错误计数: %d, 错误码: %d)", 
                    left_error_count_, ret);
            }
            return false;
        }
    }
    
    bool solveRightArmIK()
    {
        // 转换目标位姿到JAKA格式
        CartesianPose target_pose;
        target_pose.tran.x = right_target_->pose.position.x * 1000.0;  // m -> mm
        target_pose.tran.y = right_target_->pose.position.y * 1000.0;
        target_pose.tran.z = right_target_->pose.position.z * 1000.0;
        
        // 四元数转欧拉角
        tf2::Quaternion q(
            right_target_->pose.orientation.x,
            right_target_->pose.orientation.y,
            right_target_->pose.orientation.z,
            right_target_->pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 调用IK求解 - robot_id=1表示右臂
        JointValue ik_result;
        errno_t ret = robot_->kine_inverse(1, &ref_right_joints_, &target_pose, &ik_result);
        
        if (ret == ERR_SUCC) {
            // 更新参考位置
            ref_right_joints_ = ik_result;
            
            // 发布关节指令
            publishJointCommand(ik_result, right_joint_pub_);
            return true;
        } else {
            if (right_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "右臂IK失败 (错误计数: %d, 错误码: %d)", 
                    right_error_count_, ret);
            }
            return false;
        }
    }
    
    void publishJointCommand(const JointValue& joints, 
                            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
        msg.position = {
            joints.jVal[0], joints.jVal[1], joints.jVal[2],
            joints.jVal[3], joints.jVal[4], joints.jVal[5],
            joints.jVal[6]
        };
        pub->publish(msg);
    }

    // ROS相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ik_status_pub_;
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // JAKA SDK
    std::unique_ptr<JAKAZuRobot> robot_;
    std::string robot_ip_;
    bool connected_{false};
    bool auto_connect_{true};
    
    // 目标位姿
    geometry_msgs::msg::PoseStamped::SharedPtr left_target_;
    geometry_msgs::msg::PoseStamped::SharedPtr right_target_;
    bool has_left_target_{false};
    bool has_right_target_{false};
    
    // 参考关节位置
    JointValue ref_left_joints_;
    JointValue ref_right_joints_;
    
    // 配置
    double ik_rate_;
    bool use_tf_lookup_;
    
    // 统计
    int solve_count_{0};
    int left_success_count_{0};
    int right_success_count_{0};
    int left_error_count_{0};
    int right_error_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmIKSolverNode>();
    
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "🚀 双臂IK求解器运行中...");
    RCLCPP_INFO(node->get_logger(), "📌 作为第二个客户端连接到JAKA控制器");
    RCLCPP_INFO(node->get_logger(), " ");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
