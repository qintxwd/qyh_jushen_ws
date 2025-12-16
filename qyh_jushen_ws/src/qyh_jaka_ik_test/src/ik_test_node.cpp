/**
 * @file ik_test_node.cpp
 * @brief 临时测试节点 - 验证JAKA SDK多客户端连接
 * 
 * 功能：
 * - 连接到与qyh_jaka_control相同的控制器
 * - 高频调用IK求解（100Hz）
 * - 测试是否会与主控制节点冲突
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <JAKAZuRobot.h>
#include <chrono>
#include <cmath>
#include <memory>

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

using namespace std::chrono_literals;

class JakaIKTestNode : public rclcpp::Node
{
public:
    JakaIKTestNode() : Node("jaka_ik_test_node")
    {
        // 参数
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("ik_test_rate", 100.0);  // 100Hz
        declare_parameter<bool>("auto_connect", true);
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        double rate = get_parameter("ik_test_rate").as_double();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        
        // 初始化JAKA SDK
        robot_ = std::make_unique<JAKAZuRobot>();
        
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "  JAKA IK测试节点启动");
        RCLCPP_INFO(get_logger(), "========================================");
        RCLCPP_INFO(get_logger(), "目标IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_logger(), "IK测试频率: %.1f Hz", rate);
        
        // 发布器
        ik_result_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "ik_test/result", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "ik_test/status", 10);
        
        // 自动连接
        if (auto_connect_) {
            connectToRobot();
        }
        
        // 定时器 - 高频IK求解测试
        auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&JakaIKTestNode::ikTestCallback, this));
        
        RCLCPP_INFO(get_logger(), "✓ IK测试节点初始化完成");
    }
    
    ~JakaIKTestNode()
    {
        if (connected_) {
            RCLCPP_INFO(get_logger(), "断开连接...");
            robot_->login_out();
        }
    }

private:
    void connectToRobot()
    {
        RCLCPP_INFO(get_logger(), "尝试连接到 %s...", robot_ip_.c_str());
        
        errno_t ret = robot_->login_in(robot_ip_.c_str());
        if (ret == ERR_SUCC) {
            connected_ = true;
            RCLCPP_INFO(get_logger(), "✅ 成功连接到控制器！");
            RCLCPP_INFO(get_logger(), "⚠️  注意：如果qyh_jaka_control也在运行，");
            RCLCPP_INFO(get_logger(), "   现在测试的是【多客户端同时连接】");
            
            // 发布连接状态
            auto msg = std_msgs::msg::String();
            msg.data = "Connected to " + robot_ip_;
            status_pub_->publish(msg);

            // 运行一次完整性检查
            runSanityCheck();

        } else {
            connected_ = false;
            RCLCPP_ERROR(get_logger(), "❌ 连接失败！错误码: %d", ret);
            RCLCPP_WARN(get_logger(), "可能原因：");
            RCLCPP_WARN(get_logger(), "  1. 网络不通");
            RCLCPP_WARN(get_logger(), "  2. IP地址错误");
            RCLCPP_WARN(get_logger(), "  3. SDK不支持多客户端连接");
        }
    }

    void runSanityCheck()
    {
        RCLCPP_INFO(get_logger(), "🔍 运行IK完整性检查 (模仿25.kine)...");
        
        // 构造全90度关节角 - 使用数组以防库函数越界读取
        JointValue start_pos[2] = { { 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad},
                                 { 90 * deg_tp_rad, -45 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 90 * deg_tp_rad} };    
        CartesianPose pos[2];
        robot_->kine_forward(0, &start_pos[0], &pos[0]);
        robot_->kine_forward(1, &start_pos[1], &pos[1]);
        printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
        printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

        JointValue end_pos[2];
        pos[0].tran.x += 20;
        pos[0].tran.y += 20;
        pos[0].tran.z += 20;
        pos[1].tran.x += 20;
        pos[1].tran.y += 20;
        pos[1].tran.z += 20;
        errno_t ret = robot_->kine_inverse(0, &start_pos[0], &pos[0], &end_pos[0]);
        robot_->kine_inverse(1, &start_pos[1], &pos[1], &end_pos[1]);

        printf("left end pos = %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[0].jVal[0], end_pos[0].jVal[1], end_pos[0].jVal[2], end_pos[0].jVal[3], end_pos[0].jVal[4], end_pos[0].jVal[5], end_pos[0].jVal[6]);
        printf("right end pos = %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[1].jVal[0], end_pos[1].jVal[1], end_pos[1].jVal[2], end_pos[1].jVal[3], end_pos[1].jVal[4], end_pos[1].jVal[5], end_pos[0].jVal[6]);

        robot_->kine_forward(0, &end_pos[0], &pos[0]);
        robot_->kine_forward(1, &end_pos[1], &pos[1]);
        printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
        printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

        if (ret == ERR_SUCC || ret == -24) {
            RCLCPP_INFO(get_logger(), "✅ Sanity Check IK (LEFT) SUCCESS! (ret=%d)", ret);
        } else {
            RCLCPP_ERROR(get_logger(), "❌ Sanity Check IK (LEFT) FAILED: %d", ret);
        }
    }
    
    void ikTestCallback()
    {
        if (!connected_) {
            return;
        }

        // 初始化基准位姿 (仅执行一次)
        if (!base_pose_initialized_) {
            JointValue ref_joint;
            double val_90deg = 90 * deg_tp_rad;
            for(int i=0; i<7; ++i) ref_joint.jVal[i] = val_90deg;
            
            errno_t ret = robot_->kine_forward(0, &ref_joint, &base_pose_);
            if (ret == ERR_SUCC) {
                base_pose_initialized_ = true;
                RCLCPP_INFO(get_logger(), "✅ 基准位姿初始化成功: [%.2f, %.2f, %.2f] RPY:[%.2f, %.2f, %.2f]", 
                    base_pose_.tran.x, base_pose_.tran.y, base_pose_.tran.z,
                    base_pose_.rpy.rx, base_pose_.rpy.ry, base_pose_.rpy.rz);
            } else {
                RCLCPP_ERROR(get_logger(), "❌ 基准位姿初始化失败 (FK错误码: %d)", ret);
                return;
            }
        }
        
        test_count_++;
        
        // 基于基准位姿构造目标位姿
        double t = test_count_ * 0.01;  // 时间参数
        CartesianPose target_pose = base_pose_;
        
        // 模仿25.kine，增加偏移量，避免奇异点
        target_pose.tran.x += 20.0;
        target_pose.tran.y += 20.0;
        target_pose.tran.z += 20.0 + 10.0 * std::sin(t);
        
        // 使用全90度作为参考关节角度
        JointValue ref_joint;
        double val_90deg = 90 * deg_tp_rad;
        for(int i=0; i<7; ++i) {
            ref_joint.jVal[i] = val_90deg;
        }
        
        // 调用IK求解
        // robot_id=0 表示左臂
        JointValue ik_result;
        errno_t ret = robot_->kine_inverse(0, &ref_joint, &target_pose, &ik_result);
        
        if (ret == ERR_SUCC || ret == -24) {
            success_count_++;
            
            // 发布IK结果
            auto joint_msg = sensor_msgs::msg::JointState();
            joint_msg.header.stamp = now();
            joint_msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
            joint_msg.position = {
                ik_result.jVal[0], ik_result.jVal[1], ik_result.jVal[2],
                ik_result.jVal[3], ik_result.jVal[4], ik_result.jVal[5],
                ik_result.jVal[6]
            };
            ik_result_pub_->publish(joint_msg);
            
            // 每秒打印一次统计
            if (test_count_ % 100 == 0) {
                double success_rate = 100.0 * success_count_ / test_count_;
                RCLCPP_INFO(get_logger(), 
                    "📊 IK统计: 总计=%ld, 成功=%ld, 失败=%ld, 成功率=%.1f%%",
                    test_count_, success_count_, error_count_, success_rate);
            }
        } else {
            error_count_++;
            if (error_count_ % 100 == 0) {
                RCLCPP_WARN(get_logger(), 
                    "IK求解失败 (错误计数: %ld, 错误码: %d)", error_count_, ret);
            }
        }
    }

    // ROS相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ik_result_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // JAKA SDK
    std::unique_ptr<JAKAZuRobot> robot_;
    std::string robot_ip_;
    bool connected_{false};
    bool auto_connect_{true};
    
    // IK测试相关
    CartesianPose base_pose_;
    bool base_pose_initialized_{false};
    
    // 统计
    int64_t test_count_{0};
    int64_t success_count_{0};
    int64_t error_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JakaIKTestNode>();
    
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "🔥 开始高频IK测试...");
    RCLCPP_INFO(node->get_logger(), "📌 同时运行qyh_jaka_control可测试多客户端连接");
    RCLCPP_INFO(node->get_logger(), " ");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
