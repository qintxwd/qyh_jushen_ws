/**
 * @file coordinate_mapper_node.cpp
 * @brief VR坐标映射与滤波节点
 * 
 * 功能：
 * - 监听VR手柄TF变换
 * - 坐标轴对齐：VR坐标系 → 人手语义坐标系
 * - 低通滤波：消除抖动和噪声
 * - 速度/加速度限制：安全约束
 * - 位置缩放：调整操作范围
 * - 发布目标位姿给IK求解器
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class CoordinateMapperNode : public rclcpp::Node
{
public:
    CoordinateMapperNode() : Node("coordinate_mapper_node")
    {
        // 参数声明
        declare_parameter<double>("update_rate", 100.0);  // 100Hz
        declare_parameter<double>("position_scale", 1.0);
        declare_parameter<double>("rotation_scale", 1.0);
        declare_parameter<double>("filter_alpha", 0.3);  // 滤波系数
        declare_parameter<double>("max_linear_velocity", 0.5);  // m/s
        declare_parameter<double>("max_angular_velocity", 1.0);  // rad/s
        declare_parameter<bool>("enable_filter", true);
        declare_parameter<bool>("enable_velocity_limit", true);
        
        update_rate_ = get_parameter("update_rate").as_double();
        position_scale_ = get_parameter("position_scale").as_double();
        rotation_scale_ = get_parameter("rotation_scale").as_double();
        filter_alpha_ = get_parameter("filter_alpha").as_double();
        max_linear_vel_ = get_parameter("max_linear_velocity").as_double();
        max_angular_vel_ = get_parameter("max_angular_velocity").as_double();
        enable_filter_ = get_parameter("enable_filter").as_bool();
        enable_velocity_limit_ = get_parameter("enable_velocity_limit").as_bool();
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "  坐标映射与滤波节点启动");
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "更新频率: %.1f Hz", update_rate_);
        RCLCPP_INFO(get_logger(), "位置缩放: %.2f", position_scale_);
        RCLCPP_INFO(get_logger(), "滤波系数: %.2f", filter_alpha_);
        RCLCPP_INFO(get_logger(), "最大线速度: %.2f m/s", max_linear_vel_);
        RCLCPP_INFO(get_logger(), "启用滤波: %s", enable_filter_ ? "是" : "否");
        RCLCPP_INFO(get_logger(), "启用速度限制: %s", enable_velocity_limit_ ? "是" : "否");
        
        // TF监听器和发布器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // 发布目标位姿
        left_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/teleop/left_hand/target", 10);
        
        right_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/teleop/right_hand/target", 10);
        
        // 定时器
        auto period = std::chrono::duration<double>(1.0 / update_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&CoordinateMapperNode::updateCallback, this));
        
        RCLCPP_INFO(get_logger(), "✓ 坐标映射节点初始化完成");
        RCLCPP_INFO(get_logger(), "等待VR手柄TF变换...");
    }

private:
    void updateCallback()
    {
        // 处理左手
        processController("left", "vr_left_controller", "human_left_hand", 
                         left_target_pub_, left_prev_pose_, left_prev_time_,
                         left_filter_initialized_);
        
        // 处理右手
        processController("right", "vr_right_controller", "human_right_hand",
                         right_target_pub_, right_prev_pose_, right_prev_time_,
                         right_filter_initialized_);
    }
    
    void processController(const std::string& side,
                          const std::string& source_frame,
                          const std::string& target_frame,
                          rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub,
                          geometry_msgs::msg::Pose& prev_pose,
                          rclcpp::Time& prev_time,
                          bool& filter_initialized)
    {
        try {
            // 查询TF：vr_origin -> vr_*_controller
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "vr_origin", source_frame, tf2::TimePointZero);
            
            // 提取位姿
            geometry_msgs::msg::Pose vr_pose;
            vr_pose.position.x = transform_stamped.transform.translation.x;
            vr_pose.position.y = transform_stamped.transform.translation.y;
            vr_pose.position.z = transform_stamped.transform.translation.z;
            vr_pose.orientation = transform_stamped.transform.rotation;
            
            // 1. 坐标轴对齐：VR → 人手语义
            geometry_msgs::msg::Pose human_pose = vrToHumanTransform(vr_pose);
            
            // 2. 位置缩放
            human_pose.position.x *= position_scale_;
            human_pose.position.y *= position_scale_;
            human_pose.position.z *= position_scale_;
            
            // 3. 低通滤波
            if (enable_filter_) {
                if (filter_initialized) {
                    human_pose = applyLowPassFilter(human_pose, prev_pose);
                } else {
                    filter_initialized = true;
                }
            }
            
            // 4. 速度限制
            if (enable_velocity_limit_ && filter_initialized) {
                auto current_time = now();
                double dt = (current_time - prev_time).seconds();
                if (dt > 0.0) {
                    human_pose = applyVelocityLimit(human_pose, prev_pose, dt);
                }
                prev_time = current_time;
            } else {
                prev_time = now();
            }
            
            // 更新历史
            prev_pose = human_pose;
            
            // 5. 发布TF：vr_*_controller → human_*_hand
            publishHumanHandTF(source_frame, target_frame, human_pose);
            
            // 6. 发布目标位姿话题
            auto target_msg = geometry_msgs::msg::PoseStamped();
            target_msg.header.stamp = now();
            target_msg.header.frame_id = "vr_origin";  // 或 "base_link_left/right"
            target_msg.pose = human_pose;
            pub->publish(target_msg);
            
        } catch (tf2::TransformException& ex) {
            // TF查询失败（VR手柄未连接或TF延迟）
            if (update_count_ % 100 == 0) {  // 每秒打印一次
                RCLCPP_DEBUG(get_logger(), "%s手柄TF查询失败: %s", 
                    side.c_str(), ex.what());
            }
        }
        
        update_count_++;
    }
    
    /**
     * VR坐标系 → 人手语义坐标系
     * VR: X-右, Y-上, Z-后 → Human: X-前, Y-左, Z-上
     */
    geometry_msgs::msg::Pose vrToHumanTransform(const geometry_msgs::msg::Pose& vr_pose)
    {
        geometry_msgs::msg::Pose human_pose;
        
        // 位置变换：轴映射
        // X_human(前) = -Z_vr(后)
        // Y_human(左) = -X_vr(右)
        // Z_human(上) = Y_vr(上)
        human_pose.position.x = -vr_pose.position.z;
        human_pose.position.y = -vr_pose.position.x;
        human_pose.position.z = vr_pose.position.y;
        
        // 旋转变换：R_human = R_align * R_vr * R_align^T
        tf2::Quaternion q_vr(
            vr_pose.orientation.x,
            vr_pose.orientation.y,
            vr_pose.orientation.z,
            vr_pose.orientation.w
        );
        
        // 对齐旋转矩阵
        tf2::Matrix3x3 R_align(
            0,  0, -1,
           -1,  0,  0,
            0,  1,  0
        );
        
        tf2::Matrix3x3 R_vr(q_vr);
        tf2::Matrix3x3 R_human = R_align * R_vr * R_align.transpose();
        
        tf2::Quaternion q_human;
        R_human.getRotation(q_human);
        
        human_pose.orientation.x = q_human.x();
        human_pose.orientation.y = q_human.y();
        human_pose.orientation.z = q_human.z();
        human_pose.orientation.w = q_human.w();
        
        return human_pose;
    }
    
    /**
     * 低通滤波：指数移动平均 + 四元数球面插值
     */
    geometry_msgs::msg::Pose applyLowPassFilter(
        const geometry_msgs::msg::Pose& new_pose,
        const geometry_msgs::msg::Pose& prev_pose)
    {
        geometry_msgs::msg::Pose filtered_pose;
        
        // 位置滤波（EMA）
        filtered_pose.position.x = filter_alpha_ * new_pose.position.x + 
                                   (1.0 - filter_alpha_) * prev_pose.position.x;
        filtered_pose.position.y = filter_alpha_ * new_pose.position.y + 
                                   (1.0 - filter_alpha_) * prev_pose.position.y;
        filtered_pose.position.z = filter_alpha_ * new_pose.position.z + 
                                   (1.0 - filter_alpha_) * prev_pose.position.z;
        
        // 旋转滤波（Slerp）
        tf2::Quaternion q_new(
            new_pose.orientation.x,
            new_pose.orientation.y,
            new_pose.orientation.z,
            new_pose.orientation.w
        );
        
        tf2::Quaternion q_prev(
            prev_pose.orientation.x,
            prev_pose.orientation.y,
            prev_pose.orientation.z,
            prev_pose.orientation.w
        );
        
        tf2::Quaternion q_filtered = q_prev.slerp(q_new, filter_alpha_);
        
        filtered_pose.orientation.x = q_filtered.x();
        filtered_pose.orientation.y = q_filtered.y();
        filtered_pose.orientation.z = q_filtered.z();
        filtered_pose.orientation.w = q_filtered.w();
        
        return filtered_pose;
    }
    
    /**
     * 速度限制
     */
    geometry_msgs::msg::Pose applyVelocityLimit(
        const geometry_msgs::msg::Pose& target_pose,
        const geometry_msgs::msg::Pose& prev_pose,
        double dt)
    {
        geometry_msgs::msg::Pose limited_pose = target_pose;
        
        // 计算位置变化
        double dx = target_pose.position.x - prev_pose.position.x;
        double dy = target_pose.position.y - prev_pose.position.y;
        double dz = target_pose.position.z - prev_pose.position.z;
        double linear_vel = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;
        
        // 限制线速度
        if (linear_vel > max_linear_vel_) {
            double scale = max_linear_vel_ / linear_vel;
            limited_pose.position.x = prev_pose.position.x + dx * scale;
            limited_pose.position.y = prev_pose.position.y + dy * scale;
            limited_pose.position.z = prev_pose.position.z + dz * scale;
        }
        
        // 计算角速度（简化：使用四元数点积）
        tf2::Quaternion q_target(
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        );
        
        tf2::Quaternion q_prev(
            prev_pose.orientation.x,
            prev_pose.orientation.y,
            prev_pose.orientation.z,
            prev_pose.orientation.w
        );
        
        double dot = q_target.dot(q_prev);
        double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
        double angular_vel = angle / dt;
        
        // 限制角速度
        if (angular_vel > max_angular_vel_) {
            double t = max_angular_vel_ * dt / angle;
            tf2::Quaternion q_limited = q_prev.slerp(q_target, t);
            limited_pose.orientation.x = q_limited.x();
            limited_pose.orientation.y = q_limited.y();
            limited_pose.orientation.z = q_limited.z();
            limited_pose.orientation.w = q_limited.w();
        }
        
        return limited_pose;
    }
    
    /**
     * 发布TF：vr_*_controller → human_*_hand
     */
    void publishHumanHandTF(const std::string& source_frame,
                           const std::string& target_frame,
                           const geometry_msgs::msg::Pose& pose)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();
        transform.header.frame_id = source_frame;
        transform.child_frame_id = target_frame;
        
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation = pose.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }

    // ROS相关
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pub_;
    
    // 参数
    double update_rate_;
    double position_scale_;
    double rotation_scale_;
    double filter_alpha_;
    double max_linear_vel_;
    double max_angular_vel_;
    bool enable_filter_;
    bool enable_velocity_limit_;
    
    // 滤波状态
    geometry_msgs::msg::Pose left_prev_pose_;
    geometry_msgs::msg::Pose right_prev_pose_;
    rclcpp::Time left_prev_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time right_prev_time_{0, 0, RCL_ROS_TIME};
    bool left_filter_initialized_{false};
    bool right_filter_initialized_{false};
    
    // 统计
    int update_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinateMapperNode>();
    
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "🚀 坐标映射与滤波节点运行中...");
    RCLCPP_INFO(node->get_logger(), "📌 VR → 人手语义 → 目标位姿");
    RCLCPP_INFO(node->get_logger(), " ");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
