#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/**
 * Coordinate Mapper Node (节点3/5) - VR到人手语义的坐标变换
 * 
 * 职责：
 *   1. 读取 vr_origin → vr_*_controller TF (由vr_bridge发布)
 *   2. 应用握持补偿旋转（可选，如35度pitch）
 *   3. 应用位置缩放（VR移动范围 → 机器人工作空间）
 *   4. 低通滤波（平滑抖动）
 *   5. 速度限制（安全保护）
 *   6. 发布 vr_*_controller → human_*_hand TF
 *   7. 发布目标位姿话题 /teleop/*_hand/target
 * 
 * 坐标系说明：
 *   输入: vr_*_controller (ROS标准坐标系: X前 Y左 Z上)
 *   输出: human_*_hand (人手语义: X手指向前 Y左 Z上)
 *   
 *   目前vr_bridge已经做了PICO→ROS对齐，所以这里主要做：
 *   - 握持补偿（绕Y轴pitch旋转）
 *   - 缩放和滤波
 */

class CoordinateMapperNode : public rclcpp::Node
{
public:
    CoordinateMapperNode() : Node("coordinate_mapper_node")
    {
        // Parameters
        this->declare_parameter("grip_offset_deg", 35.0);      // 握持补偿角度
        this->declare_parameter("position_scale", 2.0);        // 位置缩放因子
        this->declare_parameter("rotation_scale", 1.0);        // 旋转缩放因子
        this->declare_parameter("filter_alpha", 0.3);          // 低通滤波系数
        this->declare_parameter("max_position_delta", 0.05);   // 单帧最大位移(m)
        this->declare_parameter("max_rotation_delta", 0.1);    // 单帧最大旋转(rad)
        this->declare_parameter("update_rate", 100.0);         // Hz
        
        grip_offset_deg_ = this->get_parameter("grip_offset_deg").as_double();
        position_scale_ = this->get_parameter("position_scale").as_double();
        rotation_scale_ = this->get_parameter("rotation_scale").as_double();
        filter_alpha_ = this->get_parameter("filter_alpha").as_double();
        max_position_delta_ = this->get_parameter("max_position_delta").as_double();
        max_rotation_delta_ = this->get_parameter("max_rotation_delta").as_double();
        double update_rate = this->get_parameter("update_rate").as_double();
        
        // 预计算握持补偿四元数
        grip_offset_quat_.setRPY(0, grip_offset_deg_ * M_PI / 180.0, 0);
        grip_offset_quat_.normalize();
        
        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Publishers
        left_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/teleop/left_hand/target", 10);
        right_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/teleop/right_hand/target", 10);
        
        // Timer
        auto period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&CoordinateMapperNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Coordinate Mapper Node (3/5) ===");
        RCLCPP_INFO(this->get_logger(), "Role: VR → human hand coordinate mapping");
        RCLCPP_INFO(this->get_logger(), "Grip offset: %.1f deg, Position scale: %.2f", 
                   grip_offset_deg_, position_scale_);
        RCLCPP_INFO(this->get_logger(), "Filter alpha: %.2f, Max pos delta: %.3f m", 
                   filter_alpha_, max_position_delta_);
    }

private:
    void timer_callback()
    {
        auto now = this->now();
        
        // Process left hand
        process_hand("left", "vr_left_controller", "human_left_hand", 
                    left_prev_pos_, left_prev_quat_, left_target_pub_, now);
        
        // Process right hand
        process_hand("right", "vr_right_controller", "human_right_hand",
                    right_prev_pos_, right_prev_quat_, right_target_pub_, now);
    }
    
    void process_hand(
        const std::string& hand_name,
        const std::string& vr_frame,
        const std::string& human_frame,
        tf2::Vector3& prev_pos,
        tf2::Quaternion& prev_quat,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
        const rclcpp::Time& now)
    {
        // 查询 vr_origin → vr_*_controller TF
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "vr_origin", vr_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            // 没有数据时不警告（可能controller未激活）
            return;
        }
        
        // 提取位置和姿态
        tf2::Vector3 vr_pos(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        
        tf2::Quaternion vr_quat(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        
        // === 1. 应用握持补偿 ===
        tf2::Quaternion compensated_quat = vr_quat * grip_offset_quat_;
        compensated_quat.normalize();
        
        // === 2. 位置缩放 ===
        tf2::Vector3 scaled_pos = vr_pos * position_scale_;
        
        // === 3. 旋转缩放（可选） ===
        tf2::Quaternion scaled_quat = compensated_quat;
        if (rotation_scale_ != 1.0) {
            tf2::Vector3 axis = compensated_quat.getAxis();
            double angle = compensated_quat.getAngle() * rotation_scale_;
            scaled_quat.setRotation(axis, angle);
            scaled_quat.normalize();
        }
        
        // === 4. 低通滤波 ===
        if (prev_pos.length() > 0) {  // 有历史数据
            // 位置滤波
            tf2::Vector3 filtered_pos = prev_pos.lerp(scaled_pos, filter_alpha_);
            
            // 位置变化量限制
            tf2::Vector3 delta_pos = filtered_pos - prev_pos;
            double delta_norm = delta_pos.length();
            if (delta_norm > max_position_delta_) {
                delta_pos = (delta_pos / delta_norm) * max_position_delta_;
                filtered_pos = prev_pos + delta_pos;
            }
            
            // 旋转滤波（slerp）
            tf2::Quaternion filtered_quat = prev_quat.slerp(scaled_quat, filter_alpha_);
            
            // 旋转变化量限制
            tf2::Quaternion delta_quat = prev_quat.inverse() * filtered_quat;
            double delta_angle = delta_quat.getAngle();
            if (delta_angle > max_rotation_delta_) {
                tf2::Vector3 axis = delta_quat.getAxis();
                delta_quat.setRotation(axis, max_rotation_delta_);
                filtered_quat = prev_quat * delta_quat;
            }
            filtered_quat.normalize();
            
            scaled_pos = filtered_pos;
            scaled_quat = filtered_quat;
        }
        
        // 更新历史
        prev_pos = scaled_pos;
        prev_quat = scaled_quat;
        
        // === 5. 发布 TF: vr_*_controller → human_*_hand ===
        geometry_msgs::msg::TransformStamped human_tf;
        human_tf.header.stamp = now;
        human_tf.header.frame_id = vr_frame;
        human_tf.child_frame_id = human_frame;
        
        // 这里发布的是相对于vr_controller的变换（主要是握持补偿旋转）
        // 位置保持一致，旋转应用补偿
        human_tf.transform.translation.x = 0.0;
        human_tf.transform.translation.y = 0.0;
        human_tf.transform.translation.z = 0.0;
        human_tf.transform.rotation.x = grip_offset_quat_.x();
        human_tf.transform.rotation.y = grip_offset_quat_.y();
        human_tf.transform.rotation.z = grip_offset_quat_.z();
        human_tf.transform.rotation.w = grip_offset_quat_.w();
        
        tf_broadcaster_->sendTransform(human_tf);
        
        // === 6. 发布目标位姿话题 ===
        geometry_msgs::msg::PoseStamped target_msg;
        target_msg.header.stamp = now;
        target_msg.header.frame_id = "vr_origin";
        target_msg.pose.position.x = scaled_pos.x();
        target_msg.pose.position.y = scaled_pos.y();
        target_msg.pose.position.z = scaled_pos.z();
        target_msg.pose.orientation.x = scaled_quat.x();
        target_msg.pose.orientation.y = scaled_quat.y();
        target_msg.pose.orientation.z = scaled_quat.z();
        target_msg.pose.orientation.w = scaled_quat.w();
        
        pub->publish(target_msg);
        
        // 调试输出（降频）
        static int count = 0;
        if (++count % 50 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "[%s] Pos: [%.3f, %.3f, %.3f] (scaled)", 
                hand_name.c_str(), scaled_pos.x(), scaled_pos.y(), scaled_pos.z());
        }
    }

    // Parameters
    double grip_offset_deg_;
    double position_scale_;
    double rotation_scale_;
    double filter_alpha_;
    double max_position_delta_;
    double max_rotation_delta_;
    
    // Grip compensation
    tf2::Quaternion grip_offset_quat_;
    
    // Filter state
    tf2::Vector3 left_prev_pos_{0, 0, 0};
    tf2::Quaternion left_prev_quat_{0, 0, 0, 1};
    tf2::Vector3 right_prev_pos_{0, 0, 0};
    tf2::Quaternion right_prev_quat_{0, 0, 0, 1};
    
    // ROS interfaces
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinateMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
