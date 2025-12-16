#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * Teleop Manager Node (节点2/5) - VR遥操作状态管理
 * 
 * 职责：
 *   1. 监听grip按钮状态（clutch机制）
 *   2. 管理遥操作状态机：IDLE → ENGAGED → TRACKING → RELEASING
 *   3. 零位校准：记录按下grip时的VR位姿作为vr_origin
 *   4. 发布动态TF: teleop_base → vr_origin
 *   5. 提供服务：start/stop/recenter
 * 
 * TF发布策略：
 *   - 静态TF: base_link → teleop_base (由launch文件发布)
 *   - 动态TF: teleop_base → vr_origin (本节点发布)
 *   
 *   未校准时：vr_origin = teleop_base (单位变换)
 *   已校准后：vr_origin固定在按下grip时的位置
 * 
 * Clutch状态：
 *   - IDLE: grip松开，不发布vr_origin（使用单位变换）
 *   - ENGAGING: 刚按下grip，记录零位
 *   - TRACKING: grip保持按下，持续跟踪
 *   - RELEASING: 刚松开grip，准备回到IDLE
 */

enum class ClutchState {
    IDLE = 0,
    ENGAGING = 1,
    TRACKING = 2,
    RELEASING = 3
};

class TeleopManagerNode : public rclcpp::Node
{
public:
    TeleopManagerNode() : Node("teleop_manager_node")
    {
        // Parameters
        this->declare_parameter("grip_engage_threshold", 0.8);
        this->declare_parameter("grip_release_threshold", 0.2);
        this->declare_parameter("update_rate", 100.0);  // Hz
        
        grip_engage_threshold_ = this->get_parameter("grip_engage_threshold").as_double();
        grip_release_threshold_ = this->get_parameter("grip_release_threshold").as_double();
        double update_rate = this->get_parameter("update_rate").as_double();
        
        // Subscribers
        left_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/vr/left_controller/joy", 10,
            std::bind(&TeleopManagerNode::left_joy_callback, this, std::placeholders::_1));
        
        right_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/vr/right_controller/joy", 10,
            std::bind(&TeleopManagerNode::right_joy_callback, this, std::placeholders::_1));
        
        // TF Broadcasters
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Services
        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/teleop/start",
            std::bind(&TeleopManagerNode::start_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/teleop/stop",
            std::bind(&TeleopManagerNode::stop_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        recenter_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/teleop/recenter",
            std::bind(&TeleopManagerNode::recenter_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer for TF publishing
        auto period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TeleopManagerNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Teleop Manager Node (2/5) ===");
        RCLCPP_INFO(this->get_logger(), "Role: VR state management and zero-point calibration");
        RCLCPP_INFO(this->get_logger(), "Grip engage: %.2f, release: %.2f", 
                   grip_engage_threshold_, grip_release_threshold_);
        RCLCPP_INFO(this->get_logger(), "Services: /teleop/{start,stop,recenter}");
    }

private:
    void left_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Joy axes: [joystick_x, joystick_y, trigger, grip]
        if (msg->axes.size() >= 4) {
            left_grip_value_ = msg->axes[3];
            update_clutch_state(left_clutch_state_, left_grip_value_, "LEFT");
        }
    }
    
    void right_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() >= 4) {
            right_grip_value_ = msg->axes[3];
            update_clutch_state(right_clutch_state_, right_grip_value_, "RIGHT");
        }
    }
    
    void update_clutch_state(ClutchState& state, float grip_value, const std::string& hand)
    {
        bool clutch_pressed = grip_value > grip_engage_threshold_;
        bool clutch_released = grip_value < grip_release_threshold_;
        
        ClutchState prev_state = state;
        
        switch (state) {
            case ClutchState::IDLE:
                if (clutch_pressed) {
                    state = ClutchState::ENGAGING;
                    RCLCPP_INFO(this->get_logger(), "[%s] Clutch ENGAGED (grip=%.2f)", 
                               hand.c_str(), grip_value);
                }
                break;
            
            case ClutchState::ENGAGING:
                state = ClutchState::TRACKING;
                break;
            
            case ClutchState::TRACKING:
                if (clutch_released) {
                    state = ClutchState::RELEASING;
                    RCLCPP_INFO(this->get_logger(), "[%s] Clutch RELEASED (grip=%.2f)", 
                               hand.c_str(), grip_value);
                }
                break;
            
            case ClutchState::RELEASING:
                state = ClutchState::IDLE;
                break;
        }
    }
    
    void timer_callback()
    {
        auto now = this->now();
        
        // 发布 teleop_base → vr_origin TF
        // 策略：
        // - 如果左手或右手任一处于TRACKING状态，发布单位变换（vr_origin = teleop_base）
        // - 这样vr_bridge_node发布的 vr_origin → vr_*_controller 就是相对于teleop_base的
        
        bool any_tracking = (left_clutch_state_ == ClutchState::TRACKING || 
                            left_clutch_state_ == ClutchState::ENGAGING) ||
                           (right_clutch_state_ == ClutchState::TRACKING || 
                            right_clutch_state_ == ClutchState::ENGAGING);
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "teleop_base";
        t.child_frame_id = "vr_origin";
        
        // 单位变换（零位校准由vr_bridge_node处理，这里只管理启用状态）
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(t);
        
        // 调试输出（降频）
        static int count = 0;
        if (++count % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "State: L=%d(%.2f) R=%d(%.2f) Tracking=%s",
                static_cast<int>(left_clutch_state_), left_grip_value_,
                static_cast<int>(right_clutch_state_), right_grip_value_,
                any_tracking ? "YES" : "NO");
        }
    }
    
    void start_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Teleop START requested");
        response->success = true;
        response->message = "Teleoperation started. Press grip to engage clutch.";
    }
    
    void stop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Teleop STOP requested");
        left_clutch_state_ = ClutchState::IDLE;
        right_clutch_state_ = ClutchState::IDLE;
        response->success = true;
        response->message = "Teleoperation stopped.";
    }
    
    void recenter_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Teleop RECENTER requested");
        // 重置状态，下次按下grip时会重新校准
        left_clutch_state_ = ClutchState::IDLE;
        right_clutch_state_ = ClutchState::IDLE;
        response->success = true;
        response->message = "Zero point will be reset on next grip press.";
    }

    // Parameters
    double grip_engage_threshold_;
    double grip_release_threshold_;
    
    // State
    ClutchState left_clutch_state_ = ClutchState::IDLE;
    ClutchState right_clutch_state_ = ClutchState::IDLE;
    float left_grip_value_ = 0.0f;
    float right_grip_value_ = 0.0f;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_joy_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recenter_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
