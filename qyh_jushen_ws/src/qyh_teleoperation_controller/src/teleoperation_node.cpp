#include <memory>
#include <chrono>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "qyh_teleoperation_controller/differential_ik_controller.hpp"
#include "qyh_teleoperation_controller/virtual_arm_follower.hpp"

using namespace std::chrono_literals;

class TeleoperationNode : public rclcpp::Node
{
public:
  TeleoperationNode()
  : Node("teleoperation_node"),
    left_clutch_engaged_(false),
    right_clutch_engaged_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("robot_description", "robot_description");
    this->declare_parameter<std::string>("left_arm_group", "left_arm");
    this->declare_parameter<std::string>("right_arm_group", "right_arm");
    this->declare_parameter<double>("control_frequency", 125.0);
    this->declare_parameter<double>("max_velocity", 1.0);
    this->declare_parameter<double>("max_acceleration", 0.5);
    
    // Get parameters
    std::string robot_desc_param = this->get_parameter("robot_description").as_string();
    std::string left_group = this->get_parameter("left_arm_group").as_string();
    std::string right_group = this->get_parameter("right_arm_group").as_string();
    double control_freq = this->get_parameter("control_frequency").as_double();
    
    // Load robot model
    RCLCPP_INFO(this->get_logger(), "Loading robot model...");
    robot_model_loader::RobotModelLoader robot_model_loader(
      shared_from_this(), robot_desc_param);
    robot_model_ = robot_model_loader.getModel();
    
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
      rclcpp::shutdown();
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully");
    
    // Create controllers for both arms
    try {
      left_controller_ = std::make_shared<qyh_teleoperation_controller::DifferentialIKController>(
        shared_from_this(), robot_model_, left_group);
      right_controller_ = std::make_shared<qyh_teleoperation_controller::DifferentialIKController>(
        shared_from_this(), robot_model_, right_group);
      
      RCLCPP_INFO(this->get_logger(), "Controllers initialized");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize controllers: %s", e.what());
      rclcpp::shutdown();
      return;
    }
    
    // Create virtual arm followers
    left_virtual_arm_ = std::make_shared<qyh_teleoperation_controller::VirtualArmFollower>(
      shared_from_this(), "left");
    right_virtual_arm_ = std::make_shared<qyh_teleoperation_controller::VirtualArmFollower>(
      shared_from_this(), "right");
    
    // Subscribe to VR target poses
    left_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vr/left_target_pose", 10,
      std::bind(&TeleoperationNode::leftTargetCallback, this, std::placeholders::_1));
    
    right_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vr/right_target_pose", 10,
      std::bind(&TeleoperationNode::rightTargetCallback, this, std::placeholders::_1));
    
    // Subscribe to clutch status
    left_clutch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vr/left_clutch_engaged", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        left_clutch_engaged_ = msg->data;
      });
    
    right_clutch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/vr/right_clutch_engaged", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        right_clutch_engaged_ = msg->data;
      });
    
    // Subscribe to current joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&TeleoperationNode::jointStateCallback, this, std::placeholders::_1));
    
    // Publishers for real robot commands
    left_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/left_arm/joint_command", 10);
    right_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/right_arm/joint_command", 10);
    
    // Initialize current states
    left_current_state_ = qyh_teleoperation_controller::JointState(7);
    right_current_state_ = qyh_teleoperation_controller::JointState(7);
    
    RCLCPP_INFO(this->get_logger(), 
      "Teleoperation node initialized at %.1f Hz", control_freq);
    RCLCPP_INFO(this->get_logger(), "Using Clutch mode - waiting for grip button...");
  }

private:
  void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (left_controller_) {
      auto command = left_controller_->computeJointCommand(*msg, left_current_state_);
      left_virtual_arm_->updateState(command);
      
      // Publish command to real robot
      publishJointCommand(left_command_pub_, command, "left");
    }
  }
  
  void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (right_controller_) {
      auto command = right_controller_->computeJointCommand(*msg, right_current_state_);
      right_virtual_arm_->updateState(command);
      
      // Publish command to real robot
      publishJointCommand(right_command_pub_, command, "right");
    }
  }
  
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Update current states from real robot feedback
    updateCurrentState(msg, "left", left_current_state_);
    updateCurrentState(msg, "right", right_current_state_);
  }
  
  void updateCurrentState(
    const sensor_msgs::msg::JointState::SharedPtr& msg,
    const std::string& arm_prefix,
    qyh_teleoperation_controller::JointState& state)
  {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i].find(arm_prefix) != std::string::npos) {
        size_t joint_num = std::stoi(msg->name[i].substr(msg->name[i].find("joint") + 5));
        if (joint_num >= 1 && joint_num <= 7) {
          state.position[joint_num - 1] = msg->position[i];
          if (i < msg->velocity.size()) {
            state.velocity[joint_num - 1] = msg->velocity[i];
          }
        }
      }
    }
    state.timestamp = rclcpp::Time(msg->header.stamp);
  }
  
  void publishJointCommand(
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub,
    const qyh_teleoperation_controller::JointState& state,
    const std::string& arm_name)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    
    for (int i = 1; i <= 7; ++i) {
      msg.name.push_back(arm_name + "_joint" + std::to_string(i));
    }
    
    msg.position = state.position;
    msg.velocity = state.velocity;
    
    pub->publish(msg);
  }
  
  moveit::core::RobotModelPtr robot_model_;
  
  std::shared_ptr<qyh_teleoperation_controller::DifferentialIKController> left_controller_;
  std::shared_ptr<qyh_teleoperation_controller::DifferentialIKController> right_controller_;
  
  std::shared_ptr<qyh_teleoperation_controller::VirtualArmFollower> left_virtual_arm_;
  std::shared_ptr<qyh_teleoperation_controller::VirtualArmFollower> right_virtual_arm_;
  
  qyh_teleoperation_controller::JointState left_current_state_;
  qyh_teleoperation_controller::JointState right_current_state_;
  
  // Clutch state
  std::atomic<bool> left_clutch_engaged_;
  std::atomic<bool> right_clutch_engaged_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_clutch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_clutch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_command_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleoperationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
