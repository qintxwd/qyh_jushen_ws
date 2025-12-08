#include "qyh_head_motor_control/head_motor_node.hpp"
#include <chrono>
#include <cmath>

namespace qyh_head_motor_control
{

HeadMotorNode::HeadMotorNode(const rclcpp::NodeOptions& options)
    : Node("head_motor_node", options)
{
    initParameters();
    
    if (!initCommunication()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication!");
    }
    
    initTopicsAndServices();
    
    RCLCPP_INFO(this->get_logger(), "Head motor node initialized");
    RCLCPP_INFO(this->get_logger(), "  Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Baudrate: %d", baudrate_);
    RCLCPP_INFO(this->get_logger(), "  Motor count: %zu", motor_ids_.size());
}

HeadMotorNode::~HeadMotorNode()
{
    if (protocol_) {
        protocol_->close();
    }
}

void HeadMotorNode::initParameters()
{
    // 串口参数
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baudrate", 1000000);
    
    // 电机配置
    this->declare_parameter("motor_ids", std::vector<int64_t>{1, 2});
    this->declare_parameter("joint_names", std::vector<std::string>{"head_pan_joint", "head_tilt_joint"});
    
    // 位置限制 (弧度)
    this->declare_parameter("position_min_rad", std::vector<double>{-1.57, -0.785});
    this->declare_parameter("position_max_rad", std::vector<double>{1.57, 0.785});
    
    // 控制参数
    this->declare_parameter("publish_rate", 20);
    this->declare_parameter("move_duration_ms", 100);
    
    // 获取参数
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    motor_ids_ = this->get_parameter("motor_ids").as_integer_array();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    position_min_rad_ = this->get_parameter("position_min_rad").as_double_array();
    position_max_rad_ = this->get_parameter("position_max_rad").as_double_array();
    publish_rate_ = this->get_parameter("publish_rate").as_int();
    move_duration_ms_ = this->get_parameter("move_duration_ms").as_int();
    
    // 验证参数
    if (motor_ids_.size() != joint_names_.size()) {
        RCLCPP_WARN(this->get_logger(), 
            "motor_ids size (%zu) != joint_names size (%zu), using motor_ids size",
            motor_ids_.size(), joint_names_.size());
        joint_names_.resize(motor_ids_.size());
        for (size_t i = 0; i < motor_ids_.size(); i++) {
            if (joint_names_[i].empty()) {
                joint_names_[i] = "head_joint_" + std::to_string(i);
            }
        }
    }
    
    // 初始化位置数组
    current_positions_.resize(motor_ids_.size(), 500);  // 中间位置
}

bool HeadMotorNode::initCommunication()
{
    protocol_ = std::make_unique<BusServoProtocol>(serial_port_, baudrate_);
    
    if (!protocol_->open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Serial port opened: %s @ %d baud", 
                serial_port_.c_str(), baudrate_);
    
    // 尝试读取所有电机位置
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        uint16_t pos;
        if (protocol_->readPosition(static_cast<uint8_t>(motor_ids_[i]), pos)) {
            current_positions_[i] = pos;
            RCLCPP_INFO(this->get_logger(), "Motor %ld initial position: %u", 
                        motor_ids_[i], pos);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read initial position for motor %ld",
                        motor_ids_[i]);
        }
    }
    
    return true;
}

void HeadMotorNode::initTopicsAndServices()
{
    // 发布者 - 关节状态
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_states", 10);
    
    // 订阅者 - 归一化位置命令 [-1, 1]
    cmd_position_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/cmd_position", 10,
        std::bind(&HeadMotorNode::cmdPositionCallback, this, std::placeholders::_1));
    
    // 订阅者 - 原始位置命令 [0-1000]
    cmd_position_raw_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "~/cmd_position_raw", 10,
        std::bind(&HeadMotorNode::cmdPositionRawCallback, this, std::placeholders::_1));
    
    // 服务 - 使能扭矩
    enable_torque_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_torque",
        std::bind(&HeadMotorNode::enableTorqueCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // 定时器 - 发布关节状态
    auto period = std::chrono::milliseconds(1000 / publish_rate_);
    publish_timer_ = this->create_wall_timer(
        period, std::bind(&HeadMotorNode::publishCallback, this));
}

void HeadMotorNode::publishCallback()
{
    if (!protocol_ || !protocol_->isOpen()) {
        return;
    }
    
    // 读取所有电机位置
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        uint16_t pos;
        if (protocol_->readPosition(static_cast<uint8_t>(motor_ids_[i]), pos)) {
            current_positions_[i] = pos;
        }
    }
    
    // 发布关节状态
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        msg.position.push_back(rawToRadian(current_positions_[i], i));
        msg.velocity.push_back(0.0);  // 暂不支持速度读取
        msg.effort.push_back(0.0);    // 暂不支持力矩读取
    }
    
    joint_state_pub_->publish(msg);
}

void HeadMotorNode::cmdPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!protocol_ || !protocol_->isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Serial port not open");
        return;
    }
    
    if (msg->data.size() != motor_ids_.size()) {
        RCLCPP_WARN(this->get_logger(), 
            "Position command size (%zu) != motor count (%zu)",
            msg->data.size(), motor_ids_.size());
        return;
    }
    
    // 转换为原始位置并发送
    std::vector<uint8_t> ids;
    std::vector<uint16_t> positions;
    
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        ids.push_back(static_cast<uint8_t>(motor_ids_[i]));
        positions.push_back(normalizedToRaw(msg->data[i]));
    }
    
    if (!protocol_->setPositions(ids, positions, static_cast<uint16_t>(move_duration_ms_))) {
        RCLCPP_WARN(this->get_logger(), "Failed to set positions");
    }
}

void HeadMotorNode::cmdPositionRawCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (!protocol_ || !protocol_->isOpen()) {
        RCLCPP_WARN(this->get_logger(), "Serial port not open");
        return;
    }
    
    if (msg->data.size() != motor_ids_.size()) {
        RCLCPP_WARN(this->get_logger(), 
            "Position command size (%zu) != motor count (%zu)",
            msg->data.size(), motor_ids_.size());
        return;
    }
    
    // 发送原始位置
    std::vector<uint8_t> ids;
    std::vector<uint16_t> positions;
    
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        ids.push_back(static_cast<uint8_t>(motor_ids_[i]));
        uint16_t pos = static_cast<uint16_t>(std::max(0, std::min(1000, msg->data[i])));
        positions.push_back(pos);
    }
    
    if (!protocol_->setPositions(ids, positions, static_cast<uint16_t>(move_duration_ms_))) {
        RCLCPP_WARN(this->get_logger(), "Failed to set positions");
    }
}

void HeadMotorNode::enableTorqueCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    if (!protocol_ || !protocol_->isOpen()) {
        response->success = false;
        response->message = "Serial port not open";
        return;
    }
    
    bool all_success = true;
    for (size_t i = 0; i < motor_ids_.size(); i++) {
        if (!protocol_->enableTorque(static_cast<uint8_t>(motor_ids_[i]), request->data)) {
            all_success = false;
            RCLCPP_WARN(this->get_logger(), "Failed to set torque for motor %ld", motor_ids_[i]);
        }
    }
    
    response->success = all_success;
    response->message = request->data ? "Torque enabled" : "Torque disabled";
}

uint16_t HeadMotorNode::normalizedToRaw(double normalized) const
{
    // [-1, 1] -> [0, 1000]
    double clamped = std::max(-1.0, std::min(1.0, normalized));
    return static_cast<uint16_t>((clamped + 1.0) * 500.0);
}

double HeadMotorNode::rawToNormalized(uint16_t raw) const
{
    // [0, 1000] -> [-1, 1]
    return (static_cast<double>(raw) / 500.0) - 1.0;
}

double HeadMotorNode::rawToRadian(uint16_t raw, size_t motor_index) const
{
    // [0, 1000] -> [min_rad, max_rad]
    double min_rad = (motor_index < position_min_rad_.size()) ? position_min_rad_[motor_index] : -M_PI;
    double max_rad = (motor_index < position_max_rad_.size()) ? position_max_rad_[motor_index] : M_PI;
    
    double ratio = static_cast<double>(raw) / 1000.0;
    return min_rad + ratio * (max_rad - min_rad);
}

}  // namespace qyh_head_motor_control

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<qyh_head_motor_control::HeadMotorNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
