#include "qyh_gripper_control/gripper_control_node.hpp"
#include <chrono>
#include <algorithm>

// Register addresses
#define REG_CONTROL     0x03E8
#define REG_POSITION    0x03E9
#define REG_SPEED_FORCE 0x03EA
#define REG_STATUS      0x07D0
#define REG_FAULT_POS   0x07D1
#define REG_FORCE_SPEED 0x07D2

namespace qyh_gripper_control
{

GripperControlNode::GripperControlNode(const rclcpp::NodeOptions & options)
: Node("gripper_control_node", options),
  modbus_ctx_(nullptr),
  is_connected_(false),
  is_activated_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("device_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("slave_id", 1);
  this->declare_parameter<double>("publish_rate", 20.0);
  
  // Get parameters
  device_port_ = this->get_parameter("device_port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  slave_id_ = this->get_parameter("slave_id").as_int();
  double publish_rate = this->get_parameter("publish_rate").as_double();
  
  RCLCPP_INFO(this->get_logger(), "Starting Gripper Control Node");
  RCLCPP_INFO(this->get_logger(), "Device: %s, Baudrate: %d, Slave ID: %d",
              device_port_.c_str(), baudrate_, slave_id_);
  
  // Connect to gripper
  if (!connect_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to gripper");
  }
  
  // Create publisher
  state_pub_ = this->create_publisher<qyh_gripper_msgs::msg::GripperState>(
    "gripper_state", 10);
  
  // Create services
  srv_activate_ = this->create_service<qyh_gripper_msgs::srv::ActivateGripper>(
    "activate_gripper",
    std::bind(&GripperControlNode::handle_activate, this,
              std::placeholders::_1, std::placeholders::_2));
              
  srv_move_ = this->create_service<qyh_gripper_msgs::srv::MoveGripper>(
    "move_gripper",
    std::bind(&GripperControlNode::handle_move, this,
              std::placeholders::_1, std::placeholders::_2));
              
  srv_get_state_ = this->create_service<qyh_gripper_msgs::srv::GetGripperState>(
    "get_gripper_state",
    std::bind(&GripperControlNode::handle_get_state, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // Create timer for publishing state
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&GripperControlNode::timer_callback, this));
}

GripperControlNode::~GripperControlNode()
{
  disconnect_modbus();
}

bool GripperControlNode::connect_modbus()
{
  try {
    // 创建C++ ModbusRTU对象
    modbus_ctx_ = std::make_unique<modbus::ModbusRTU>(
      device_port_, baudrate_, 'N', 8, 1);
    
    // 设置从站ID和超时
    modbus_ctx_->set_slave(slave_id_);
    modbus_ctx_->set_response_timeout(0, 500000); // 500ms
    
    // 连接
    modbus_ctx_->connect();
    
    RCLCPP_INFO(this->get_logger(), "Successfully connected to gripper");
    is_connected_ = true;
    return true;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus connection failed: %s", e.what());
    modbus_ctx_.reset();
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during connection: %s", e.what());
    modbus_ctx_.reset();
    return false;
  }
}

void GripperControlNode::disconnect_modbus()
{
  if (modbus_ctx_) {
    try {
      modbus_ctx_->close();
    } catch (...) {
      // Ignore close errors
    }
    modbus_ctx_.reset();
    is_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "Disconnected from gripper");
  }
}

void GripperControlNode::timer_callback()
{
  if (!is_connected_) {
    return;
  }
  
  if (read_gripper_state()) {
    current_state_.header.stamp = this->now();
    current_state_.communication_ok = true;
    state_pub_->publish(current_state_);
  } else {
    current_state_.communication_ok = false;
  }
}

bool GripperControlNode::read_gripper_state()
{
  if (!modbus_ctx_) return false;
  
  try {
    // 读取3个寄存器：0x07D0, 0x07D1, 0x07D2
    std::vector<uint16_t> read_buf;
    
    try {
      // 尝试读取输入寄存器
      read_buf = modbus_ctx_->read_input_registers(REG_STATUS, 3);
    } catch (const modbus::Exception &) {
      // 回退到保持寄存器
      read_buf = modbus_ctx_->read_holding_registers(REG_STATUS, 3);
    }
    
    // 解析 0x07D0 (状态寄存器)
    uint16_t status_reg = read_buf[0];
    current_state_.is_activated = (status_reg & 0x01) == 0x01;
    current_state_.object_status = (status_reg >> 6) & 0x03;
    current_state_.is_moving = (current_state_.object_status == 0) && 
                               ((status_reg >> 3) & 0x01);
    
    // 解析 0x07D1 (故障码与位置)
    uint16_t fault_pos_reg = read_buf[1];
    current_state_.fault_code = fault_pos_reg & 0x00FF;
    current_state_.current_position = (fault_pos_reg >> 8) & 0xFF;
    current_state_.fault_message = get_fault_message(current_state_.fault_code);
    
    // 解析 0x07D2 (力反馈与速度)
    uint16_t force_speed_reg = read_buf[2];
    current_state_.current_force = (force_speed_reg >> 8) & 0xFF;
    current_state_.current_speed = force_speed_reg & 0xFF;
    
    is_activated_ = current_state_.is_activated;
    
    return true;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read gripper state: %s", e.what());
    return false;
  }
}

bool GripperControlNode::activate_gripper()
{
  if (!modbus_ctx_) return false;
  
  try {
    RCLCPP_INFO(this->get_logger(), "Activating gripper...");
    
    // 步骤 1: 复位
    modbus_ctx_->write_register(REG_CONTROL, 0x0000);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 步骤 2: 激活
    modbus_ctx_->write_register(REG_CONTROL, 0x0001);
    
    // 步骤 3: 等待激活完成
    for (int i = 0; i < 50; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (read_gripper_state() && current_state_.is_activated) {
        RCLCPP_INFO(this->get_logger(), "Gripper activated successfully");
        return true;
      }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Gripper activation timeout");
    return false;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate gripper: %s", e.what());
    return false;
  }
}

bool GripperControlNode::move_gripper(uint8_t position, uint8_t speed, uint8_t force)
{
  if (!modbus_ctx_ || !is_activated_) {
    RCLCPP_ERROR(this->get_logger(), "Gripper not activated");
    return false;
  }
  
  try {
    // 限制数值范围
    position = std::min(position, (uint8_t)255);
    speed = std::min(speed, (uint8_t)255);
    force = std::min(force, (uint8_t)255);
    
    std::vector<uint16_t> values(3);
    values[0] = 0x0009; // Enable + Go
    values[1] = (uint16_t)(position << 8);
    values[2] = (uint16_t)((force << 8) | speed);
    
    modbus_ctx_->write_registers(REG_CONTROL, values);
    
    // 更新目标值
    current_state_.target_position = position;
    current_state_.target_speed = speed;
    current_state_.target_force = force;
    
    return true;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send move command: %s", e.what());
    return false;
  }
}

void GripperControlNode::handle_activate(
  const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr,
  qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response)
{
  response->success = activate_gripper();
  response->message = response->success ? "Gripper activated" : "Activation failed";
}

void GripperControlNode::handle_move(
  const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
  qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response)
{
  response->success = move_gripper(request->position, request->speed, request->force);
  response->message = response->success ? "Move command sent" : "Move command failed";
}

void GripperControlNode::handle_get_state(
  const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr,
  qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response)
{
  response->success = read_gripper_state();
  response->state = current_state_;
  response->message = response->success ? "State retrieved" : "Failed to read state";
}

std::string GripperControlNode::get_fault_message(uint8_t fault_code)
{
  if (fault_code == 0) return "No fault";
  
  std::string msg = "Fault: ";
  if (fault_code & 0x01) msg += "Motor not activated; ";
  if (fault_code & 0x08) msg += "Overcurrent/stall; ";
  if (fault_code & 0x10) msg += "Voltage abnormal; ";
  if (fault_code & 0x40) msg += "Overtemperature; ";
  
  return msg;
}

}  // namespace qyh_gripper_control
