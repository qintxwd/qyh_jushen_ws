#include "gripper_control_node.hpp"
#include <chrono>
#include <algorithm>
#include <thread>
#include <sstream>

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
  blink_color_index_(0),
  is_blinking_(false),
  left_is_activated_(false),
  right_is_activated_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("device_port", "/dev/ttyUSB_gripper");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<int>("left_slave_id", 2);
  this->declare_parameter<int>("right_slave_id", 1);
  this->declare_parameter<double>("publish_rate", 20.0);
  this->declare_parameter<bool>("auto_activate", true);
  this->declare_parameter<bool>("led_enabled", true);
  this->declare_parameter<int>("led_slave_id", 3);
  this->declare_parameter<std::string>("led_topic", "/robot_led/set_color");
  this->declare_parameter<int>("led_default_r", 0);
  this->declare_parameter<int>("led_default_g", 255);
  this->declare_parameter<int>("led_default_b", 0);
  this->declare_parameter<int>("led_default_w", 0);
  this->declare_parameter<std::string>("led_blink_topic", "/robot_led/blink");
  
  // Get parameters
  device_port_ = this->get_parameter("device_port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  left_slave_id_ = this->get_parameter("left_slave_id").as_int();
  right_slave_id_ = this->get_parameter("right_slave_id").as_int();
  double publish_rate = this->get_parameter("publish_rate").as_double();
  auto_activate_ = this->get_parameter("auto_activate").as_bool();
  led_enabled_ = this->get_parameter("led_enabled").as_bool();
  led_slave_id_ = this->get_parameter("led_slave_id").as_int();
  led_topic_ = this->get_parameter("led_topic").as_string();
  led_default_r_ = static_cast<uint8_t>(std::clamp(static_cast<int>(this->get_parameter("led_default_r").as_int()), 0, 255));
  led_default_g_ = static_cast<uint8_t>(std::clamp(static_cast<int>(this->get_parameter("led_default_g").as_int()), 0, 255));
  led_default_b_ = static_cast<uint8_t>(std::clamp(static_cast<int>(this->get_parameter("led_default_b").as_int()), 0, 255));
  led_default_w_ = static_cast<uint8_t>(std::clamp(static_cast<int>(this->get_parameter("led_default_w").as_int()), 0, 255));
  led_blink_topic_ = this->get_parameter("led_blink_topic").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Starting Gripper Control Node");
  RCLCPP_INFO(this->get_logger(), "Device: %s, Baudrate: %d, Left Slave ID: %d, Right Slave ID: %d",
              device_port_.c_str(), baudrate_, left_slave_id_, right_slave_id_);
  if (led_enabled_) {
    RCLCPP_INFO(this->get_logger(), "LED enabled on Modbus slave ID: %d, topic: %s",
                led_slave_id_, led_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "LED default color: R=%d, G=%d, B=%d, W=%d",
                led_default_r_, led_default_g_, led_default_b_, led_default_w_);
  }
  
  // Connect to gripper
  if (!connect_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to gripper");
  }
  
  // Create publisher
  left_state_pub_ = this->create_publisher<qyh_gripper_msgs::msg::GripperState>(
    "/left/gripper_state", 10);
  
  right_state_pub_ = this->create_publisher<qyh_gripper_msgs::msg::GripperState>(
    "/right/gripper_state", 10);
  
  // Create services
  left_srv_activate_ = this->create_service<qyh_gripper_msgs::srv::ActivateGripper>(
    "/left/activate_gripper",
    std::bind(&GripperControlNode::handle_activate_left, this,
              std::placeholders::_1, std::placeholders::_2));

  right_srv_activate_ = this->create_service<qyh_gripper_msgs::srv::ActivateGripper>(
    "/right/activate_gripper",
    std::bind(&GripperControlNode::handle_activate_right, this,
              std::placeholders::_1, std::placeholders::_2));
              
  left_srv_move_ = this->create_service<qyh_gripper_msgs::srv::MoveGripper>(
    "/left/move_gripper",
    std::bind(&GripperControlNode::handle_move_left, this,
              std::placeholders::_1, std::placeholders::_2));

  right_srv_move_ = this->create_service<qyh_gripper_msgs::srv::MoveGripper>(
    "/right/move_gripper",
    std::bind(&GripperControlNode::handle_move_right, this,
              std::placeholders::_1, std::placeholders::_2));
              
  left_srv_get_state_ = this->create_service<qyh_gripper_msgs::srv::GetGripperState>(
    "/left/get_gripper_state",
    std::bind(&GripperControlNode::handle_get_state_left, this,
              std::placeholders::_1, std::placeholders::_2));
  
  right_srv_get_state_ = this->create_service<qyh_gripper_msgs::srv::GetGripperState>(
    "/right/get_gripper_state",
    std::bind(&GripperControlNode::handle_get_state_right, this,
              std::placeholders::_1, std::placeholders::_2));

  // LED color subscriber (shared 485 bus)
  if (led_enabled_) {
    led_color_sub_ = this->create_subscription<std_msgs::msg::ColorRGBA>(
      led_topic_, 10,
      [this](const std_msgs::msg::ColorRGBA::SharedPtr msg) {
        // 设置纯色时停止闪烁
        stop_blinking();
        const int r_i = std::clamp(static_cast<int>(msg->r * 255.0f), 0, 255);
        const int g_i = std::clamp(static_cast<int>(msg->g * 255.0f), 0, 255);
        const int b_i = std::clamp(static_cast<int>(msg->b * 255.0f), 0, 255);
        const int w_i = std::clamp(static_cast<int>(msg->a * 255.0f), 0, 255);
        RCLCPP_INFO(this->get_logger(), "LED set to %d,%d,%d,%d",r_i,g_i,b_i,w_i);
        send_led_color(static_cast<uint8_t>(r_i), static_cast<uint8_t>(g_i),
                       static_cast<uint8_t>(b_i), static_cast<uint8_t>(w_i));
      }
    );
    
    // LED闪烁订阅器
    led_blink_sub_ = this->create_subscription<std_msgs::msg::String>(
      led_blink_topic_, 10,
      std::bind(&GripperControlNode::handle_led_blink, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "LED blink topic: %s", led_blink_topic_.c_str());
  }
  
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
    // 创建C++ ModbusRTU对象并连接（保护 modbus_ctx_）
    {
      std::lock_guard<std::mutex> lock(modbus_mutex_);
      modbus_ctx_ = std::make_unique<modbus::ModbusRTU>(
        device_port_, baudrate_, 'N', 8, 1);
      modbus_ctx_->set_response_timeout(0, 500000); // 500ms
      modbus_ctx_->connect();
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to gripper");
    is_connected_ = true;

    // 自动激活（如果配置启用）——分别激活左右两个从站
    if (auto_activate_) {
      RCLCPP_INFO(this->get_logger(), "Auto-activation enabled, activating both grippers...");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (activate_gripper(true)) {
        RCLCPP_INFO(this->get_logger(), "Left gripper auto-activated");
      } else {
        RCLCPP_WARN(this->get_logger(), "Left gripper auto-activation failed");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (activate_gripper(false)) {
        RCLCPP_INFO(this->get_logger(), "Right gripper auto-activated");
      } else {
        RCLCPP_WARN(this->get_logger(), "Right gripper auto-activation failed");
      }
    }

    // 设置默认LED颜色
    if (led_enabled_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (send_led_color(led_default_r_, led_default_g_, led_default_b_, led_default_w_)) {
        RCLCPP_INFO(this->get_logger(), "LED set to default color: R=%d, G=%d, B=%d, W=%d",
                    led_default_r_, led_default_g_, led_default_b_, led_default_w_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set LED default color");
      }
    }

    return true;

  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus connection failed: %s", e.what());
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    modbus_ctx_.reset();
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during connection: %s", e.what());
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    modbus_ctx_.reset();
    return false;
  }
}

void GripperControlNode::disconnect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
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

  if( read_gripper_state(true) ) {
    left_current_state_.header.stamp = this->now();
    left_current_state_.communication_ok = true;
    left_state_pub_->publish(left_current_state_);
  } else {
    left_current_state_.communication_ok = false;
    left_state_pub_->publish(left_current_state_);
  }

  if( read_gripper_state(false) ) {
    right_current_state_.header.stamp = this->now();
    right_current_state_.communication_ok = true;
    right_state_pub_->publish(right_current_state_);
  } else {
    right_current_state_.communication_ok = false;
    right_state_pub_->publish(right_current_state_);
  }
}

bool GripperControlNode::read_gripper_state(bool left)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!modbus_ctx_) return false;

  // 选择从站
  modbus_ctx_->set_slave(left ? left_slave_id_ : right_slave_id_);
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

    qyh_gripper_msgs::msg::GripperState &state = left ? left_current_state_ : right_current_state_;

    // 解析 0x07D0 (状态寄存器)
    uint16_t status_reg = read_buf[0];
    state.is_activated = (status_reg & 0x01) == 0x01;
    state.object_status = (status_reg >> 6) & 0x03;
    state.is_moving = (state.object_status == 0) && ((status_reg >> 3) & 0x01);

    // 解析 0x07D1 (故障码与位置)
    uint16_t fault_pos_reg = read_buf[1];
    state.fault_code = fault_pos_reg & 0x00FF;
    state.current_position = (fault_pos_reg >> 8) & 0xFF;
    state.fault_message = get_fault_message(state.fault_code);

    // 解析 0x07D2 (力反馈与速度)
    uint16_t force_speed_reg = read_buf[2];
    state.current_force = (force_speed_reg >> 8) & 0xFF;
    state.current_speed = force_speed_reg & 0xFF;

    if (left) left_is_activated_ = state.is_activated;
    else right_is_activated_ = state.is_activated;

    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read gripper state: %s", e.what());
    return false;
  }
}

bool GripperControlNode::activate_gripper(bool left)
{
  if (!modbus_ctx_) return false;

  try {
    RCLCPP_INFO(this->get_logger(), "Activating %s gripper...", left ? "left" : "right");

    // 步骤 1: 复位 (仅在写入时短暂加锁)
    {
      std::lock_guard<std::mutex> lock(modbus_mutex_);
      if (!modbus_ctx_) return false;
      modbus_ctx_->set_slave(left ? left_slave_id_ : right_slave_id_);
      modbus_ctx_->write_register(REG_CONTROL, 0x0000);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 步骤 2: 激活 (再次短暂加锁)
    {
      std::lock_guard<std::mutex> lock(modbus_mutex_);
      if (!modbus_ctx_) return false;
      modbus_ctx_->set_slave(left ? left_slave_id_ : right_slave_id_);
      modbus_ctx_->write_register(REG_CONTROL, 0x0001);
    }

    // 步骤 3: 等待激活完成 -- 使用 read_gripper_state()，它会自行加锁
    for (int i = 0; i < 50; i++) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (read_gripper_state(left) && (left ? left_is_activated_ : right_is_activated_)) {
        RCLCPP_INFO(this->get_logger(), "%s gripper activated successfully", left ? "Left" : "Right");
        return true;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "%s gripper activation timeout", left ? "Left" : "Right");
    return false;

  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate gripper: %s", e.what());
    return false;
  }
}

bool GripperControlNode::move_gripper(bool left, uint8_t position, uint8_t speed, uint8_t force)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);
  if (!modbus_ctx_) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not available");
    return false;
  }

  if (left && !left_is_activated_) {
    RCLCPP_ERROR(this->get_logger(), "Left gripper not activated");
    return false;
  }
  if (!left && !right_is_activated_) {
    RCLCPP_ERROR(this->get_logger(), "Right gripper not activated");
    return false;
  }

  // 选择从站
  modbus_ctx_->set_slave(left ? left_slave_id_ : right_slave_id_);

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

    // 更新目标值对应的状态结构
    qyh_gripper_msgs::msg::GripperState &state = left ? left_current_state_ : right_current_state_;
    state.target_position = position;
    state.target_speed = speed;
    state.target_force = force;

    return true;

  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send move command: %s", e.what());
    return false;
  }
}

bool GripperControlNode::send_led_color(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
  constexpr int max_retries = 3;
  constexpr int retry_delay_ms = 50;

  for (int attempt = 0; attempt < max_retries; ++attempt) {
    {
      std::lock_guard<std::mutex> lock(modbus_mutex_);
      if (!modbus_ctx_) {
        RCLCPP_ERROR(this->get_logger(), "Modbus context not available for LED");
        return false;
      }

      try {
        modbus_ctx_->set_slave(led_slave_id_);
        
        // LED 寄存器 0x0000~0x0003: R,G,B,W
        std::vector<uint16_t> values(4);
        values[0] = static_cast<uint16_t>(r);
        values[1] = static_cast<uint16_t>(g);
        values[2] = static_cast<uint16_t>(b);
        values[3] = static_cast<uint16_t>(w);

        modbus_ctx_->write_registers(0x0000, values);
        return true;
      } catch (const modbus::Exception & e) {
        if (attempt < max_retries - 1) {
          RCLCPP_DEBUG(this->get_logger(), "LED command attempt %d failed: %s, retrying...", 
                       attempt + 1, e.what());
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to send LED command after %d attempts: %s", 
                      max_retries, e.what());
        }
      }
    }
    // 释放锁后等待一段时间再重试，让timer有机会完成读取
    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
  }
  return false;
}

void GripperControlNode::handle_activate_left(
  const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr,
  qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response)
{
  response->success = activate_gripper(true);
  response->message = response->success ? "Gripper activated" : "Activation failed";
}
void GripperControlNode::handle_activate_right(
  const qyh_gripper_msgs::srv::ActivateGripper::Request::SharedPtr,
  qyh_gripper_msgs::srv::ActivateGripper::Response::SharedPtr response)
{
  response->success = activate_gripper(false);
  response->message = response->success ? "Gripper activated" : "Activation failed";
}

void GripperControlNode::handle_move_left(
  const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
  qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response)
{
  response->success = move_gripper(true, request->position, request->speed, request->force);
  response->message = response->success ? "Move command sent" : "Move command failed";
}
void GripperControlNode::handle_move_right(
  const qyh_gripper_msgs::srv::MoveGripper::Request::SharedPtr request,
  qyh_gripper_msgs::srv::MoveGripper::Response::SharedPtr response)
{
  response->success = move_gripper(false, request->position, request->speed, request->force);
  response->message = response->success ? "Move command sent" : "Move command failed";
}

void GripperControlNode::handle_get_state_left(
  const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr,
  qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response)
{
  response->success = read_gripper_state(true);
  response->state = left_current_state_;
  response->message = response->success ? "State retrieved" : "Failed to read state";
}
void GripperControlNode::handle_get_state_right(
  const qyh_gripper_msgs::srv::GetGripperState::Request::SharedPtr,
  qyh_gripper_msgs::srv::GetGripperState::Response::SharedPtr response)
{
  response->success = read_gripper_state(false);
  response->state = right_current_state_;
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

// ============ LED闪烁功能实现 ============

bool GripperControlNode::parse_blink_command(const std::string& cmd, 
                                              std::vector<RGBWColor>& colors, 
                                              int& interval_ms)
{
  // 命令格式: "interval_ms:r,g,b,w;r,g,b,w;..."
  // 例如: "500:255,0,0,0;0,255,0,0;0,0,255,0" 表示500ms间隔，红->绿->蓝循环
  // 发送 "stop" 停止闪烁
  
  colors.clear();
  
  if (cmd.empty() || cmd == "stop") {
    return false;  // 停止闪烁
  }
  
  size_t colon_pos = cmd.find(':');
  if (colon_pos == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "Invalid blink command format, missing ':'");
    return false;
  }
  
  // 解析间隔时间
  try {
    interval_ms = std::stoi(cmd.substr(0, colon_pos));
    if (interval_ms < 50) {
      interval_ms = 50;  // 最小50ms
      RCLCPP_WARN(this->get_logger(), "Blink interval too small, set to 50ms");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse interval: %s", e.what());
    return false;
  }
  
  // 解析颜色序列
  std::string colors_str = cmd.substr(colon_pos + 1);
  std::istringstream color_stream(colors_str);
  std::string color_item;
  
  while (std::getline(color_stream, color_item, ';')) {
    if (color_item.empty()) continue;
    
    std::istringstream rgbw_stream(color_item);
    std::string value;
    std::vector<int> rgbw_values;
    
    while (std::getline(rgbw_stream, value, ',')) {
      try {
        int v = std::stoi(value);
        rgbw_values.push_back(std::clamp(v, 0, 255));
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse color value: %s", e.what());
        return false;
      }
    }
    
    // 确保有4个值 (R, G, B, W)
    while (rgbw_values.size() < 4) {
      rgbw_values.push_back(0);
    }
    
    RGBWColor color;
    color.r = static_cast<uint8_t>(rgbw_values[0]);
    color.g = static_cast<uint8_t>(rgbw_values[1]);
    color.b = static_cast<uint8_t>(rgbw_values[2]);
    color.w = static_cast<uint8_t>(rgbw_values[3]);
    colors.push_back(color);
  }
  
  if (colors.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No valid colors in blink command");
    return false;
  }
  
  return true;
}

void GripperControlNode::handle_led_blink(const std_msgs::msg::String::SharedPtr msg)
{
  std::vector<RGBWColor> colors;
  int interval_ms = 500;
  
  if (!parse_blink_command(msg->data, colors, interval_ms)) {
    // 停止闪烁，恢复默认颜色
    stop_blinking();
    send_led_color(led_default_r_, led_default_g_, led_default_b_, led_default_w_);
    RCLCPP_INFO(this->get_logger(), "LED blink stopped, restored to default color");
    return;
  }
  
  // 停止之前的闪烁
  stop_blinking();
  
  // 设置新的闪烁参数
  blink_colors_ = colors;
  blink_color_index_ = 0;
  is_blinking_ = true;
  
  RCLCPP_INFO(this->get_logger(), "LED blink started: %zu colors, %d ms interval", 
              colors.size(), interval_ms);
  
  // 立即显示第一个颜色
  const auto& first_color = blink_colors_[0];
  send_led_color(first_color.r, first_color.g, first_color.b, first_color.w);
  
  // 创建闪烁定时器
  blink_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(interval_ms),
    std::bind(&GripperControlNode::blink_timer_callback, this)
  );
}

void GripperControlNode::blink_timer_callback()
{
  if (!is_blinking_ || blink_colors_.empty()) {
    return;
  }
  
  // 切换到下一个颜色
  blink_color_index_ = (blink_color_index_ + 1) % blink_colors_.size();
  const auto& color = blink_colors_[blink_color_index_];
  send_led_color(color.r, color.g, color.b, color.w);
}

void GripperControlNode::stop_blinking()
{
  is_blinking_ = false;
  if (blink_timer_) {
    blink_timer_->cancel();
    blink_timer_.reset();
  }
  blink_colors_.clear();
  blink_color_index_ = 0;
}

}  // namespace qyh_gripper_control
