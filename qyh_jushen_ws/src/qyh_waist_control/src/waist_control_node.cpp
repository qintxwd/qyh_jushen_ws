/*
 * 腰部电机控制节点实现
 * 
 * 位置范围: 163711(前倾45°) ~ 230715(竖直0°)
 * 位置和速度都是 int32 类型
 */

#include "qyh_waist_control/waist_control_node.hpp"
#include <chrono>
#include <cstring>
#include <algorithm>

namespace qyh_waist_control
{

WaistControlNode::WaistControlNode(const rclcpp::NodeOptions & options)
: Node("waist_control_node", options),
  modbus_ctx_(nullptr),
  is_connected_(false),
  is_enabled_(false)
{
  // 声明参数
  this->declare_parameter<std::string>("plc_ip", "192.168.1.88");
  this->declare_parameter<int>("plc_port", 502);
  this->declare_parameter<double>("publish_rate", 10.0);
  this->declare_parameter<int>("position_upright", PositionConstants::UPRIGHT);
  this->declare_parameter<int>("position_max_lean", PositionConstants::MAX_LEAN);
  this->declare_parameter<double>("max_angle", PositionConstants::MAX_ANGLE);

  // 获取参数
  plc_ip_ = this->get_parameter("plc_ip").as_string();
  plc_port_ = this->get_parameter("plc_port").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  position_upright_ = this->get_parameter("position_upright").as_int();
  position_max_lean_ = this->get_parameter("position_max_lean").as_int();
  max_angle_ = static_cast<float>(this->get_parameter("max_angle").as_double());

  RCLCPP_INFO(this->get_logger(), "Starting Waist Control Node");
  RCLCPP_INFO(this->get_logger(), "PLC IP: %s, Port: %d", plc_ip_.c_str(), plc_port_);
  RCLCPP_INFO(this->get_logger(), "Position range: %d (upright) ~ %d (max lean %.1f deg)",
              position_upright_, position_max_lean_, max_angle_);

  // 连接 Modbus TCP
  if (!connect_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC");
  }

  // 创建发布者
  state_pub_ = this->create_publisher<qyh_waist_msgs::msg::WaistState>(
    "waist_state", 10);

  // 创建服务
  control_srv_ = this->create_service<qyh_waist_msgs::srv::WaistControl>(
    "waist_control",
    std::bind(&WaistControlNode::handle_control, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建定时器 - 状态发布
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&WaistControlNode::timer_callback, this));

  // 创建重连定时器 - 10秒间隔
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&WaistControlNode::reconnect_callback, this));

  RCLCPP_INFO(this->get_logger(), "Waist Control Node started");
}

WaistControlNode::~WaistControlNode()
{
  disconnect_modbus();
}

bool WaistControlNode::connect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  try {
    modbus_ctx_ = std::make_unique<modbus::ModbusTCP>(plc_ip_, plc_port_);
    modbus_ctx_->set_response_timeout(1, 0);  // 1秒超时
    modbus_ctx_->connect();

    RCLCPP_INFO(this->get_logger(), "Successfully connected to PLC at %s:%d",
                plc_ip_.c_str(), plc_port_);
    is_connected_ = true;
    return true;

  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus connection failed: %s", e.what());
    modbus_ctx_.reset();
    is_connected_ = false;
    return false;
  }
}

void WaistControlNode::disconnect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (modbus_ctx_) {
    try {
      modbus_ctx_->close();
    } catch (...) {
      // 忽略关闭错误
    }
    modbus_ctx_.reset();
    is_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "Disconnected from PLC");
  }
}

void WaistControlNode::timer_callback()
{
  // 未连接时不读取状态，由重连定时器处理
  if (!is_connected_) {
    current_state_.connected = false;
    current_state_.header.stamp = this->now();
    state_pub_->publish(current_state_);
    return;
  }

  if (read_waist_state()) {
    current_state_.header.stamp = this->now();
    current_state_.connected = true;
    state_pub_->publish(current_state_);
  } else {
    current_state_.connected = false;
    state_pub_->publish(current_state_);
  }
}

void WaistControlNode::reconnect_callback()
{
  // 已连接时不需要重连
  if (is_connected_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to PLC...");
  if (connect_modbus()) {
    RCLCPP_INFO(this->get_logger(), "Reconnected to PLC successfully");
  } else {
    RCLCPP_WARN(this->get_logger(), "Reconnection failed, will retry in 10 seconds");
  }
}

bool WaistControlNode::read_waist_state()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    // 读取线圈状态 (21个线圈: 0-20)
    auto coils = modbus_ctx_->read_coils(
      ModbusAddress::COIL_BASE, 21);

    current_state_.enabled = (coils[ModbusAddress::COIL_ENABLE] != 0);
    current_state_.alarm = (coils[ModbusAddress::COIL_ALARM] != 0);
    current_state_.position_reached = (coils[ModbusAddress::COIL_POSITION_REACHED] != 0);

    is_enabled_ = current_state_.enabled;

    // 读取寄存器 - 当前位置 (int32, 2个寄存器)
    auto regs = modbus_ctx_->read_holding_registers(
      ModbusAddress::REG_BASE + ModbusAddress::REG_CURRENT_POSITION, 2);

    // int32: 低16位在前，高16位在后 (Little-endian)
    int32_t position = static_cast<int32_t>(
      (static_cast<uint32_t>(regs[1]) << 16) | regs[0]);
    current_state_.current_position = position;
    current_state_.current_angle = position_to_angle(position);

    // 读取速度 (int32, 2个寄存器)
    auto speed_regs = modbus_ctx_->read_holding_registers(
      ModbusAddress::REG_BASE + ModbusAddress::REG_SPEED, 2);
    current_state_.current_speed = static_cast<int32_t>(
      (static_cast<uint32_t>(speed_regs[1]) << 16) | speed_regs[0]);

    return true;

  } catch (const modbus::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read waist state: %s", e.what());
    is_connected_ = false;
    return false;
  }
}

bool WaistControlNode::write_coil(int offset, bool value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    modbus_ctx_->write_coil(ModbusAddress::COIL_BASE + offset, value);
    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write coil %d: %s", offset, e.what());
    return false;
  }
}

bool WaistControlNode::read_coil(int offset)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    auto coils = modbus_ctx_->read_coils(ModbusAddress::COIL_BASE + offset, 1);
    return coils[0] != 0;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read coil %d: %s", offset, e.what());
    return false;
  }
}

bool WaistControlNode::write_register_int32(int offset, int32_t value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    // 将 int32 转换为两个16位寄存器 (Little-endian: 低16位在前)
    uint32_t raw = static_cast<uint32_t>(value);
    std::vector<uint16_t> regs(2);
    regs[0] = static_cast<uint16_t>(raw & 0xFFFF);        // 低16位
    regs[1] = static_cast<uint16_t>((raw >> 16) & 0xFFFF); // 高16位

    modbus_ctx_->write_registers(ModbusAddress::REG_BASE + offset, regs);
    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write int32 register %d: %s", offset, e.what());
    return false;
  }
}

int32_t WaistControlNode::read_register_int32(int offset)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return 0;
  }

  try {
    auto regs = modbus_ctx_->read_holding_registers(ModbusAddress::REG_BASE + offset, 2);
    // Little-endian: 低16位在前
    return static_cast<int32_t>((static_cast<uint32_t>(regs[1]) << 16) | regs[0]);
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read int32 register %d: %s", offset, e.what());
    return 0;
  }
}

int32_t WaistControlNode::angle_to_position(float angle)
{
  // 角度范围: 0 (竖直) ~ max_angle_ (最大前倾)
  // 位置范围: position_upright_ (竖直) ~ position_max_lean_ (最大前倾)
  // 
  // 线性映射:
  // angle=0 -> position_upright_
  // angle=max_angle_ -> position_max_lean_

  // 限制角度范围
  angle = std::max(0.0f, std::min(angle, max_angle_));

  // 线性插值
  float ratio = angle / max_angle_;
  int32_t position = position_upright_ + 
    static_cast<int32_t>(ratio * (position_max_lean_ - position_upright_));

  return position;
}

float WaistControlNode::position_to_angle(int32_t position)
{
  // 反向映射
  // position_upright_ -> 0
  // position_max_lean_ -> max_angle_

  if (position_max_lean_ == position_upright_) {
    return 0.0f;
  }

  float ratio = static_cast<float>(position - position_upright_) / 
                static_cast<float>(position_max_lean_ - position_upright_);
  
  // 限制范围
  ratio = std::max(0.0f, std::min(ratio, 1.0f));
  
  return ratio * max_angle_;
}

bool WaistControlNode::enable_waist(bool enable)
{
  RCLCPP_INFO(this->get_logger(), "%s waist", enable ? "Enabling" : "Disabling");
  return write_coil(ModbusAddress::COIL_ENABLE, enable);
}

bool WaistControlNode::set_speed(int32_t speed)
{
  RCLCPP_INFO(this->get_logger(), "Setting speed to %d", speed);
  return write_register_int32(ModbusAddress::REG_SPEED, speed);
}

bool WaistControlNode::go_to_position(int32_t position)
{
  if (!is_enabled_) {
    RCLCPP_WARN(this->get_logger(), "Waist is not enabled");
    return false;
  }

  // 限制位置范围 (注意: position_max_lean_ < position_upright_)
  int32_t min_pos = std::min(position_upright_, position_max_lean_);
  int32_t max_pos = std::max(position_upright_, position_max_lean_);
  position = std::max(min_pos, std::min(position, max_pos));

  RCLCPP_INFO(this->get_logger(), "Going to position %d (angle: %.1f deg)", 
              position, position_to_angle(position));

  // 1. 写入目标位置 (int32)
  if (!write_register_int32(ModbusAddress::REG_TARGET_POSITION, position)) {
    return false;
  }

  // 2. 触发绝对位置GO
  return write_coil(ModbusAddress::COIL_GO_POSITION, true);
}

bool WaistControlNode::go_to_angle(float angle)
{
  // 限制角度范围: 0 ~ max_angle_
  if (angle < 0.0f) {
    RCLCPP_WARN(this->get_logger(), "Cannot lean backward, angle must be >= 0");
    angle = 0.0f;
  }
  if (angle > max_angle_) {
    RCLCPP_WARN(this->get_logger(), "Angle %.1f exceeds max %.1f, limiting", angle, max_angle_);
    angle = max_angle_;
  }

  int32_t position = angle_to_position(angle);
  RCLCPP_INFO(this->get_logger(), "Going to angle %.1f deg (position: %d)", angle, position);
  
  return go_to_position(position);
}

bool WaistControlNode::manual_lean(bool forward, bool hold)
{
  if (!is_enabled_ && hold) {
    RCLCPP_WARN(this->get_logger(), "Waist is not enabled");
    return false;
  }

  int coil = forward ? ModbusAddress::COIL_LEAN_FORWARD : ModbusAddress::COIL_LEAN_BACK;
  RCLCPP_INFO(this->get_logger(), "Manual %s: %s",
              forward ? "LEAN FORWARD" : "LEAN BACK", hold ? "START" : "STOP");

  // 如果停止，确保两个方向都停止
  if (!hold) {
    write_coil(ModbusAddress::COIL_LEAN_FORWARD, false);
    write_coil(ModbusAddress::COIL_LEAN_BACK, false);
    return true;
  }

  return write_coil(coil, hold);
}

bool WaistControlNode::reset_alarm()
{
  RCLCPP_INFO(this->get_logger(), "Resetting alarm");

  // 触发复位信号
  if (!write_coil(ModbusAddress::COIL_RESET, true)) {
    return false;
  }

  // 短暂延时后清除复位信号
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return write_coil(ModbusAddress::COIL_RESET, false);
}

bool WaistControlNode::stop_move()
{
  RCLCPP_INFO(this->get_logger(), "Stopping movement");

  // 停止手动控制
  write_coil(ModbusAddress::COIL_LEAN_FORWARD, false);
  write_coil(ModbusAddress::COIL_LEAN_BACK, false);

  // 取消绝对位置移动
  return write_coil(ModbusAddress::COIL_GO_POSITION, false);
}

bool WaistControlNode::go_upright()
{
  RCLCPP_INFO(this->get_logger(), "Going to upright position");
  return go_to_position(position_upright_);
}

void WaistControlNode::handle_control(
  const qyh_waist_msgs::srv::WaistControl::Request::SharedPtr request,
  qyh_waist_msgs::srv::WaistControl::Response::SharedPtr response)
{
  using WaistControl = qyh_waist_msgs::srv::WaistControl;

  switch (request->command) {
    case WaistControl::Request::CMD_ENABLE:
      response->success = enable_waist(true);
      response->message = response->success ? "Waist enabled" : "Failed to enable waist";
      break;

    case WaistControl::Request::CMD_DISABLE:
      response->success = enable_waist(false);
      response->message = response->success ? "Waist disabled" : "Failed to disable waist";
      break;

    case WaistControl::Request::CMD_SET_SPEED:
      response->success = set_speed(static_cast<int32_t>(request->value));
      response->message = response->success ?
        "Speed set to " + std::to_string(static_cast<int32_t>(request->value)) : "Failed to set speed";
      break;

    case WaistControl::Request::CMD_GO_POSITION:
      response->success = go_to_position(static_cast<int32_t>(request->value));
      response->message = response->success ?
        "Moving to position " + std::to_string(static_cast<int32_t>(request->value)) : "Failed to move";
      break;

    case WaistControl::Request::CMD_GO_ANGLE:
      response->success = go_to_angle(request->value);
      response->message = response->success ?
        "Moving to angle " + std::to_string(request->value) + " deg" : "Failed to move";
      break;

    case WaistControl::Request::CMD_LEAN_FORWARD:
      response->success = manual_lean(true, request->hold);
      response->message = response->success ?
        (request->hold ? "Leaning forward" : "Stopped") : "Failed";
      break;

    case WaistControl::Request::CMD_LEAN_BACK:
      response->success = manual_lean(false, request->hold);
      response->message = response->success ?
        (request->hold ? "Leaning back" : "Stopped") : "Failed";
      break;

    case WaistControl::Request::CMD_RESET_ALARM:
      response->success = reset_alarm();
      response->message = response->success ? "Alarm reset" : "Failed to reset alarm";
      break;

    case WaistControl::Request::CMD_STOP:
      response->success = stop_move();
      response->message = response->success ? "Movement stopped" : "Failed to stop movement";
      break;

    case WaistControl::Request::CMD_GO_UPRIGHT:
      response->success = go_upright();
      response->message = response->success ? "Moving to upright position" : "Failed to move";
      break;

    default:
      response->success = false;
      response->message = "Unknown command: " + std::to_string(request->command);
      break;
  }
}

}  // namespace qyh_waist_control
