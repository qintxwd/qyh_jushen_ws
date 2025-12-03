/*
 * 升降电机控制节点实现
 */

#include "qyh_lift_control/lift_control_node.hpp"
#include <chrono>
#include <cstring>

namespace qyh_lift_control
{

LiftControlNode::LiftControlNode(const rclcpp::NodeOptions & options)
: Node("lift_control_node", options),
  modbus_ctx_(nullptr),
  is_connected_(false),
  is_enabled_(false)
{
  // 声明参数
  this->declare_parameter<std::string>("plc_ip", "192.168.1.88");
  this->declare_parameter<int>("plc_port", 502);
  this->declare_parameter<double>("publish_rate", 10.0);

  // 获取参数
  plc_ip_ = this->get_parameter("plc_ip").as_string();
  plc_port_ = this->get_parameter("plc_port").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Starting Lift Control Node");
  RCLCPP_INFO(this->get_logger(), "PLC IP: %s, Port: %d", plc_ip_.c_str(), plc_port_);

  // 连接 Modbus TCP
  if (!connect_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC");
  }

  // 创建发布者
  state_pub_ = this->create_publisher<qyh_lift_msgs::msg::LiftState>(
    "lift_state", 10);

  // 创建服务
  control_srv_ = this->create_service<qyh_lift_msgs::srv::LiftControl>(
    "lift_control",
    std::bind(&LiftControlNode::handle_control, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建定时器 - 状态发布
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&LiftControlNode::timer_callback, this));

  // 创建重连定时器 - 10秒间隔
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&LiftControlNode::reconnect_callback, this));

  RCLCPP_INFO(this->get_logger(), "Lift Control Node started");
}

LiftControlNode::~LiftControlNode()
{
  disconnect_modbus();
}

bool LiftControlNode::connect_modbus()
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

void LiftControlNode::disconnect_modbus()
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

void LiftControlNode::timer_callback()
{
  // 未连接时不读取状态，由重连定时器处理
  if (!is_connected_) {
    current_state_.connected = false;
    current_state_.header.stamp = this->now();
    state_pub_->publish(current_state_);
    return;
  }

  if (read_lift_state()) {
    current_state_.header.stamp = this->now();
    current_state_.connected = true;
    state_pub_->publish(current_state_);
  } else {
    current_state_.connected = false;
    state_pub_->publish(current_state_);
  }
}

void LiftControlNode::reconnect_callback()
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

bool LiftControlNode::read_lift_state()
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

    // 读取寄存器 - 当前位置 (Register 40, 2个寄存器组成的float)
    // Float格式: Little-endian byte swap (CDAB)
    auto regs = modbus_ctx_->read_holding_registers(
      ModbusAddress::REG_BASE + ModbusAddress::REG_CURRENT_POSITION, 2);

    // Little-endian byte swap: 寄存器顺序 [低16位, 高16位]
    uint32_t raw = (static_cast<uint32_t>(regs[1]) << 16) | regs[0];
    float position;
    std::memcpy(&position, &raw, sizeof(float));
    current_state_.current_position = position;

    // 读取速度 (Register 20)
    auto speed_regs = modbus_ctx_->read_holding_registers(
      ModbusAddress::REG_BASE + ModbusAddress::REG_SPEED, 1);
    current_state_.current_speed = static_cast<float>(speed_regs[0]);

    return true;

  } catch (const modbus::Exception & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read lift state: %s", e.what());
    is_connected_ = false;
    return false;
  }
}

bool LiftControlNode::write_coil(int offset, bool value)
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

bool LiftControlNode::read_coil(int offset)
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

bool LiftControlNode::write_register(int offset, uint16_t value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    modbus_ctx_->write_register(ModbusAddress::REG_BASE + offset, value);
    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write register %d: %s", offset, e.what());
    return false;
  }
}

bool LiftControlNode::write_register_float(int offset, float value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_) {
    return false;
  }

  try {
    // 将float转换为两个16位寄存器 (大端序)
    uint32_t raw;
    std::memcpy(&raw, &value, sizeof(float));
    std::vector<uint16_t> regs(2);
    regs[0] = static_cast<uint16_t>(raw >> 16);
    regs[1] = static_cast<uint16_t>(raw & 0xFFFF);

    modbus_ctx_->write_registers(ModbusAddress::REG_BASE + offset, regs);
    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write float register %d: %s", offset, e.what());
    return false;
  }
}

bool LiftControlNode::enable_lift(bool enable)
{
  RCLCPP_INFO(this->get_logger(), "%s lift", enable ? "Enabling" : "Disabling");
  return write_coil(ModbusAddress::COIL_ENABLE, enable);
}

bool LiftControlNode::set_speed(float speed)
{
  RCLCPP_INFO(this->get_logger(), "Setting speed to %.1f", speed);
  // 速度是单个16位寄存器
  return write_register(ModbusAddress::REG_SPEED, static_cast<uint16_t>(speed));
}

bool LiftControlNode::go_to_position(float position)
{
  if (!is_enabled_) {
    RCLCPP_WARN(this->get_logger(), "Lift is not enabled");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Going to position %.2f", position);

  // 1. 写入目标位置 (float, Little-endian byte swap格式)
  {
    std::lock_guard<std::mutex> lock(modbus_mutex_);
    if (!modbus_ctx_) return false;

    try {
      uint32_t raw;
      std::memcpy(&raw, &position, sizeof(float));
      // Little-endian byte swap: [低16位, 高16位]
      std::vector<uint16_t> regs(2);
      regs[0] = static_cast<uint16_t>(raw & 0xFFFF);
      regs[1] = static_cast<uint16_t>(raw >> 16);
      modbus_ctx_->write_registers(
        ModbusAddress::REG_BASE + ModbusAddress::REG_TARGET_POSITION, regs);
    } catch (const modbus::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write target position: %s", e.what());
      return false;
    }
  }

  // 2. 触发绝对位置GO
  return write_coil(ModbusAddress::COIL_GO_POSITION, true);
}

bool LiftControlNode::manual_move(bool up, bool hold)
{
  if (!is_enabled_ && hold) {
    RCLCPP_WARN(this->get_logger(), "Lift is not enabled");
    return false;
  }

  int coil = up ? ModbusAddress::COIL_MOVE_UP : ModbusAddress::COIL_MOVE_DOWN;
  RCLCPP_INFO(this->get_logger(), "Manual %s: %s",
              up ? "UP" : "DOWN", hold ? "START" : "STOP");

  // 如果停止，确保两个方向都停止
  if (!hold) {
    write_coil(ModbusAddress::COIL_MOVE_UP, false);
    write_coil(ModbusAddress::COIL_MOVE_DOWN, false);
    return true;
  }

  return write_coil(coil, hold);
}

bool LiftControlNode::reset_alarm()
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

bool LiftControlNode::stop_move()
{
  RCLCPP_INFO(this->get_logger(), "Stopping movement");

  // 写入 COIL_GO_POSITION = 0 来取消当前的绝对位置移动
  return write_coil(ModbusAddress::COIL_GO_POSITION, false);
}

void LiftControlNode::handle_control(
  const qyh_lift_msgs::srv::LiftControl::Request::SharedPtr request,
  qyh_lift_msgs::srv::LiftControl::Response::SharedPtr response)
{
  using LiftControl = qyh_lift_msgs::srv::LiftControl;

  switch (request->command) {
    case LiftControl::Request::CMD_ENABLE:
      response->success = enable_lift(true);
      response->message = response->success ? "Lift enabled" : "Failed to enable lift";
      break;

    case LiftControl::Request::CMD_DISABLE:
      response->success = enable_lift(false);
      response->message = response->success ? "Lift disabled" : "Failed to disable lift";
      break;

    case LiftControl::Request::CMD_SET_SPEED:
      response->success = set_speed(request->value);
      response->message = response->success ?
        "Speed set to " + std::to_string(request->value) : "Failed to set speed";
      break;

    case LiftControl::Request::CMD_GO_POSITION:
      response->success = go_to_position(request->value);
      response->message = response->success ?
        "Moving to position " + std::to_string(request->value) : "Failed to move";
      break;

    case LiftControl::Request::CMD_MOVE_UP:
      response->success = manual_move(true, request->hold);
      response->message = response->success ?
        (request->hold ? "Moving up" : "Stopped") : "Failed";
      break;

    case LiftControl::Request::CMD_MOVE_DOWN:
      response->success = manual_move(false, request->hold);
      response->message = response->success ?
        (request->hold ? "Moving down" : "Stopped") : "Failed";
      break;

    case LiftControl::Request::CMD_RESET_ALARM:
      response->success = reset_alarm();
      response->message = response->success ? "Alarm reset" : "Failed to reset alarm";
      break;

    case LiftControl::Request::CMD_STOP:
      response->success = stop_move();
      response->message = response->success ? "Movement stopped" : "Failed to stop movement";
      break;

    default:
      response->success = false;
      response->message = "Unknown command: " + std::to_string(request->command);
      break;
  }
}

}  // namespace qyh_lift_control
