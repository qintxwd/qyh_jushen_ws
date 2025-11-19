#include "qyh_standard_robot/standard_robot_node.hpp"
#include <cstring>
#include <cmath>

namespace qyh_standard_robot
{

StandardRobotNode::StandardRobotNode(const rclcpp::NodeOptions & options)
: Node("standard_robot_node", options),
  modbus_ctx_(nullptr),
  is_connected_(false),
  is_manual_control_(false),
  last_cmd_(nullptr)
{
  // Declare parameters
  this->declare_parameter<std::string>("modbus_ip", "192.168.1.100");
  this->declare_parameter<int>("modbus_port", 502);
  this->declare_parameter<int>("slave_id", 1);
  this->declare_parameter<double>("publish_rate", 10.0);
  this->declare_parameter<double>("manual_control_rate", 20.0);
  this->declare_parameter<int>("manual_command_timeout_ms", 200);
  
  // Get parameters
  modbus_ip_ = this->get_parameter("modbus_ip").as_string();
  modbus_port_ = this->get_parameter("modbus_port").as_int();
  slave_id_ = this->get_parameter("slave_id").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  manual_control_rate_ = this->get_parameter("manual_control_rate").as_double();
  manual_command_timeout_ms_ = this->get_parameter("manual_command_timeout_ms").as_int();
  last_cmd_time_ = std::chrono::steady_clock::now();
  
  RCLCPP_INFO(this->get_logger(), "Starting Standard Robot Node");
  RCLCPP_INFO(this->get_logger(), "Modbus IP: %s, Port: %d, Slave ID: %d", 
              modbus_ip_.c_str(), modbus_port_, slave_id_);
  
  // Create publisher
  status_pub_ = this->create_publisher<qyh_standard_robot_msgs::msg::StandardRobotStatus>(
    "standard_robot_status", 10);
  
  // Connect to Modbus
  if (!connect_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Modbus device");
  }
  
  // Create timer
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&StandardRobotNode::timer_callback, this));

  srv_pause_move_ = this->create_service<qyh_standard_robot_msgs::srv::ControlPauseMove>(
    "control_pause_move",
    std::bind(&StandardRobotNode::handle_pause_move, this, std::placeholders::_1, std::placeholders::_2));
  srv_resume_move_ = this->create_service<qyh_standard_robot_msgs::srv::ControlResumeMove>(
    "control_resume_move",
    std::bind(&StandardRobotNode::handle_resume_move, this, std::placeholders::_1, std::placeholders::_2));
  srv_stop_move_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStopMove>(
    "control_stop_move",
    std::bind(&StandardRobotNode::handle_stop_move, this, std::placeholders::_1, std::placeholders::_2));
  srv_stop_localization_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStopLocalization>(
    "control_stop_localization",
    std::bind(&StandardRobotNode::handle_stop_localization, this, std::placeholders::_1, std::placeholders::_2));
  srv_emergency_stop_ = this->create_service<qyh_standard_robot_msgs::srv::ControlEmergencyStop>(
    "control_emergency_stop",
    std::bind(&StandardRobotNode::handle_emergency_stop, this, std::placeholders::_1, std::placeholders::_2));
  srv_release_emergency_stop_ = this->create_service<qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop>(
    "control_release_emergency_stop",
    std::bind(&StandardRobotNode::handle_release_emergency_stop, this, std::placeholders::_1, std::placeholders::_2));
  srv_start_charging_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStartCharging>(
    "control_start_charging",
    std::bind(&StandardRobotNode::handle_start_charging, this, std::placeholders::_1, std::placeholders::_2));
  srv_stop_charging_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStopCharging>(
    "control_stop_charging",
    std::bind(&StandardRobotNode::handle_stop_charging, this, std::placeholders::_1, std::placeholders::_2));
  srv_enter_low_power_ = this->create_service<qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode>(
    "control_enter_low_power_mode",
    std::bind(&StandardRobotNode::handle_enter_low_power, this, std::placeholders::_1, std::placeholders::_2));
  srv_exit_low_power_ = this->create_service<qyh_standard_robot_msgs::srv::ControlExitLowPowerMode>(
    "control_exit_low_power_mode",
    std::bind(&StandardRobotNode::handle_exit_low_power, this, std::placeholders::_1, std::placeholders::_2));
  srv_system_reset_ = this->create_service<qyh_standard_robot_msgs::srv::ControlSystemReset>(
    "control_system_reset",
    std::bind(&StandardRobotNode::handle_system_reset, this, std::placeholders::_1, std::placeholders::_2));
  srv_start_manual_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStartManualControl>(
    "control_start_manual_control",
    std::bind(&StandardRobotNode::handle_start_manual, this, std::placeholders::_1, std::placeholders::_2));
  srv_stop_manual_ = this->create_service<qyh_standard_robot_msgs::srv::ControlStopManualControl>(
    "control_stop_manual_control",
    std::bind(&StandardRobotNode::handle_stop_manual, this, std::placeholders::_1, std::placeholders::_2));
  srv_pause_mission_ = this->create_service<qyh_standard_robot_msgs::srv::ControlPauseMission>(
    "control_pause_mission",
    std::bind(&StandardRobotNode::handle_pause_mission, this, std::placeholders::_1, std::placeholders::_2));
  srv_resume_mission_ = this->create_service<qyh_standard_robot_msgs::srv::ControlResumeMission>(
    "control_resume_mission",
    std::bind(&StandardRobotNode::handle_resume_mission, this, std::placeholders::_1, std::placeholders::_2));
  srv_cancel_mission_ = this->create_service<qyh_standard_robot_msgs::srv::ControlCancelMission>(
    "control_cancel_mission",
    std::bind(&StandardRobotNode::handle_cancel_mission, this, std::placeholders::_1, std::placeholders::_2));

  manual_motion_sub_ = this->create_subscription<qyh_standard_robot_msgs::msg::ManualMotionCommand>(
    "manual_motion_cmd", 10,
    [this](const qyh_standard_robot_msgs::msg::ManualMotionCommand::SharedPtr msg) {
      last_cmd_ = msg;
      last_cmd_time_ = std::chrono::steady_clock::now();
    });
  auto cmd_period = std::chrono::duration<double>(1.0 / manual_control_rate_);
  command_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(cmd_period),
    std::bind(&StandardRobotNode::command_timer_callback, this));
}

StandardRobotNode::~StandardRobotNode()
{
  disconnect_modbus();
}

bool StandardRobotNode::connect_modbus()
{
  try {
    // Create Modbus TCP context using C++ API
    modbus_ctx_ = std::make_unique<modbus::ModbusTCP>(modbus_ip_, modbus_port_);
    
    // Set slave ID
    modbus_ctx_->set_slave(slave_id_);
    
    // Set response timeout (2 seconds)
    modbus_ctx_->set_response_timeout(2, 0);
    
    // Connect
    modbus_ctx_->connect();
    
    RCLCPP_INFO(this->get_logger(), "Successfully connected to Modbus device");
    is_connected_ = true;
    return true;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus exception: %s", e.what());
    modbus_ctx_.reset();
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    modbus_ctx_.reset();
    return false;
  }
}

void StandardRobotNode::disconnect_modbus()
{
  if (modbus_ctx_) {
    try {
      modbus_ctx_->close();
    } catch (...) {
      // Ignore exceptions during disconnect
    }
    modbus_ctx_.reset();
    is_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "Disconnected from Modbus device");
  }
}

void StandardRobotNode::timer_callback()
{
  if (!is_connected_) {
    // Try to reconnect
    if (!connect_modbus()) {
      return;
    }
  }
  
  if (read_robot_status()) {
    // status_msg_.header.stamp = this->now();
    // status_msg_.header.frame_id = "base_link";
    status_pub_->publish(status_msg_);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Failed to read robot status");
    // Disconnect and try to reconnect on next iteration
    disconnect_modbus();
  }
}

bool StandardRobotNode::read_robot_status()
{
  if (!modbus_ctx_) {
    return false;
  }
  
  try {
    // Read discrete inputs (10001-10136)
    // Note: Modbus addressing starts from 0, so 10001 -> 0, 10002 -> 1, etc.
    std::vector<uint8_t> discrete_inputs = modbus_ctx_->read_discrete_inputs(10001, 136);
    
    // Parse discrete inputs
    status_msg_.is_emergency_stopped = discrete_inputs[0];
    status_msg_.is_emergency_recoverable = discrete_inputs[1];
    status_msg_.is_brake_released = discrete_inputs[2];
    status_msg_.is_charging = discrete_inputs[3];
    status_msg_.is_low_power_mode = discrete_inputs[4];
    status_msg_.obstacle_slowdown = discrete_inputs[5];
    status_msg_.obstacle_paused = discrete_inputs[6];
    status_msg_.can_run_motion_task = discrete_inputs[8];
    status_msg_.is_auto_mode = discrete_inputs[50];
    status_msg_.is_loaded = discrete_inputs[53];
    status_msg_.has_wifi = discrete_inputs[54];
    status_msg_.workplace_obstacle_signal = discrete_inputs[135];
    
    // Read input registers (30001-30122)
    // Modbus function 0x04 - Read Input Registers
    std::vector<uint16_t> tab_reg = modbus_ctx_->read_input_registers(30001, 122);
    
    // Parse input registers (30001 -> tab_reg[0], 30002 -> tab_reg[1], etc.)
    // 30001 系统状态
    status_msg_.system_status = tab_reg[0];
    
    // 30002 定位状态
    status_msg_.location_status = tab_reg[1];
    
    // 30003-30007 位姿信息 (x, y, yaw)
    int32_t pos_x = (static_cast<int32_t>(tab_reg[2]) << 16) | tab_reg[3];
    int32_t pos_y = (static_cast<int32_t>(tab_reg[4]) << 16) | tab_reg[5];
    int32_t yaw = (static_cast<int32_t>(tab_reg[6]) << 16) | tab_reg[7];
    
    status_msg_.pose.header.stamp = this->now();
    status_msg_.pose.header.frame_id = "map";
    status_msg_.pose.pose.pose.position.x = pos_x / 1000.0;  // mm to m
    status_msg_.pose.pose.pose.position.y = pos_y / 1000.0;  // mm to m
    status_msg_.pose.pose.pose.position.z = 0.0;
    
    // Convert yaw (rad*1000) to quaternion
    double yaw_rad = yaw / 1000.0;
    status_msg_.pose.pose.pose.orientation.x = 0.0;
    status_msg_.pose.pose.pose.orientation.y = 0.0;
    status_msg_.pose.pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    status_msg_.pose.pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
    
    // 30009 位姿置信度
    status_msg_.pose_confidence = tab_reg[8] * 0.0001;  // 0.01% to ratio
    
    // 30015 当前站点编号
    status_msg_.current_station_id = tab_reg[14];
    
    // 30016 操作状态
    status_msg_.operation_status = tab_reg[15];
    
    // 30017-30019 速度信息
    int16_t vel_x = static_cast<int16_t>(tab_reg[16]);
    int16_t vel_y = static_cast<int16_t>(tab_reg[17]);
    int16_t omega = static_cast<int16_t>(tab_reg[18]);
    
    status_msg_.twist.linear.x = vel_x / 1000.0;  // mm/s to m/s
    status_msg_.twist.linear.y = vel_y / 1000.0;
    status_msg_.twist.linear.z = 0.0;
    status_msg_.twist.angular.x = 0.0;
    status_msg_.twist.angular.y = 0.0;
    status_msg_.twist.angular.z = omega / 1000.0;  // (1/1000)rad/s to rad/s
    
    // 30020 调度模式
    status_msg_.scheduling_mode = tab_reg[19];
    
    // 30027-30028 系统上一次错误 (uint32)
    status_msg_.last_error_code = (static_cast<uint32_t>(tab_reg[26]) << 16) | tab_reg[27];
    
    // 30033 电池电压 (mV)
    status_msg_.battery_voltage = tab_reg[32] / 1000.0;  // mV to V
    
    // 30034 电池电流 (mA)
    status_msg_.battery_current = static_cast<int16_t>(tab_reg[33]) / 1000.0;  // mA to A
    
    // 30035 电池温度 (°C)
    status_msg_.battery_temperature = static_cast<int16_t>(tab_reg[34]);
    
    // 30036 电池预计使用时间 (min)
    status_msg_.battery_estimated_using_time = tab_reg[35];
    
    // 30037 当前电量百分比
    status_msg_.battery_remaining_percentage = tab_reg[36];
    
    // 30038 电池状态
    status_msg_.battery_status = tab_reg[37];
    
    // 30039 电池循环次数
    status_msg_.battery_cycle_count = tab_reg[38];
    
    // 30040 电池标称容量 (mAh)
    status_msg_.battery_nominal_capacity = tab_reg[39];
    
    // 30041-30042 运动总里程 (m)
    status_msg_.total_motion_distance = (static_cast<uint32_t>(tab_reg[40]) << 16) | tab_reg[41];
    
    // 30043-30044 开机总时间 (s)
    status_msg_.total_boot_time = (static_cast<uint32_t>(tab_reg[42]) << 16) | tab_reg[43];
    
    // 30045-30046 开机总次数
    status_msg_.total_boot_count = (static_cast<uint32_t>(tab_reg[44]) << 16) | tab_reg[45];
    
    // 30047-30048 系统当前时间 (Linux时间戳)
    status_msg_.system_current_time = (static_cast<uint32_t>(tab_reg[46]) << 16) | tab_reg[47];
    
    // 30049-30052 IP地址
    status_msg_.ip_addresses[0] = tab_reg[48];
    status_msg_.ip_addresses[1] = tab_reg[49];
    status_msg_.ip_addresses[2] = tab_reg[50];
    status_msg_.ip_addresses[3] = tab_reg[51];
    
    // 30065 当前地图名 (simplified - just store the value)
    // For now, we'll convert the register values to a simple string
    char map_name[32];
    snprintf(map_name, sizeof(map_name), "%04X", tab_reg[64]);
    status_msg_.current_map_name = map_name;
    
    // 30070 当前系统音量
    status_msg_.current_system_volume = tab_reg[69];
    
    // 30071 关机状态
    status_msg_.shutdown_status = tab_reg[70];
    
    // 30072 当前时区
    status_msg_.current_timezone = static_cast<int16_t>(tab_reg[71]);
    
    // 30073 机器人运动状态
    status_msg_.motion_status = tab_reg[72];
    
    // 30091-30092 工位避障触发源 (uint32, bit flags)
    uint32_t obstacle_flags = (static_cast<uint32_t>(tab_reg[90]) << 16) | tab_reg[91];
    status_msg_.obstacle_avoidance_triggered_main_radar = (obstacle_flags & 0x01) != 0;
    status_msg_.obstacle_avoidance_triggered_aux_radar = (obstacle_flags & 0x02) != 0;
    status_msg_.obstacle_avoidance_triggered_depth_camera1 = (obstacle_flags & 0x04) != 0;
    status_msg_.obstacle_avoidance_triggered_depth_camera2 = (obstacle_flags & 0x08) != 0;
    status_msg_.obstacle_avoidance_triggered_depth_camera3 = (obstacle_flags & 0x10) != 0;
    status_msg_.obstacle_avoidance_triggered_depth_camera4 = (obstacle_flags & 0x20) != 0;
    status_msg_.obstacle_avoidance_triggered_obstacle_radar1 = (obstacle_flags & 0x40) != 0;
    status_msg_.obstacle_avoidance_triggered_obstacle_radar2 = (obstacle_flags & 0x80) != 0;
    status_msg_.obstacle_avoidance_triggered_obstacle_radar3 = (obstacle_flags & 0x100) != 0;
    status_msg_.obstacle_avoidance_triggered_obstacle_radar4 = (obstacle_flags & 0x200) != 0;
    
    // 30097-30098 当前mission id
    status_msg_.current_mission_id = (static_cast<uint32_t>(tab_reg[96]) << 16) | tab_reg[97];
    
    // 30099 Mission运行状态
    status_msg_.mission_status = tab_reg[98];
    
    // 30100 Mission执行结果
    status_msg_.mission_result = tab_reg[99];
    
    // 30101-30102 Mission错误码
    status_msg_.mission_error_code = (static_cast<uint32_t>(tab_reg[100]) << 16) | tab_reg[101];
    
    // 30113 移动任务状态
    status_msg_.move_task_status = tab_reg[112];
    
    // 30114-30115 当前移动任务no (int32)
    status_msg_.current_move_task_no = (static_cast<int32_t>(tab_reg[113]) << 16) | tab_reg[114];
    
    // 30116 当前移动任务目标站点
    status_msg_.current_move_task_destination_station = tab_reg[115];
    
    // 30117 当前路径编号
    status_msg_.current_move_task_path_no = tab_reg[116];
    
    // 30118 当前移动任务起始站点
    status_msg_.current_move_task_start_station = tab_reg[117];
    
    // 30122 移动任务结果
    status_msg_.move_task_result = tab_reg[121];
    
    return true;
    
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus exception while reading: %s", e.what());
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while reading: %s", e.what());
    return false;
  }
}

bool StandardRobotNode::write_coil(uint16_t addr)
{
  if (!is_connected_ || !modbus_ctx_) {
    return false;
  }
  try {
    modbus_ctx_->write_coil(addr, true);
    return true;
  } catch (const modbus::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Modbus exception while writing coil %u: %s", addr, e.what());
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while writing coil %u: %s", addr, e.what());
    return false;
  }
}

void StandardRobotNode::command_timer_callback()
{
  if(!is_manual_control_||!last_cmd_) {
    return;
  }
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time_).count() > manual_command_timeout_ms_) {
    return;
  }
  if (last_cmd_->forward && !last_cmd_->backward > 0) {
    write_coil(17);
  } else if (last_cmd_->backward && !last_cmd_->forward ) {
    write_coil(18);
  }
  if (last_cmd_->rotate_left && !last_cmd_->rotate_right) {
    write_coil(19);
  } else if (last_cmd_->rotate_right && !last_cmd_->rotate_left ) {
    write_coil(20);
  }
}

void StandardRobotNode::handle_pause_move(const qyh_standard_robot_msgs::srv::ControlPauseMove::Request::SharedPtr,
                                          qyh_standard_robot_msgs::srv::ControlPauseMove::Response::SharedPtr res)
{
  bool ok = write_coil(1);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_resume_move(const qyh_standard_robot_msgs::srv::ControlResumeMove::Request::SharedPtr,
                                           qyh_standard_robot_msgs::srv::ControlResumeMove::Response::SharedPtr res)
{
  bool ok = write_coil(2);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_stop_move(const qyh_standard_robot_msgs::srv::ControlStopMove::Request::SharedPtr,
                                         qyh_standard_robot_msgs::srv::ControlStopMove::Response::SharedPtr res)
{
  bool ok = write_coil(3);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_stop_localization(const qyh_standard_robot_msgs::srv::ControlStopLocalization::Request::SharedPtr,
                                                 qyh_standard_robot_msgs::srv::ControlStopLocalization::Response::SharedPtr res)
{
  bool ok = write_coil(4);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_emergency_stop(const qyh_standard_robot_msgs::srv::ControlEmergencyStop::Request::SharedPtr,
                                              qyh_standard_robot_msgs::srv::ControlEmergencyStop::Response::SharedPtr res)
{
  bool ok = write_coil(6);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_release_emergency_stop(const qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop::Request::SharedPtr,
                                                      qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop::Response::SharedPtr res)
{
  bool ok = write_coil(7);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_start_charging(const qyh_standard_robot_msgs::srv::ControlStartCharging::Request::SharedPtr,
                                              qyh_standard_robot_msgs::srv::ControlStartCharging::Response::SharedPtr res)
{
  bool ok = write_coil(8);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_stop_charging(const qyh_standard_robot_msgs::srv::ControlStopCharging::Request::SharedPtr,
                                             qyh_standard_robot_msgs::srv::ControlStopCharging::Response::SharedPtr res)
{
  bool ok = write_coil(9);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_enter_low_power(const qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode::Request::SharedPtr,
                                               qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode::Response::SharedPtr res)
{
  bool ok = write_coil(10);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_exit_low_power(const qyh_standard_robot_msgs::srv::ControlExitLowPowerMode::Request::SharedPtr,
                                              qyh_standard_robot_msgs::srv::ControlExitLowPowerMode::Response::SharedPtr res)
{
  bool ok = write_coil(11);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_system_reset(const qyh_standard_robot_msgs::srv::ControlSystemReset::Request::SharedPtr,
                                            qyh_standard_robot_msgs::srv::ControlSystemReset::Response::SharedPtr res)
{
  bool ok = write_coil(12);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_start_manual(const qyh_standard_robot_msgs::srv::ControlStartManualControl::Request::SharedPtr,
                                            qyh_standard_robot_msgs::srv::ControlStartManualControl::Response::SharedPtr res)
{
  bool ok = write_coil(15);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
  if(ok)
  {
    is_manual_control_ = true;
  }
}

void StandardRobotNode::handle_stop_manual(const qyh_standard_robot_msgs::srv::ControlStopManualControl::Request::SharedPtr,
                                           qyh_standard_robot_msgs::srv::ControlStopManualControl::Response::SharedPtr res)
{
  bool ok = write_coil(16);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
  if(ok)
  {
    is_manual_control_ = false;
  }
}

void StandardRobotNode::handle_pause_mission(const qyh_standard_robot_msgs::srv::ControlPauseMission::Request::SharedPtr,
                                             qyh_standard_robot_msgs::srv::ControlPauseMission::Response::SharedPtr res)
{
  bool ok = write_coil(97);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_resume_mission(const qyh_standard_robot_msgs::srv::ControlResumeMission::Request::SharedPtr,
                                              qyh_standard_robot_msgs::srv::ControlResumeMission::Response::SharedPtr res)
{
  bool ok = write_coil(98);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

void StandardRobotNode::handle_cancel_mission(const qyh_standard_robot_msgs::srv::ControlCancelMission::Request::SharedPtr,
                                              qyh_standard_robot_msgs::srv::ControlCancelMission::Response::SharedPtr res)
{
  bool ok = write_coil(99);
  res->success = ok;
  res->message = ok ? "ok" : "failed";
}

}  // namespace qyh_standard_robot

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qyh_standard_robot::StandardRobotNode)
