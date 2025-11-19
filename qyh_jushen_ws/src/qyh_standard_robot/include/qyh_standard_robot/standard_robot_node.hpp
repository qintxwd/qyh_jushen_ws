#ifndef QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
#define QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qyh_standard_robot_msgs/msg/standard_robot_status.hpp>
#include <qyh_standard_robot_msgs/msg/manual_motion_command.hpp>
#include <qyh_standard_robot_msgs/srv/control_cancel_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_emergency_stop.hpp>
#include <qyh_standard_robot_msgs/srv/control_enter_low_power_mode.hpp>
#include <qyh_standard_robot_msgs/srv/control_exit_low_power_mode.hpp>
#include <qyh_standard_robot_msgs/srv/control_pause_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_pause_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_release_emergency_stop.hpp>
#include <qyh_standard_robot_msgs/srv/control_resume_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_resume_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_start_charging.hpp>
#include <qyh_standard_robot_msgs/srv/control_start_manual_control.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_charging.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_localization.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_manual_control.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_system_reset.hpp>
#include <modbus/modbus.hpp>

namespace qyh_standard_robot
{

class StandardRobotNode : public rclcpp::Node
{
public:
  explicit StandardRobotNode(const rclcpp::NodeOptions & options);
  virtual ~StandardRobotNode();

private:
  void timer_callback();
  bool connect_modbus();
  void disconnect_modbus();
  bool read_robot_status();
  bool write_coil(uint16_t addr);
  void command_timer_callback();

  // ROS2 members
  rclcpp::Publisher<qyh_standard_robot_msgs::msg::StandardRobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  qyh_standard_robot_msgs::msg::StandardRobotStatus status_msg_;
  rclcpp::Subscription<qyh_standard_robot_msgs::msg::ManualMotionCommand>::SharedPtr manual_motion_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  // Modbus members - use C++ API
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  bool is_connected_;

  // Parameters
  std::string modbus_ip_;
  int modbus_port_;
  int slave_id_;
  double publish_rate_;
  bool is_manual_control_;
  double manual_control_rate_;
  int manual_command_timeout_ms_;  
  std::chrono::steady_clock::time_point last_cmd_time_;
  qyh_standard_robot_msgs::msg::ManualMotionCommand::SharedPtr last_cmd_;

  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlPauseMove>::SharedPtr srv_pause_move_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlResumeMove>::SharedPtr srv_resume_move_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStopMove>::SharedPtr srv_stop_move_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStopLocalization>::SharedPtr srv_stop_localization_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlEmergencyStop>::SharedPtr srv_emergency_stop_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop>::SharedPtr srv_release_emergency_stop_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStartCharging>::SharedPtr srv_start_charging_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStopCharging>::SharedPtr srv_stop_charging_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode>::SharedPtr srv_enter_low_power_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlExitLowPowerMode>::SharedPtr srv_exit_low_power_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlSystemReset>::SharedPtr srv_system_reset_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStartManualControl>::SharedPtr srv_start_manual_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlStopManualControl>::SharedPtr srv_stop_manual_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlPauseMission>::SharedPtr srv_pause_mission_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlResumeMission>::SharedPtr srv_resume_mission_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::ControlCancelMission>::SharedPtr srv_cancel_mission_;

  void handle_pause_move(const qyh_standard_robot_msgs::srv::ControlPauseMove::Request::SharedPtr,
                         qyh_standard_robot_msgs::srv::ControlPauseMove::Response::SharedPtr);
  void handle_resume_move(const qyh_standard_robot_msgs::srv::ControlResumeMove::Request::SharedPtr,
                          qyh_standard_robot_msgs::srv::ControlResumeMove::Response::SharedPtr);
  void handle_stop_move(const qyh_standard_robot_msgs::srv::ControlStopMove::Request::SharedPtr,
                        qyh_standard_robot_msgs::srv::ControlStopMove::Response::SharedPtr);
  void handle_stop_localization(const qyh_standard_robot_msgs::srv::ControlStopLocalization::Request::SharedPtr,
                                qyh_standard_robot_msgs::srv::ControlStopLocalization::Response::SharedPtr);
  void handle_emergency_stop(const qyh_standard_robot_msgs::srv::ControlEmergencyStop::Request::SharedPtr,
                             qyh_standard_robot_msgs::srv::ControlEmergencyStop::Response::SharedPtr);
  void handle_release_emergency_stop(const qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop::Request::SharedPtr,
                                     qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop::Response::SharedPtr);
  void handle_start_charging(const qyh_standard_robot_msgs::srv::ControlStartCharging::Request::SharedPtr,
                             qyh_standard_robot_msgs::srv::ControlStartCharging::Response::SharedPtr);
  void handle_stop_charging(const qyh_standard_robot_msgs::srv::ControlStopCharging::Request::SharedPtr,
                            qyh_standard_robot_msgs::srv::ControlStopCharging::Response::SharedPtr);
  void handle_enter_low_power(const qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode::Request::SharedPtr,
                              qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode::Response::SharedPtr);
  void handle_exit_low_power(const qyh_standard_robot_msgs::srv::ControlExitLowPowerMode::Request::SharedPtr,
                             qyh_standard_robot_msgs::srv::ControlExitLowPowerMode::Response::SharedPtr);
  void handle_system_reset(const qyh_standard_robot_msgs::srv::ControlSystemReset::Request::SharedPtr,
                           qyh_standard_robot_msgs::srv::ControlSystemReset::Response::SharedPtr);
  void handle_start_manual(const qyh_standard_robot_msgs::srv::ControlStartManualControl::Request::SharedPtr,
                           qyh_standard_robot_msgs::srv::ControlStartManualControl::Response::SharedPtr);
  void handle_stop_manual(const qyh_standard_robot_msgs::srv::ControlStopManualControl::Request::SharedPtr,
                          qyh_standard_robot_msgs::srv::ControlStopManualControl::Response::SharedPtr);
  void handle_pause_mission(const qyh_standard_robot_msgs::srv::ControlPauseMission::Request::SharedPtr,
                            qyh_standard_robot_msgs::srv::ControlPauseMission::Response::SharedPtr);
  void handle_resume_mission(const qyh_standard_robot_msgs::srv::ControlResumeMission::Request::SharedPtr,
                             qyh_standard_robot_msgs::srv::ControlResumeMission::Response::SharedPtr);
  void handle_cancel_mission(const qyh_standard_robot_msgs::srv::ControlCancelMission::Request::SharedPtr,
                             qyh_standard_robot_msgs::srv::ControlCancelMission::Response::SharedPtr);
};

}  // namespace qyh_standard_robot

#endif  // QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
