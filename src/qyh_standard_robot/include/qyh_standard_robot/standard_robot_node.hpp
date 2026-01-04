#ifndef QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
#define QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <qyh_standard_robot_msgs/msg/standard_robot_status.hpp>
#include <qyh_standard_robot_msgs/msg/manual_motion_command.hpp>
#include <qyh_standard_robot_msgs/msg/manual_velocity_command.hpp>
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
#include <qyh_standard_robot_msgs/srv/go_set_speed_type.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_coordinate.hpp>
#include <qyh_standard_robot_msgs/srv/go_execute_action_task.hpp>
#include <qyh_standard_robot_msgs/srv/go_set_obstacle_strategy.hpp>
#include <qyh_standard_robot_msgs/srv/go_set_current_site.hpp>
#include <qyh_standard_robot_msgs/srv/go_set_speaker_volume.hpp>
#include <qyh_standard_robot_msgs/srv/go_set_current_map.hpp>
#include <qyh_standard_robot_msgs/srv/go_force_localize.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_pose_with_task.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_site.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_site_with_task.hpp>
#include <qyh_standard_robot_msgs/msg/navigation_status.hpp>
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
  bool write_holding_register(uint16_t addr, uint16_t value);
  bool write_holding_registers(uint16_t addr, const std::vector<uint16_t>& values);
  void command_timer_callback();

  // ROS2 members
  rclcpp::Publisher<qyh_standard_robot_msgs::msg::StandardRobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  qyh_standard_robot_msgs::msg::StandardRobotStatus status_msg_;
  rclcpp::Subscription<qyh_standard_robot_msgs::msg::ManualMotionCommand>::SharedPtr manual_motion_sub_;
  rclcpp::Subscription<qyh_standard_robot_msgs::msg::ManualVelocityCommand>::SharedPtr manual_velocity_sub_;
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
  std::chrono::steady_clock::time_point last_vel_cmd_time_;
  qyh_standard_robot_msgs::msg::ManualMotionCommand::SharedPtr last_cmd_;
  qyh_standard_robot_msgs::msg::ManualVelocityCommand::SharedPtr last_vel_cmd_;

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

  // Go series services for navigation control
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoSetSpeedType>::SharedPtr srv_go_set_speed_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoNavigateToCoordinate>::SharedPtr srv_go_nav_coord_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoExecuteActionTask>::SharedPtr srv_go_nav_site_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoNavigateToSite>::SharedPtr srv_go_nav_site_simple_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoSetObstacleStrategy>::SharedPtr srv_go_obstacle_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoSetCurrentSite>::SharedPtr srv_go_current_site_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoSetSpeakerVolume>::SharedPtr srv_go_volume_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoSetCurrentMap>::SharedPtr srv_go_map_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoForceLocalize>::SharedPtr srv_go_force_loc_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoNavigateToPoseWithTask>::SharedPtr srv_go_nav_pose_task_;
  rclcpp::Service<qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask>::SharedPtr srv_go_nav_site_task_;

  rclcpp::Publisher<qyh_standard_robot_msgs::msg::NavigationStatus>::SharedPtr nav_status_pub_;

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

  // Go series service handlers
  void handle_go_set_speed(const qyh_standard_robot_msgs::srv::GoSetSpeedType::Request::SharedPtr,
                           qyh_standard_robot_msgs::srv::GoSetSpeedType::Response::SharedPtr);
  void handle_go_nav_coord(const qyh_standard_robot_msgs::srv::GoNavigateToCoordinate::Request::SharedPtr,
                           qyh_standard_robot_msgs::srv::GoNavigateToCoordinate::Response::SharedPtr);
  void handle_go_nav_site(const qyh_standard_robot_msgs::srv::GoExecuteActionTask::Request::SharedPtr,
                          qyh_standard_robot_msgs::srv::GoExecuteActionTask::Response::SharedPtr);
  void handle_go_nav_site_simple(const qyh_standard_robot_msgs::srv::GoNavigateToSite::Request::SharedPtr,
                                 qyh_standard_robot_msgs::srv::GoNavigateToSite::Response::SharedPtr);
  void handle_go_obstacle(const qyh_standard_robot_msgs::srv::GoSetObstacleStrategy::Request::SharedPtr,
                          qyh_standard_robot_msgs::srv::GoSetObstacleStrategy::Response::SharedPtr);
  void handle_go_current_site(const qyh_standard_robot_msgs::srv::GoSetCurrentSite::Request::SharedPtr,
                              qyh_standard_robot_msgs::srv::GoSetCurrentSite::Response::SharedPtr);
  void handle_go_volume(const qyh_standard_robot_msgs::srv::GoSetSpeakerVolume::Request::SharedPtr,
                        qyh_standard_robot_msgs::srv::GoSetSpeakerVolume::Response::SharedPtr);
  void handle_go_map(const qyh_standard_robot_msgs::srv::GoSetCurrentMap::Request::SharedPtr,
                     qyh_standard_robot_msgs::srv::GoSetCurrentMap::Response::SharedPtr);
  void handle_go_force_loc(const qyh_standard_robot_msgs::srv::GoForceLocalize::Request::SharedPtr,
                           qyh_standard_robot_msgs::srv::GoForceLocalize::Response::SharedPtr);
  void handle_go_nav_pose_task(const qyh_standard_robot_msgs::srv::GoNavigateToPoseWithTask::Request::SharedPtr,
                               qyh_standard_robot_msgs::srv::GoNavigateToPoseWithTask::Response::SharedPtr);
  void handle_go_nav_site_task(const qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask::Request::SharedPtr,
                               qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask::Response::SharedPtr);
};

}  // namespace qyh_standard_robot

#endif  // QYH_STANDARD_ROBOT__STANDARD_ROBOT_NODE_HPP_
