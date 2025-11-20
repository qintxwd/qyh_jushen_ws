cmake_minimum_required(VERSION 3.8)
project(qyh_standard_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StandardRobotStatus.msg"
  "msg/ManualMotionCommand.msg"
  "msg/NavigationPose.msg"
  "msg/NavigationStatus.msg"
  "srv/ControlPauseMove.srv"
  "srv/ControlResumeMove.srv"
  "srv/ControlStopMove.srv"
  "srv/ControlStopLocalization.srv"
  "srv/ControlEmergencyStop.srv"
  "srv/ControlReleaseEmergencyStop.srv"
  "srv/ControlStartCharging.srv"
  "srv/ControlStopCharging.srv"
  "srv/ControlEnterLowPowerMode.srv"
  "srv/ControlExitLowPowerMode.srv"
  "srv/ControlSystemReset.srv"
  "srv/ControlStartManualControl.srv"
  "srv/ControlStopManualControl.srv"
  "srv/ControlPauseMission.srv"
  "srv/ControlResumeMission.srv"
  "srv/ControlCancelMission.srv"
  "srv/GoSetSpeedType.srv"
  "srv/GoNavigateToCoordinate.srv"
  "srv/GoExecuteActionTask.srv"
  "srv/GoSetManualControl.srv"
  "srv/GoSetObstacleStrategy.srv"
  "srv/GoSetCurrentSite.srv"
  "srv/GoSetSpeakerVolume.srv"
  "srv/GoSetCurrentMap.srv"
  "srv/GoForceLocalize.srv"
  "srv/GoNavigateToPoseWithTask.srv"
  "srv/GoNavigateToSiteWithTask.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
