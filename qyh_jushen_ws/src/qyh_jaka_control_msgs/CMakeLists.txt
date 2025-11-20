cmake_minimum_required(VERSION 3.8)
project(qyh_jaka_control_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JakaDualJointServo.msg"
  "msg/JakaDualCartesianServo.msg"
  "msg/JakaServoStatus.msg"
  "msg/VRPose.msg"
  "msg/VRFollowStatus.msg"
  "msg/RobotState.msg"
  "srv/StartServo.srv"
  "srv/StopServo.srv"
  "srv/ZeroFT.srv"
  "srv/SetFTConfig.srv"
  "srv/SetFilter.srv"
  "srv/EnableVRFollow.srv"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  "srv/CalibrateVR.srv"
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/SetCollisionLevel.srv"
  "srv/SetToolOffset.srv"
  "srv/GetRobotState.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()