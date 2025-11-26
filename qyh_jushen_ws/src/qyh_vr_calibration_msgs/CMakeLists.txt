cmake_minimum_required(VERSION 3.8)
project(qyh_vr_calibration_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CalibrationPose.msg"
)

set(srv_files
  "srv/CalibrateUser.srv"
  "srv/LoadUserCalibration.srv"
  "srv/DeleteUser.srv"
  "srv/ListUsers.srv"
  "srv/GetRobotCalibration.srv"
  "srv/SetRobotCalibration.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
