cmake_minimum_required(VERSION 3.8)
project(qyh_teleoperation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 声明消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VirtualArmState.msg"
  "msg/TeleopStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/PerformanceMetrics.msg"
  "srv/StartTeleoperation.srv"
  "srv/StopTeleoperation.srv"
  "srv/SetControlMode.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
