cmake_minimum_required(VERSION 3.8)
project(qyh_head_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)

# Head motor control node
add_executable(head_motor_node
  src/head_motor_node.cpp
  src/bus_servo_protocol.cpp
)
ament_target_dependencies(head_motor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
)

# Install executables
install(TARGETS
  head_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
