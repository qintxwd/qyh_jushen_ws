cmake_minimum_required(VERSION 3.8)
project(qyh_gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qyh_gripper_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  thirdparty/include
)

# Link directories
link_directories(
  thirdparty/lib/linux
)

# Gripper control node
add_executable(gripper_control_node
  src/gripper_control_node_main.cpp
  src/gripper_control_node.cpp
)

ament_target_dependencies(gripper_control_node
  rclcpp
  qyh_gripper_msgs
  std_msgs
)

target_link_libraries(gripper_control_node
  modbus
)

# Install
install(TARGETS
  gripper_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY thirdparty/lib/linux/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
