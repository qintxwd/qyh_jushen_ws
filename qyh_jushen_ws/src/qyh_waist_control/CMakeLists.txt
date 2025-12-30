cmake_minimum_required(VERSION 3.8)
project(qyh_waist_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qyh_waist_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  thirdparty/include
)

# Link directories
link_directories(
  thirdparty/lib/linux
)

# Waist control node
add_executable(waist_control_node
  src/waist_control_node_main.cpp
  src/waist_control_node.cpp
)

ament_target_dependencies(waist_control_node
  rclcpp
  qyh_waist_msgs
  std_msgs
  sensor_msgs
)

target_link_libraries(waist_control_node
  modbus
)

# Install
install(TARGETS
  waist_control_node
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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
