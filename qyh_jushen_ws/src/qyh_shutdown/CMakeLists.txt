cmake_minimum_required(VERSION 3.8)
project(qyh_shutdown)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  thirdparty/include
)

# Link directories
link_directories(
  thirdparty/lib/linux
)

# Lift control node
add_executable(qyh_shutdown_node
  src/qyh_shutdown_node.cpp
)

ament_target_dependencies(qyh_shutdown_node
  rclcpp
  std_msgs
  std_srvs
)

target_link_libraries(qyh_shutdown_node
  modbus
)

# Install
install(TARGETS
  qyh_shutdown_node
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
