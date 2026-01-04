cmake_minimum_required(VERSION 3.8)
project(qyh_lift_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qyh_lift_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Detect architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(ARCH_DIR "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(ARCH_DIR "x64")
else()
  set(ARCH_DIR "arm64")  # Default to arm64 for Jetson
endif()

message(STATUS "Detected architecture: ${CMAKE_SYSTEM_PROCESSOR}, using ${ARCH_DIR}")

# Include directories
include_directories(
  include
  thirdparty/include
)

# Link directories
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/linux/${ARCH_DIR}
)

# Lift control node
add_executable(lift_control_node
  src/lift_control_node_main.cpp
  src/lift_control_node.cpp
)

ament_target_dependencies(lift_control_node
  rclcpp
  qyh_lift_msgs
  std_msgs
)

target_link_libraries(lift_control_node
  modbus
)

# Install
install(TARGETS
  lift_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the correct architecture libraries
install(DIRECTORY thirdparty/lib/linux/${ARCH_DIR}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
  PATTERN "*.so" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  PATTERN "*.so.*" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
