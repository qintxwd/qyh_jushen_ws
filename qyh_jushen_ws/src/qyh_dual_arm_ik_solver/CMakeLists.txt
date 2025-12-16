cmake_minimum_required(VERSION 3.8)
project(qyh_dual_arm_ik_solver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ./thirdparty/include
)

# JAKA SDK library (use symlink or copy from qyh_jaka_control)
set(JAKA_API_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libjakaAPI_2_3_0_13.so")
if(NOT EXISTS ${JAKA_API_LIB})
  message(FATAL_ERROR "JAKA API library not found: ${JAKA_API_LIB}")
endif()

# Set RPATH
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/thirdparty")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Main IK solver node
add_executable(dual_arm_ik_solver_node src/dual_arm_ik_solver_node.cpp)
ament_target_dependencies(dual_arm_ik_solver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(dual_arm_ik_solver_node
  ${JAKA_API_LIB}
)

install(TARGETS
  dual_arm_ik_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install JAKA SDK library
install(FILES
  ${JAKA_API_LIB}
  DESTINATION lib/${PROJECT_NAME}/thirdparty
)

ament_package()
