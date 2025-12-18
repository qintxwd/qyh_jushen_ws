cmake_minimum_required(VERSION 3.8)
project(qyh_jaka_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(qyh_jaka_control_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# Include directories
include_directories(
  include
  ./thirdparty/include
  ${orocos_kdl_INCLUDE_DIRS}
)

# Resolve JAKA API shared library (Jetson ARM64 version)
set(JAKA_API_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libjakaAPI_2_3_0_13.so")
if(NOT EXISTS ${JAKA_API_LIB})
  message(FATAL_ERROR "JAKA API shared library not found at ${JAKA_API_LIB}. Please verify thirdparty/lib contents.")
endif()

# Set RPATH so the executable can find the shared library at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/thirdparty")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Libraries
add_library(jaka_interface src/jaka_interface.cpp)
ament_target_dependencies(jaka_interface
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(jaka_interface
  ${JAKA_API_LIB}
)

add_library(velocity_servo_controller src/velocity_servo_controller.cpp)
ament_target_dependencies(velocity_servo_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  orocos_kdl
  kdl_parser
  urdf
)
target_link_libraries(velocity_servo_controller
  ${orocos_kdl_LIBRARIES}
)

# Main control node (unified: basic + servo + VR)
add_executable(jaka_control_node 
  src/jaka_control_node.cpp
  src/jaka_service_handlers.cpp
)
ament_target_dependencies(jaka_control_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  qyh_jaka_control_msgs
  ament_index_cpp
  orocos_kdl
  kdl_parser
  urdf
)
target_link_libraries(jaka_control_node
  jaka_interface
  velocity_servo_controller
  ${JAKA_API_LIB}
  ${orocos_kdl_LIBRARIES}
)

# Bridge node for teleoperation (subscribes to /left_arm/joint_command, /right_arm/joint_command)
# add_executable(jaka_bridge_node src/jaka_bridge_node.cpp)
# ament_target_dependencies(jaka_bridge_node
#   rclcpp
#   std_srvs
#   sensor_msgs
# )
# target_link_libraries(jaka_bridge_node
#   jaka_interface velocity_servo_controller
#   smooth_servo_bridge
#   ${JAKA_API_LIB}
# )

install(TARGETS jaka_interface velocity_servo_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  jaka_control_node
  # jaka_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts (including joint adapter)
install(PROGRAMS
  scripts/qyh_jaka_joint_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install JAKA API shared library
install(FILES
  ${JAKA_API_LIB}
  DESTINATION lib/${PROJECT_NAME}/thirdparty
)

ament_package()