cmake_minimum_required(VERSION 3.8)
project(jaka_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EnableRobot.srv"
  "srv/DisableRobot.srv"
  "srv/PowerOn.srv"
  "srv/PowerOff.srv"
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/SetCollisionLevel.srv"
  "srv/SetToolOffset.srv"
  "srv/GetRobotState.srv"
  "srv/ClearError.srv"
  "srv/MotionAbort.srv"
  "srv/ServoMoveEnable.srv"
  "srv/ServoJ.srv"
  "srv/ServoP.srv"
  "srv/SetServoFilter.srv"
  "msg/JointPosition.msg"
  "msg/CartesianPose.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/include
)

# Find JAKA SDK library
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib
)

# Add executable
add_executable(jaka_robot_node 
  src/jaka_robot_node.cpp
  src/jaka_robot_interface.cpp
)

ament_target_dependencies(jaka_robot_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Link JAKA SDK library
target_link_libraries(jaka_robot_node
  jakaAPI  # Adjust library name based on actual SDK library
)

rosidl_target_interfaces(jaka_robot_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  jaka_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
