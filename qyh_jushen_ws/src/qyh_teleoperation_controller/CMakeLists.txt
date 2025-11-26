cmake_minimum_required(VERSION 3.8)
project(qyh_teleoperation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qyh_teleoperation_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  qyh_teleoperation_msgs
  control_toolbox
)

# Trajectory Smoother Library
add_library(trajectory_smoother SHARED
  src/trajectory_smoother.cpp
)
ament_target_dependencies(trajectory_smoother ${DEPENDENCIES})
target_link_libraries(trajectory_smoother Eigen3::Eigen)

# Safety Checker Library
add_library(safety_checker SHARED
  src/safety_checker.cpp
)
ament_target_dependencies(safety_checker ${DEPENDENCIES})
target_link_libraries(safety_checker Eigen3::Eigen)

# Differential IK Controller Library
add_library(differential_ik_controller SHARED
  src/differential_ik_controller.cpp
)
ament_target_dependencies(differential_ik_controller ${DEPENDENCIES})
target_link_libraries(differential_ik_controller 
  Eigen3::Eigen
  trajectory_smoother
  safety_checker
)

# Virtual Arm Follower Library
add_library(virtual_arm_follower SHARED
  src/virtual_arm_follower.cpp
)
ament_target_dependencies(virtual_arm_follower ${DEPENDENCIES})
target_link_libraries(virtual_arm_follower Eigen3::Eigen)

# Teleoperation Node Executable
add_executable(teleoperation_node
  src/teleoperation_node.cpp
)
ament_target_dependencies(teleoperation_node ${DEPENDENCIES})
target_link_libraries(teleoperation_node
  differential_ik_controller
  virtual_arm_follower
  trajectory_smoother
  safety_checker
  Eigen3::Eigen
)

# Install targets
install(TARGETS
  trajectory_smoother
  safety_checker
  differential_ik_controller
  virtual_arm_follower
  teleoperation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
