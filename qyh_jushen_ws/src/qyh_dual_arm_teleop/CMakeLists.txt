cmake_minimum_required(VERSION 3.8)
project(qyh_dual_arm_teleop)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)

# Dependencies list
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# === Node 1: Teleop Manager ===
add_executable(teleop_manager_node src/teleop_manager_node.cpp)
ament_target_dependencies(teleop_manager_node ${dependencies})

# === Node 2: Coordinate Mapper ===
add_executable(coordinate_mapper_node src/coordinate_mapper_node.cpp)
ament_target_dependencies(coordinate_mapper_node ${dependencies})

# Install executables
install(TARGETS
  teleop_manager_node
  coordinate_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Testing disabled to avoid catkin_pkg dependency
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
