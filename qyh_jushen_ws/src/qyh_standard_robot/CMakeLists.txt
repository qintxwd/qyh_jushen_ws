cmake_minimum_required(VERSION 3.8)
project(qyh_standard_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(qyh_standard_robot_msgs REQUIRED)

# Include directories
include_directories(
  include
  thirdparty/include
)

# Link directories
link_directories(
  thirdparty/lib/linux
)

# Add library
add_library(${PROJECT_NAME}_node SHARED
  src/standard_robot_node.cpp
)

target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  qyh_standard_robot_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  modbus
)

rclcpp_components_register_nodes(${PROJECT_NAME}_node "qyh_standard_robot::StandardRobotNode")

# Install
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
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
