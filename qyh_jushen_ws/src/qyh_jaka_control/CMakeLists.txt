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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(qyh_jaka_control_msgs REQUIRED)
find_package(Protobuf REQUIRED)

# Include directories
include_directories(
  include
  ./thirdparty/include
  ${Protobuf_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ./thirdparty/lib
)

# Libraries
add_library(jaka_interface src/jaka_interface.cpp)
ament_target_dependencies(jaka_interface
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(jaka_interface
  jakaAPI
  ${Protobuf_LIBRARIES}
)

# Main control node (unified: basic + servo + VR)
add_executable(jaka_control_node src/jaka_control_node.cpp)
ament_target_dependencies(jaka_control_node
  rclcpp
  std_srvs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  qyh_jaka_control_msgs
)
target_link_libraries(jaka_control_node
  jaka_interface
  jakaAPI
  ${Protobuf_LIBRARIES}
)

install(TARGETS jaka_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  jaka_control_node
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

ament_package()