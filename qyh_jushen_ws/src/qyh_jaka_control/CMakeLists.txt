cmake_minimum_required(VERSION 3.8)
project(qyh_jaka_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qyh_jaka_control_msgs REQUIRED)

add_executable(jaka_servo_node src/jaka_servo_node.cpp)
ament_target_dependencies(jaka_servo_node rclcpp qyh_jaka_control_msgs)

install(TARGETS jaka_servo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()