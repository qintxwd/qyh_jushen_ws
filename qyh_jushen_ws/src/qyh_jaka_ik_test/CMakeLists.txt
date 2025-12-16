cmake_minimum_required(VERSION 3.8)
project(qyh_jaka_ik_test)

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

# 使用qyh_jaka_control的JAKA SDK
set(JAKA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../qyh_jaka_control/thirdparty/include")
set(JAKA_API_LIB "${CMAKE_CURRENT_SOURCE_DIR}/../qyh_jaka_control/thirdparty/lib/libjakaAPI_2_3_0_13.so")

if(NOT EXISTS ${JAKA_API_LIB})
  message(FATAL_ERROR "JAKA API library not found: ${JAKA_API_LIB}")
endif()

include_directories(
  include
  ${JAKA_INCLUDE_DIR}
)

# IK测试节点
add_executable(ik_test_node src/ik_test_node.cpp)
ament_target_dependencies(ik_test_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(ik_test_node
  ${JAKA_API_LIB}
)

install(TARGETS
  ik_test_node
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

ament_package()
