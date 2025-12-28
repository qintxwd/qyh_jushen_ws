cmake_minimum_required(VERSION 3.5)
project(qyh_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(qyh_teleop_node src/qyh_teleop.cpp)
target_link_libraries(qyh_teleop_node Eigen3::Eigen)
ament_target_dependencies(qyh_teleop_node rclcpp sensor_msgs geometry_msgs std_msgs tf2)

install(TARGETS
  qyh_teleop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
