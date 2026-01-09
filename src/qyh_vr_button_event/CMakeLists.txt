cmake_minimum_required(VERSION 3.8)
project(qyh_vr_button_event)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(qyh_gripper_msgs REQUIRED)
find_package(qyh_lift_msgs REQUIRED)
find_package(qyh_waist_msgs REQUIRED)
find_package(qyh_standard_robot_msgs REQUIRED)

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

add_executable(qyh_vr_button_event
  src/qyh_vr_button_event.cpp)

ament_target_dependencies(qyh_vr_button_event
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  qyh_gripper_msgs
  qyh_lift_msgs
  qyh_waist_msgs
  qyh_standard_robot_msgs
  std_srvs)

install(TARGETS
  qyh_vr_button_event
  DESTINATION lib/${PROJECT_NAME})

# 安装launch和config文件夹
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)
