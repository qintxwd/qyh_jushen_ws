cmake_minimum_required(VERSION 3.8)
project(qyh_gripper_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "srv/ActivateGripper.srv"
  "srv/MoveGripper.srv"
  "srv/GetGripperState.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
