cmake_minimum_required(VERSION 3.8)
project(qyh_head_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  qyh_head_control/head_servo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME head_servo_node
)

install(PROGRAMS
  qyh_head_control/set_servo_id.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_servo_id
)

install(PROGRAMS
  scripts/test_head_servo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_head_servo
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
