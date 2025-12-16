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
# Include directories
include_directories(
  include
  ./thirdparty/include
)

# Resolve JAKA API shared library (Jetson ARM64 version)
set(JAKA_API_LIB "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/libjakaAPI_2_3_0_13.so")
if(NOT EXISTS ${JAKA_API_LIB})
  message(FATAL_ERROR "JAKA API shared library not found at ${JAKA_API_LIB}. Please verify thirdparty/lib contents.")
endif()

# Set RPATH so the executable can find the shared library at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/thirdparty")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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

# Install JAKA API shared library
install(FILES
  ${JAKA_API_LIB}
  DESTINATION lib/${PROJECT_NAME}/thirdparty
)

ament_package()
