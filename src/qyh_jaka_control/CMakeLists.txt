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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(qyh_jaka_control_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(yaml-cpp REQUIRED)

# Detect system architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(JAKA_ARCH "arm64")
  message(STATUS "🔧 Detected ARM64 architecture")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
  set(JAKA_ARCH "x64")
  message(STATUS "🔧 Detected x64 architecture")
else()
  message(WARNING "⚠️ Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}, defaulting to arm64")
  set(JAKA_ARCH "arm64")
endif()

# Set architecture-specific paths
set(JAKA_THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/${JAKA_ARCH}")
set(JAKA_INCLUDE_DIR "${JAKA_THIRDPARTY_DIR}/include")
set(JAKA_LIB_DIR "${JAKA_THIRDPARTY_DIR}/lib")

message(STATUS "📁 JAKA Include directory: ${JAKA_INCLUDE_DIR}")
message(STATUS "📁 JAKA Library directory: ${JAKA_LIB_DIR}")

# Include directories
include_directories(
  ${JAKA_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Find JAKA API shared library
file(GLOB JAKA_API_LIBS "${JAKA_LIB_DIR}/libjakaAPI*.so")
if(NOT JAKA_API_LIBS)
  message(FATAL_ERROR "❌ JAKA API shared library not found in ${JAKA_LIB_DIR}. Please verify thirdparty/${JAKA_ARCH}/lib contents.")
endif()
list(GET JAKA_API_LIBS 0 JAKA_API_LIB)
get_filename_component(JAKA_API_LIB_NAME ${JAKA_API_LIB} NAME)
message(STATUS "✅ Found JAKA API library: ${JAKA_API_LIB_NAME}")

# Set RPATH so the executable can find the shared library at runtime
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/thirdparty")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Main control node (unified: basic + servo + VR)
add_executable(jaka_control_node 
  src/jaka_control_node.cpp
  src/jaka_service_handlers.cpp
)
ament_target_dependencies(jaka_control_node
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  qyh_jaka_control_msgs
  ament_index_cpp
  orocos_kdl
  kdl_parser
  urdf
  trac_ik_lib
)
target_link_libraries(jaka_control_node
  ${JAKA_API_LIB}
  ${orocos_kdl_LIBRARIES}
  yaml-cpp
)

install(TARGETS
  jaka_control_node
  # jaka_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/qyh_jaka_joint_adapter_node.py
  scripts/test_joint_adapter.py
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