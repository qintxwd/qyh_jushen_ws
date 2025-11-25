cmake_minimum_required(VERSION 3.10)
project(jaka_servo_examples)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall")

# SDK根目录
set(JAKA_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

# 包含头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    "${JAKA_SDK_ROOT}/Linux/c&c++/inc_of_c++"
)

# 链接库目录 - 根据架构选择
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(JAKA_LIB_DIR "${JAKA_SDK_ROOT}/Linux/c&c++/x86_64-linux-gnu/shared")
    set(JAKA_LIB_NAME "jakaAPI_2_3_3")
    message(STATUS "Building for x86_64")
    message(STATUS "Library directory: ${JAKA_LIB_DIR}")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(JAKA_LIB_DIR "${JAKA_SDK_ROOT}/Linux/c&c++/aarch64-linux-gnu/shared")
    set(JAKA_LIB_NAME "jakaAPI_2_3_3")
    message(STATUS "Building for aarch64 (ARM64)")
    message(STATUS "Library directory: ${JAKA_LIB_DIR}")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 检查库文件是否存在
if(NOT EXISTS "${JAKA_LIB_DIR}/lib${JAKA_LIB_NAME}.so")
    message(FATAL_ERROR "JAKA library not found: ${JAKA_LIB_DIR}/lib${JAKA_LIB_NAME}.so")
endif()

link_directories(${JAKA_LIB_DIR})

# 30.edgservo - 关节伺服测试
add_executable(edgservo_test 
    30.edgservo.cpp
)
target_link_libraries(edgservo_test ${JAKA_LIB_NAME} pthread)

# 31.edgservo_fct - 力控伺服测试
add_executable(edgservo_fct_test 
    31.edgservo_fct.cpp
)
target_link_libraries(edgservo_fct_test ${JAKA_LIB_NAME} pthread)

# 32.controlloop - 控制环测试
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/32.controlloop.cpp")
    add_executable(controlloop_test 
        32.controlloop.cpp
    )
    target_link_libraries(controlloop_test ${JAKA_LIB_NAME} pthread)
endif()

# 23.multi_movj - 多臂MoveJ测试
add_executable(multi_movj_test 
    23.multi_movj.cpp
)
target_link_libraries(multi_movj_test ${JAKA_LIB_NAME} pthread)

# 24.multi_movl - 多臂MoveL测试
add_executable(multi_movl_test 
    24.multi_movl.cpp
)
target_link_libraries(multi_movl_test ${JAKA_LIB_NAME} pthread)

# 设置RPATH以便运行时找到库
set_target_properties(edgservo_test PROPERTIES
    INSTALL_RPATH "${JAKA_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

set_target_properties(edgservo_fct_test PROPERTIES
    INSTALL_RPATH "${JAKA_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# 安装规则
install(TARGETS edgservo_test edgservo_fct_test
    RUNTIME DESTINATION bin
)

# 打印配置信息
message(STATUS "========================================")
message(STATUS "JAKA SDK Configuration")
message(STATUS "========================================")
message(STATUS "SDK Root: ${JAKA_SDK_ROOT}")
message(STATUS "Include Dir: ${JAKA_SDK_ROOT}/Linux/c&c++/inc_of_c++")
message(STATUS "Library Dir: ${JAKA_LIB_DIR}")
message(STATUS "Library Name: ${JAKA_LIB_NAME}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Targets to build:")
message(STATUS "  - edgservo_test (关节伺服)")
message(STATUS "  - edgservo_fct_test (力控伺服)")
message(STATUS "  - multi_movj_test (多臂MoveJ)")
message(STATUS "  - multi_movl_test (多臂MoveL)")
message(STATUS "========================================")
