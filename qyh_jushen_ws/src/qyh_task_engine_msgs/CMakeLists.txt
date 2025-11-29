cmake_minimum_required(VERSION 3.8)
project(qyh_task_engine_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# 生成消息、服务和动作
rosidl_generate_interfaces(${PROJECT_NAME}
  # 消息
  "msg/TaskStatus.msg"
  "msg/NodeStatus.msg"
  "msg/SkillParameter.msg"
  # 服务
  "srv/ExecuteTask.srv"
  "srv/PauseTask.srv"
  "srv/ResumeTask.srv"
  "srv/CancelTask.srv"
  "srv/GetTaskStatus.srv"
  # 动作
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs builtin_interfaces action_msgs
)

ament_package()
