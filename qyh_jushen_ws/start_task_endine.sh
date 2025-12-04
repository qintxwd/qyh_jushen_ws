#!/bin/bash
# 任务引擎启动脚本

# 读取 ROS_DOMAIN_ID
ROS_DOMAIN_ID_FILE="$HOME/qyh_jushen_ws/persistent/ros/ROS_DOMAIN_ID"
if [ -f "$ROS_DOMAIN_ID_FILE" ]; then
    export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
else
    export ROS_DOMAIN_ID=0
fi
echo "🔧 ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# Source ROS2 环境
source /opt/ros/humble/setup.bash

# Source 工作空间
source "$HOME/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash"

# 启动任务引擎
echo "🚀 启动任务引擎..."
ros2 launch qyh_task_engine task_engine.launch.py
