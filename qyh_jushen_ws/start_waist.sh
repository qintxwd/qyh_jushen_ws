#!/bin/bash
# 腰部电机控制节点启动脚本

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

# 启动bringup 并将日志输出到文件
export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

echo "🚀 启动腰部电机控制节点..."
ros2 launch qyh_waist_control waist_control.launch.py
