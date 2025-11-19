#!/bin/bash
# 运行机器人监控 GUI

# 设置环境变量
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8

# Source ROS2 环境
source /opt/ros/humble/setup.bash
cd /mnt/e/work/yc/src/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

# 运行 GUI
ros2 run qyh_standard_robot_gui standard_robot_monitor
