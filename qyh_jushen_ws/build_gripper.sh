#!/bin/bash

# Build gripper packages
echo "========================================="
echo "编译夹爪控制系统"
echo "========================================="

# Source ROS2 environment
echo ""
echo "Source ROS2 Humble环境..."
source /opt/ros/humble/setup.bash

cd /mnt/d/work/yc/qyh_jushen_ws/qyh_jushen_ws

echo ""
echo "步骤1: 编译消息定义包..."
colcon build --packages-select qyh_gripper_msgs --symlink-install

echo ""
echo "步骤2: Source本地环境..."
source install/setup.bash

echo ""
echo "步骤3: 编译控制节点包..."
colcon build --packages-select qyh_gripper_control --symlink-install

echo ""
echo "步骤4: 编译GUI包..."
colcon build --packages-select qyh_gripper_gui --symlink-install

echo ""
echo "========================================="
echo "编译完成！"
echo "========================================="
echo ""
echo "使用方法："
echo "  1. Source环境:    source install/setup.bash"
echo "  2. 启动双手夹爪: ros2 launch qyh_gripper_control dual_gripper.launch.py"
echo "  3. 启动GUI:      ros2 run qyh_gripper_gui gripper_gui"
echo ""
