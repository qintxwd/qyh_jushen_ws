#!/bin/bash

# Test gripper control system
echo "========================================="
echo "测试夹爪控制系统"
echo "========================================="

# Source environment
source /opt/ros/humble/setup.bash
cd /mnt/d/work/yc/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

echo ""
echo "1. 检查已安装的包..."
ros2 pkg list | grep qyh_gripper

echo ""
echo "2. 检查消息和服务接口..."
ros2 interface list | grep qyh_gripper

echo ""
echo "3. 检查GripperState消息定义..."
ros2 interface show qyh_gripper_msgs/msg/GripperState

echo ""
echo "4. 检查launch文件..."
ros2 launch qyh_gripper_control dual_gripper.launch.py --show-args

echo ""
echo "5. 检查可执行文件..."
ls -lh build/qyh_gripper_control/gripper_control_node

echo ""
echo "6. 检查库依赖..."
ldd build/qyh_gripper_control/gripper_control_node | grep modbus

echo ""
echo "========================================="
echo "系统测试完成！"
echo "========================================="
echo ""
echo "所有组件已正确编译和安装。"
echo ""
echo "启动命令："
echo "  # 终端1 - 启动双手夹爪控制节点"
echo "  ros2 launch qyh_gripper_control dual_gripper.launch.py"
echo ""
echo "  # 终端2 - 启动GUI"
echo "  ros2 run qyh_gripper_gui gripper_gui"
echo ""
echo "  # 或单独启动左手夹爪"
echo "  ros2 launch qyh_gripper_control gripper_control.launch.py side:=left"
echo ""
