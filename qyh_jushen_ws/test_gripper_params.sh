#!/bin/bash

echo "测试夹爪控制节点参数加载"
echo "======================================"

source /opt/ros/humble/setup.bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

echo ""
echo "1. 检查配置文件内容:"
echo "   Left gripper config:"
cat install/qyh_gripper_control/share/qyh_gripper_control/config/gripper_left.yaml

echo ""
echo "2. 启动left gripper节点（5秒后自动停止）..."
timeout 5 ros2 launch qyh_gripper_control gripper_control.launch.py side:=left 2>&1 &
LAUNCH_PID=$!

sleep 3

echo ""
echo "3. 检查节点参数..."
ros2 param list /left/left_gripper_control 2>/dev/null || echo "节点尚未就绪"

echo ""
echo "4. 获取device_port参数..."
ros2 param get /left/left_gripper_control device_port 2>/dev/null || echo "无法获取参数"

echo ""
wait $LAUNCH_PID 2>/dev/null
echo "测试完成"
