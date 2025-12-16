#!/bin/bash
# 双臂IK求解器测试脚本
# 用于Jetson实机环境

echo "=============================================="
echo "  双臂IK求解器 - 测试脚本"
echo "=============================================="
echo ""

# 检查qyh_jaka_control是否运行
echo "1. 检查JAKA控制节点状态..."
if ros2 node list | grep -q "jaka_control"; then
    echo "   ✅ qyh_jaka_control 正在运行"
else
    echo "   ❌ qyh_jaka_control 未运行！"
    echo "   请先启动: ros2 launch qyh_jaka_control jaka_control.launch.py"
    exit 1
fi

echo ""
echo "2. 启动IK求解节点..."
echo "   作为第二个客户端连接到192.168.2.200"
echo ""

ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
