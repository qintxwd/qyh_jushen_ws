#!/bin/bash
# 启动可视化测试模式 - 仅在RViz中显示，不控制真实机器人
# 
# 使用方法:
# 1. 在一个终端运行此脚本: ./start_visualization_test.sh
# 2. 在另一个终端启动RViz: rviz2 -d $(ros2 pkg prefix qyh_dual_arms_description)/share/qyh_dual_arms_description/rviz/dual_arms.rviz
# 3. 在第三个终端发送手柄指令测试

source install/setup.bash

echo "========================================"
echo "  启动可视化测试模式"
echo "  - 伺服指令仅发布到 /joint_states"
echo "  - 不会发送到真实机器人"
echo "  - 请在另一个终端打开 RViz2"
echo "========================================"

ros2 launch qyh_jaka_control jaka_control.launch.py \
    visualization_only:=true \
    cycle_time_ms:=8.0 \
    buffer_size:=10
