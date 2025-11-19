#!/bin/bash

echo "========================================="
echo "  仿真环境测试脚本"
echo "========================================="
echo ""

# Source环境
echo "📦 加载ROS2环境..."
source install/setup.bash

echo ""
echo "✓ 环境加载完成"
echo ""
echo "准备启动仿真..."
echo ""
echo "预期结果："
echo "  1. Gazebo打开，显示桌子和红色零件"
echo "  2. 双臂机器人生成在场景中"
echo "  3. 控制器自动加载（约5秒后）"
echo ""
echo "验证命令（在另一个终端运行）："
echo "  source install/setup.bash"
echo "  ros2 control list_controllers"
echo "  ros2 topic list | grep joint"
echo ""
echo "按Enter继续启动..."
read

ros2 launch sim_training_env training_sim.launch.py
