#!/bin/bash
source /opt/ros/humble/setup.bash
cd /mnt/d/work/yc/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash
ros2 launch qyh_gripper_control gripper_control_left.launch.py
