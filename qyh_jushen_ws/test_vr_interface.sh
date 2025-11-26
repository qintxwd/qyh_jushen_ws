#!/bin/bash
# Quick test script for VR interface

cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

echo "=========================================="
echo "VR Interface Quick Test"
echo "=========================================="
echo ""
echo "This will:"
echo "1. Launch VR simulator (circle motion)"
echo "2. Launch VR interface node"
echo "3. You can check output topics in another terminal:"
echo ""
echo "   ros2 topic list | grep vr"
echo "   ros2 topic echo /vr/left_target_pose"
echo "   ros2 topic hz /vr/left_target_pose"
echo ""
echo "Press Ctrl+C to stop"
echo ""
sleep 3

ros2 launch qyh_vr_calibration test_vr_interface.launch.py \
    motion_type:=circle \
    amplitude:=0.05 \
    frequency:=0.1
