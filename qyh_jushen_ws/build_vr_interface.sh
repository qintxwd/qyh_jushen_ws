#!/bin/bash
# Build VR interface package in WSL

cd ~/qyh_jushen_ws/qyh_jushen_ws

echo "Building qyh_vr_calibration package..."
colcon build --packages-select qyh_vr_calibration

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Build successful!"
    echo ""
    echo "Source the workspace:"
    echo "  source install/setup.bash"
    echo ""
    echo "Test VR interface with simulator:"
    echo "  ros2 launch qyh_vr_calibration test_vr_interface.launch.py"
else
    echo ""
    echo "✗ Build failed!"
    exit 1
fi
