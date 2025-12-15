#!/bin/bash
# 修复URDF中的gazebo_ros2_control插件配置

URDF_FILE="src/dual_arms/urdf/dual_arms-gazebo.urdf"

# 备份
cp "$URDF_FILE" "${URDF_FILE}.bak2"

# 使用Python修复
python3 << 'EOF'
with open("src/dual_arms/urdf/dual_arms-gazebo.urdf", "r") as f:
    lines = f.readlines()

with open("src/dual_arms/urdf/dual_arms-gazebo.urdf", "w") as f:
    for line in lines:
        if "$(find dual_arms)" in line:
            f.write("      <robot_param_node>robot_description</robot_param_node>\n")
        else:
            f.write(line)
EOF

echo "✓ URDF已修复"
