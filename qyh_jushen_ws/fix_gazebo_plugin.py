#!/usr/bin/env python3

urdf_file = 'src/dual_arms/urdf/dual_arms-gazebo.urdf'

with open(urdf_file, 'r') as f:
    lines = f.readlines()

# 找到并替换gazebo插件部分
output_lines = []
in_gazebo_plugin = False
skip_until_closing = False

for i, line in enumerate(lines):
    if '<gazebo>' in line and i > 1000:  # 确保是文件末尾的那个
        in_gazebo_plugin = True
        output_lines.append(line)
        output_lines.append('    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">\n')
        output_lines.append('      <ros>\n')
        output_lines.append('        <namespace>/</namespace>\n')
        output_lines.append('      </ros>\n')
        skip_until_closing = True
    elif '</gazebo>' in line and in_gazebo_plugin:
        output_lines.append('    </plugin>\n')
        output_lines.append(line)
        in_gazebo_plugin = False
        skip_until_closing = False
    elif skip_until_closing and ('<plugin' in line or '</plugin>' in line or 'robot_param_node' in line):
        continue  # 跳过旧的插件配置
    else:
        output_lines.append(line)

with open(urdf_file, 'w') as f:
    f.writelines(output_lines)

print("✓ Gazebo插件配置已更新")
