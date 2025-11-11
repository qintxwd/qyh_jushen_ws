#!/usr/bin/env python3
# 修复URDF中的gazebo_ros2_control插件

urdf_file = 'src/Dual-Arms/urdf/Dual-Arms-gazebo.urdf'

with open(urdf_file, 'r', encoding='utf-8') as f:
    content = f.read()

# 找到并替换gazebo插件部分
old_plugin = '''  <!-- Gazebo ROS2 Control Plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <robot_param_node>robot_description</robot_param_node>
    </plugin>
  </gazebo>'''

new_plugin = '''  <!-- Gazebo ROS2 Control Plugin -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(arg controller_config)</parameters>
      <ros>
        <namespace>/</namespace>
      </ros>
    </plugin>
  </gazebo>'''

if old_plugin in content:
    content = content.replace(old_plugin, new_plugin)
    with open(urdf_file, 'w', encoding='utf-8') as f:
        f.write(content)
    print("✓ URDF插件配置已更新")
else:
    print("⚠️  未找到预期的插件配置，手动检查")
    print("查找: robot_param_node")
