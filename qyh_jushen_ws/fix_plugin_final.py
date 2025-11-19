#!/usr/bin/env python3

urdf_file = 'src/Dual-Arms/urdf/Dual-Arms-gazebo.urdf'

with open(urdf_file, 'r') as f:
    content = f.read()

# 找到最后的</ros2_control>标签位置
last_control_tag = content.rfind('</ros2_control>')

if last_control_tag != -1:
    # 分割为ros2_control之前的部分和之后的部分
    before = content[:last_control_tag + len('</ros2_control>')]
    
    # 添加正确的Gazebo插件配置
    gazebo_plugin = '''
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find dual_arms)/config/dual_arms_controllers.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
'''
    
    # 组合
    result = before + gazebo_plugin
    
    with open(urdf_file, 'w') as f:
        f.write(result)
    
    print("✓ 插件配置已修复")
else:
    print("✗ 找不到ros2_control标签")
