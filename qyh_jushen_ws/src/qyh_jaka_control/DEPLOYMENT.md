# 部署说明

## ⚠️ 编译平台要求

此包**必须在 Jetson (ARM64) 平台**上编译，因为使用了JAKA SDK的ARM64版本库：
- `thirdparty/lib/libjakaAPI_2_3_0_13.so` (ARM64)

### 在x86_64/WSL上编译会失败
```
/usr/bin/ld: .../libjakaAPI_2_3_0_13.so: error adding symbols: file in wrong format
```

这是**正常现象**，不影响代码正确性。

## 🚀 在Jetson上编译

### 1. 将代码传输到Jetson

```bash
# 从开发机打包
cd ~/qyh_jushen_ws
tar czf jaka_control_update.tar.gz qyh_jushen_ws/src/qyh_jaka_control

# 传输到Jetson
scp jaka_control_update.tar.gz jetson@<IP>:~/

# 在Jetson上解压
ssh jetson@<IP>
tar xzf jaka_control_update.tar.gz -C ~/
```

### 2. 在Jetson上编译

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qyh_jaka_control
```

### 3. 更新配置

编辑 `config/robot_config.yaml`:

```yaml
ik_solver:
  enabled: true  # 启用IK模式
  target_x_left: false
  has_z_offset: true
  left_z_offset: 0.219885132
  right_z_offset: 0.217950931
```

### 4. 测试

```bash
# 启动节点
ros2 launch qyh_jaka_control jaka_control.launch.py

# 在另一个终端发布VR目标
ros2 topic pub /teleop/left_hand/target geometry_msgs/PoseStamped "{...}" --once
```

## 📋 代码变更清单

### ✅ 已完成
- [x] 添加TF2头文件
- [x] 添加IK相关成员变量（tf_buffer_, tf_listener_, 目标位姿等）
- [x] 添加IK参数初始化
- [x] 实现 `leftVRTargetCallback()` / `rightVRTargetCallback()`
- [x] 实现 `solveLeftArmIK()` / `solveRightArmIK()`
- [x] 实现 `checkJointLimits()`
- [x] 更新 `robot_config.yaml` 配置
- [x] 创建集成说明文档

### 🔧 架构优势
1. **单一连接** - 只有一个节点连接JAKA控制器
2. **内部调用** - IK结果直接调用 `left_bridge_->addCommand()`，无话题延迟
3. **模式切换** - `ik_solver.enabled` 参数控制Bridge/IK模式
4. **向后兼容** - 保留所有现有功能

### 📦 依赖检查
所有依赖已在 `CMakeLists.txt` 和 `package.xml` 中：
- ✅ tf2
- ✅ tf2_ros
- ✅ tf2_geometry_msgs
- ✅ geometry_msgs

## 🐞 故障排查

### 编译错误：file in wrong format
**原因**: 在x86_64平台编译ARM64库
**解决**: 在Jetson上编译

### 运行时：IK求解失败
**检查**:
```bash
# 1. TF树是否正确
ros2 run tf2_tools view_frames.py

# 2. 是否收到VR目标
ros2 topic hz /teleop/left_hand/target

# 3. 查看节点日志
ros2 node info /jaka_control_node
```

### 切换回标准模式
编辑 `config/robot_config.yaml`:
```yaml
ik_solver:
  enabled: false  # 禁用IK模式
```

重启节点即可恢复到原来的Bridge模式。

## 📝 后续工作

- [ ] 更新启动脚本 `start_vr_real_robot.sh` 使用新的IK模式
- [ ] 废弃 `qyh_dual_arm_ik_solver` 包（可选）
- [ ] 添加实际测试记录到文档
