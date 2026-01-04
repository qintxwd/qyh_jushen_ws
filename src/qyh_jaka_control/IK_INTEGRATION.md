# IK求解器集成说明

## 🎯 功能概述

已将 `qyh_dual_arm_ik_solver` 的IK求解功能**完全集成**到 `qyh_jaka_control` 节点中，使用**单一连接**到JAKA控制器。

## ⚡ 最新更新（实时IK参考）

**2024-12-18**: 移除预设参考关节角度，**直接使用当前真实机械臂角度作为IK参考**！

- ❌ 移除：`JAKA_ZU7_REF_DEFAULT_JOINT_LEFT/RIGHT` 常量
- ❌ 移除：`ref_left_joints_`, `current_left_joints_` 等缓存变量
- ✅ 改为：每次IK求解时实时调用 `jaka_interface_.get_joint_position()` 获取当前关节角度

**优势**：
- 更准确 - 使用实时关节状态，避免累积误差
- 更简洁 - 减少状态管理，代码更清晰
- 更鲁棒 - 不依赖初始参考位置，适应任意起始姿态

## 📐 架构变化

### 之前（两个连接）
```
VR → coordinate_mapper → /teleop/left_hand/target
                              ↓
                    [dual_arm_ik_solver] ← 连接2（IK求解）
                              ↓
                    /left_arm/joint_command
                              ↓
                    [jaka_control] ← 连接1（伺服控制）
```

### 现在（单一连接）✅
```
VR → coordinate_mapper → /teleop/left_hand/target
                              ↓
                    [jaka_control] ← 单一连接
                         ↓          ↓
                      IK求解   伺服控制
```

## 🚀 使用方法

### 1. 启用IK模式

编辑 `config/robot_config.yaml`:

```yaml
ik_solver:
  enabled: true  # ⭐ 启用IK模式
  target_x_left: false
  has_z_offset: true
  left_z_offset: 0.219885132
  right_z_offset: 0.217950931
```

### 2. 启动节点

```bash
ros2 launch qyh_jaka_control jaka_control.launch.py
```

### 3. 发布VR目标位姿

```bash
# 使用coordinate_mapper或直接发布
ros2 topic pub /teleop/left_hand/target geometry_msgs/PoseStamped "..." --once
```

## 🔧 工作模式

### IK模式 (`ik_solver.enabled: true`)
- 订阅: `/teleop/left_hand/target`, `/teleop/right_hand/target` (PoseStamped)
- 执行: VR位姿 → TF转换 → 末端校正 → JAKA IK → 关节指令
- 输出: 内部调用 `left_bridge_->addCommand()` (不走话题)

### 标准Bridge模式 (`ik_solver.enabled: false`)
- 订阅: `/left_arm/joint_command`, `/right_arm/joint_command` (JointState)
- 执行: 关节指令 → Bridge插值 → 伺服发送
- 兼容: 所有现有的测试脚本和工具

## ⚙️ 配置参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `ik_solver.enabled` | 启用IK模式 | `false` |
| `ik_solver.target_x_left` | 目标坐标系X轴向左 | `false` (X向前) |
| `ik_solver.has_z_offset` | 应用Z轴偏移 | `true` |
| `ik_solver.left_z_offset` | 左臂Z偏移(m) | `0.219885132` |
| `ik_solver.right_z_offset` | 右臂Z偏移(m) | `0.217950931` |

## 🔍 调试

### 查看IK统计
节点日志会每100次错误输出一次统计信息。

### 测试IK模式
```bash
# 1. 启用IK模式
# 编辑 config/robot_config.yaml, 设置 ik_solver.enabled: true

# 2. 启动节点
ros2 launch qyh_jaka_control jaka_control.launch.py

# 3. 使用测试脚本发布目标位姿
# (从 dual_arm_ik_solver 复制测试脚本)
ros2 run qyh_dual_arm_ik_solver test_publish_targets.py
```

### 切换回标准模式
```bash
# 编辑 config/robot_config.yaml
ik_solver:
  enabled: false  # ⭐ 禁用IK模式
```

## ✅ 优势

1. **单一连接** - 避免多客户端调用冲突
2. **更低延迟** - IK→Bridge内部调用，无话题传输开销
3. **代码集中** - 所有控制逻辑在一个节点中
4. **兼容性好** - 保留两种模式，可随时切换
5. **易于维护** - 减少节点数量，简化架构

## 📝 注意事项

1. **不再需要** `qyh_dual_arm_ik_solver` 节点
2. TF树必须正确设置（`vr_origin` → `base_link_left/right`）
3. IK模式下会自动进行关节限位检查
4. 保留了所有原有的安全机制

## 🐛 故障排查

**问题：IK求解失败**
- 检查TF树: `ros2 run tf2_tools view_frames.py`
- 检查目标位姿是否合理
- 查看日志中的错误计数

**问题：没有运动**
- 确认 `ik_solver.enabled: true`
- 确认伺服模式已启动
- 检查是否收到VR目标位姿: `ros2 topic hz /teleop/left_hand/target`

**问题：想切回标准模式**
- 设置 `ik_solver.enabled: false`
- 重启节点
- 使用关节指令话题 `/left_arm/joint_command`
