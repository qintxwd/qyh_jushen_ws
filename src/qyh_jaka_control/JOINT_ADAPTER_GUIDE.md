# JAKA关节名称适配器 - 使用说明

## 📌 问题背景

JAKA机械臂驱动发布的关节名称格式与URDF定义不一致：

| 数据源 | 关节名称格式 | Topic |
|--------|-------------|-------|
| JAKA驱动 | `left_jointN`, `right_jointN` <br> 或 `lN`, `rN` | `/joint_states_raw` |
| URDF定义 | `l-jN`, `r-jN` | - |
| robot_state_publisher需要 | `l-jN`, `r-jN` | `/joint_states` |

**解决方案**: 创建适配器节点 `qyh_jaka_joint_adapter`，自动转换关节名称。

---

## 🔧 组件说明

### 1. 适配器节点 (`qyh_jaka_joint_adapter_node.py`)

**功能**:
- 订阅 `/joint_states_raw` (来自 jaka_control_node)
- 自动检测关节名称格式（支持3种格式）
- 转换为URDF标准格式 (`l-j1` ~ `l-j7`, `r-j1` ~ `r-j7`)
- 发布到 `/joint_states` (供 robot_state_publisher 使用)

**支持的输入格式**:

| 格式 | 示例 | 转换规则 |
|------|------|---------|
| Format 1 | `left_joint1`, `right_joint1` | `left_jointN` → `l-jN` <br> `right_jointN` → `r-jN` |
| Format 2 | `l1`, `r1` | `lN` → `l-jN` <br> `rN` → `r-jN` |
| URDF native | `l-j1`, `r-j1` | 直接透传（无需转换） |

**自动检测逻辑**:
```python
# 适配器会自动检测第一条消息的格式，并应用相应的转换规则
# 无需手动配置
```

---

## 🚀 使用方法

### 方法1: 使用集成的launch文件（推荐）

```bash
# 一键启动（包含jaka_control + 适配器 + robot_state_publisher）
ros2 launch qyh_jaka_control jaka_control.launch.py

# 验证节点启动
ros2 node list
# 预期输出:
#   /jaka_control_node
#   /qyh_jaka_joint_adapter
#   /robot_state_publisher
```

### 方法2: 手动启动各节点

```bash
# 终端1: 启动JAKA控制节点
ros2 launch qyh_jaka_control jaka_control.launch.py

# 终端2: 单独启动适配器（如需调试）
ros2 run qyh_jaka_control qyh_jaka_joint_adapter_node.py

# 终端3: 单独启动robot_state_publisher（如需调试）
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat ~/qyh_jushen_ws/src/qyh_dual_arms_description/urdf/dual_arms.urdf)"
```

---

## ✅ 验证测试

### 快速验证

```bash
# 1. 检查原始数据
ros2 topic echo /joint_states_raw --once

# 2. 检查适配后的数据
ros2 topic echo /joint_states --once
# 预期: 关节名称为 l-j1, l-j2, ..., r-j1, r-j2, ...

# 3. 验证TF树
ros2 run tf2_ros tf2_echo base_link lt
# 预期: 输出左臂末端变换（无错误）
```

### 使用测试脚本（详细验证）

```bash
cd ~/qyh_jushen_ws/src/qyh_jaka_control/scripts
python3 test_joint_adapter.py
```

**测试脚本会检查**:
- ✅ 原始数据接收
- ✅ 适配数据接收
- ✅ 关节名称格式正确性
- ✅ 关节数量（14个）
- ✅ 数据完整性（位置/速度/力矩）

**预期输出**:
```
✓ Raw data received: 14 joints
✓ Adapted data received: 14 joints
✅ Joint names correctly converted to URDF format
✅ Joint count correct (14 joints)
✅ Position data preserved
🎉 ADAPTER WORKING CORRECTLY!
```

---

## 🐛 故障排查

### 问题1: 适配器无输出

**症状**: `/joint_states` topic无数据

**检查**:
```bash
# 1. 确认jaka_control_node正在发布
ros2 topic hz /joint_states_raw
# 应该有输出 (10-125 Hz)

# 2. 确认适配器节点运行
ros2 node list | grep adapter
# 应该看到 /qyh_jaka_joint_adapter

# 3. 查看适配器日志
ros2 node info /qyh_jaka_joint_adapter
```

### 问题2: robot_state_publisher报错 "joint not found"

**症状**: `Could not find joint 'l-j1' in robot model`

**原因**: 适配器未正确转换关节名称

**解决**:
```bash
# 1. 检查适配后的关节名称
ros2 topic echo /joint_states --once

# 2. 对比URDF中的关节名称
grep "joint name=" ~/qyh_jushen_ws/src/qyh_dual_arms_description/urdf/dual_arms.urdf

# 3. 如果格式不匹配，检查适配器日志
ros2 run rqt_console rqt_console
# 查找 "Detected format" 消息
```

### 问题3: TF树不完整

**症状**: `ros2 run tf2_ros tf2_echo base_link lt` 报错 "frame does not exist"

**检查**:
```bash
# 1. 确认robot_state_publisher订阅了正确的topic
ros2 node info /robot_state_publisher | grep Subscribers
# 应该看到: /joint_states

# 2. 查看TF发布情况
ros2 topic hz /tf
# 应该有输出

# 3. 生成TF树图
ros2 run tf2_tools view_frames
evince frames.pdf
# 检查 base_link → l1 → ... → lt 链条是否完整
```

---

## 📝 编译和安装

### 编译

```bash
cd ~/qyh_jushen_ws
colcon build --packages-select qyh_jaka_control --symlink-install

# 刷新环境
source install/setup.bash
```

### 确认安装

```bash
# 检查适配器脚本是否安装
ls install/qyh_jaka_control/lib/qyh_jaka_control/qyh_jaka_joint_adapter_node.py

# 检查launch文件是否更新
cat install/qyh_jaka_control/share/qyh_jaka_control/launch/jaka_control.launch.py | grep adapter
```

---

## 🎯 集成到现有系统

如果您已经有自定义launch文件，添加适配器的方法：

```python
# 在您的launch文件中添加:

# 1. 获取URDF
urdf_pkg = get_package_share_directory('qyh_dual_arms_description')
urdf_file = os.path.join(urdf_pkg, 'urdf', 'dual_arms.urdf')
with open(urdf_file, 'r') as f:
    robot_description = f.read()

# 2. 添加适配器节点
Node(
    package='qyh_jaka_control',
    executable='qyh_jaka_joint_adapter_node.py',
    name='qyh_jaka_joint_adapter',
    output='screen'
),

# 3. 添加robot_state_publisher
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
),
```

---

## 📚 相关文档

- [STEP_BY_STEP_VERIFICATION.md](./STEP_BY_STEP_VERIFICATION.md) - 系统验证手册
- [qyh_dual_arms_description/urdf/dual_arms.urdf](../qyh_dual_arms_description/urdf/dual_arms.urdf) - 机器人URDF定义
- [qyh_jaka_control/README.md](./README.md) - JAKA控制包文档

---

**版本**: v1.0  
**更新日期**: 2025-12-17  
**维护者**: AI Assistant
