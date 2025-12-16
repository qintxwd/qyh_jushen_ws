# qyh_dual_arm_ik_solver

双臂逆运动学求解节点 - VR遥操作专用

## 🎯 功能

- **第二客户端连接**：作为第二个客户端连接到JAKA控制器（qyh_jaka_control是第一个）
- **高频IK求解**：125Hz频率实时求解双臂逆运动学
- **无缝集成**：订阅VR目标位姿，发布关节指令

## 🏗️ 架构定位

```
VR → coordinate_mapper → /teleop/left_hand/target
                              ↓
                    [dual_arm_ik_solver] ← 第二个JAKA连接
                              ↓
                    /left_arm/joint_command
                              ↓
                    qyh_jaka_control (伺服控制)
```

## 📋 节点信息

### 订阅话题
- `/teleop/left_hand/target` (geometry_msgs/PoseStamped) - 左手目标位姿
- `/teleop/right_hand/target` (geometry_msgs/PoseStamped) - 右手目标位姿

### 发布话题
- `/left_arm/joint_command` (sensor_msgs/JointState) - 左臂关节指令
- `/right_arm/joint_command` (sensor_msgs/JointState) - 右臂关节指令
- `/ik_solver/status` (std_msgs/Bool) - IK求解状态

### 参数
- `robot_ip`: JAKA控制器IP地址（默认: 192.168.2.200）
- `ik_rate`: IK求解频率（默认: 125.0 Hz）
- `auto_connect`: 自动连接控制器（默认: true）
- `use_tf_lookup`: 使用TF查询（默认: false）

## 🚀 使用方法

### 1. 编译
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_dual_arm_ik_solver
source install/setup.bash
```

### 2. 启动完整遥操作系统

**终端1 - JAKA控制节点（主连接）：**
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py
```

**终端2 - VR遥操作节点：**
```bash
ros2 launch qyh_dual_arm_teleop teleop.launch.py
```

**终端3 - IK求解节点（第二连接）：**
```bash
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
```

### 3. 验证运行

**查看IK统计：**
```bash
# 应该看到：
# 📊 IK统计: 左臂成功率=XX%, 右臂成功率=XX%, 总计=XXX次
```

**查看关节指令：**
```bash
ros2 topic echo /left_arm/joint_command
ros2 topic hz /left_arm/joint_command  # 应该约125Hz
```

## 🔧 配置说明

编辑 `config/ik_solver_params.yaml`：

```yaml
robot_ip: "192.168.2.200"  # 与qyh_jaka_control相同
ik_rate: 125.0             # 匹配伺服周期
auto_connect: true
use_tf_lookup: false       # 直接订阅话题更高效
```

## ⚠️ 重要说明

### 多客户端连接
- ✅ JAKA SDK支持**最多2个客户端**同时连接
- 🔌 qyh_jaka_control = 第一连接（主控制）
- 🔌 qyh_dual_arm_ik_solver = 第二连接（IK求解）
- ❌ 第三个连接会失败

### IK参考位置
- 使用**上一次成功的IK解**作为参考
- 初始参考位置：零位或张开姿态
- 确保IK解的连续性和稳定性

### 坐标系约定
- 输入位姿：`human_left/right_hand` 语义坐标
- robot_id: 0=左臂, 1=右臂
- 位置单位：米 (m) → 转换为毫米 (mm)
- 姿态：四元数 → 转换为欧拉角 (rad)

## 📊 性能指标

- **IK求解频率**: 125 Hz
- **成功率**: >95% (工作空间内)
- **延迟**: <8ms (单次求解)

## 🐛 故障排查

### 问题1：连接失败
```
❌ 连接失败！错误码: -1
```
**原因**：qyh_jaka_control未运行或已有2个连接
**解决**：确认第一个节点运行正常，最多2个连接

### 问题2：IK成功率低
```
左臂IK失败 (错误码: -4)
```
**原因**：目标位姿超出工作空间或奇异点
**解决**：检查VR目标位姿是否合理

### 问题3：话题无数据
```bash
ros2 topic hz /left_arm/joint_command
# 显示: no messages received
```
**原因**：未收到VR目标位姿
**解决**：确认coordinate_mapper节点运行并发布目标

## 🔗 相关文档

- [TF_design.md](../TF_design.md) - VR遥操作架构设计
- [qyh_dual_arm_teleop/README.md](../qyh_dual_arm_teleop/README.md) - VR遥操作节点
- [qyh_jaka_control/README.md](../qyh_jaka_control/README.md) - JAKA控制节点
