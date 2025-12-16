# qyh_coordinate_mapper

VR坐标映射与滤波节点 - VR遥操作数据处理核心

## 🎯 功能

- **坐标轴对齐**：VR坐标系 → 人手语义坐标系
- **低通滤波**：消除VR手抖和传感器噪声
- **速度限制**：限制线速度和角速度，确保安全
- **位置缩放**：调整操作范围和灵敏度
- **TF发布**：`vr_*_controller → human_*_hand`

## 🏗️ 架构定位

```
VR手柄 (原始数据, 有噪声)
    ↓ TF: vr_origin → vr_*_controller
    
[coordinate_mapper] ← ⭐ 数据处理核心
    ↓ 轴对齐 + 滤波 + 速度限制 + 缩放
    ↓ TF: vr_*_controller → human_*_hand
    ↓ Topic: /teleop/left_hand/target (平滑位姿)
    
[dual_arm_ik_solver]
    ↓ IK求解
[arm_controller]
```

## 📋 节点信息

### 输入
- **TF监听**：`vr_origin → vr_left_controller`
- **TF监听**：`vr_origin → vr_right_controller`

### 输出
- **TF发布**：`vr_left_controller → human_left_hand`
- **TF发布**：`vr_right_controller → human_right_hand`
- **话题**：`/teleop/left_hand/target` (geometry_msgs/PoseStamped)
- **话题**：`/teleop/right_hand/target` (geometry_msgs/PoseStamped)

### 参数
- `update_rate`: 更新频率（默认: 100.0 Hz）
- `position_scale`: 位置缩放因子（默认: 1.0，1.0=无缩放）
- `rotation_scale`: 旋转缩放因子（默认: 1.0）
- `filter_alpha`: 低通滤波系数（默认: 0.3，越小越平滑）
- `max_linear_velocity`: 最大线速度（默认: 0.5 m/s）
- `max_angular_velocity`: 最大角速度（默认: 1.0 rad/s）
- `enable_filter`: 启用滤波（默认: true）
- `enable_velocity_limit`: 启用速度限制（默认: true）

## 🚀 使用方法

### 1. 编译
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_coordinate_mapper
source install/setup.bash
```

### 2. 启动完整遥操作系统

**终端1 - VR数据接收：**
```bash
ros2 launch qyh_vr_bridge vr_bridge.launch.py
```

**终端2 - 坐标映射（本节点）：**
```bash
ros2 launch qyh_coordinate_mapper coordinate_mapper.launch.py
```

**终端3 - IK求解器：**
```bash
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
```

**终端4 - JAKA控制：**
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py
```

### 3. 调整参数

**降低滤波延迟（更灵敏但更抖）：**
```bash
ros2 launch qyh_coordinate_mapper coordinate_mapper.launch.py \
  filter_alpha:=0.5
```

**增加平滑度（更稳定但有延迟）：**
```bash
ros2 launch qyh_coordinate_mapper coordinate_mapper.launch.py \
  filter_alpha:=0.2
```

**缩小操作范围（慢速精细操作）：**
```bash
ros2 launch qyh_coordinate_mapper coordinate_mapper.launch.py \
  position_scale:=0.5
```

## 🔧 核心算法

### 1. 坐标轴对齐

VR坐标系（OpenXR标准）→ 人手语义坐标系

```cpp
// 轴映射关系
X_human(前) = -Z_vr(后)
Y_human(左) = -X_vr(右)
Z_human(上) = Y_vr(上)

// 旋转变换
R_human = R_align * R_vr * R_align^T

// 对齐矩阵
R_align = [
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0]
]
```

### 2. 低通滤波

**位置滤波（指数移动平均）：**
```cpp
pos_filtered = alpha * pos_new + (1 - alpha) * pos_prev
```

**旋转滤波（四元数球面插值Slerp）：**
```cpp
q_filtered = slerp(q_prev, q_new, alpha)
```

### 3. 速度限制

**线速度限制：**
```cpp
v_linear = ||Δpos|| / Δt
if (v_linear > max_linear_vel) {
    scale = max_linear_vel / v_linear
    pos_limited = pos_prev + Δpos * scale
}
```

**角速度限制：**
```cpp
angle = 2 * acos(|q_new · q_prev|)
v_angular = angle / Δt
if (v_angular > max_angular_vel) {
    t = max_angular_vel * Δt / angle
    q_limited = slerp(q_prev, q_new, t)
}
```

## 📊 性能指标

- **更新频率**: 100 Hz
- **滤波延迟**: ~30ms (alpha=0.3)
- **速度限制响应**: <10ms

## 🐛 故障排查

### 问题1：TF查询失败
```
vr_left_controller TF查询失败: frame not found
```
**原因**：VR数据接收节点未运行
**解决**：先启动 `ros2 launch qyh_vr_bridge vr_bridge.launch.py`

### 问题2：位姿抖动严重
```
机械臂运动不平滑
```
**原因**：滤波系数太大（alpha > 0.5）
**解决**：降低filter_alpha到0.2-0.3

### 问题3：响应延迟明显
```
VR手移动，机械臂延迟反应
```
**原因**：滤波系数太小（alpha < 0.2）
**解决**：提高filter_alpha到0.4-0.5

### 问题4：运动过快不安全
```
机械臂速度超出预期
```
**原因**：速度限制参数过大
**解决**：降低max_linear_velocity和max_angular_velocity

## ⚙️ 参数调优指南

### 滤波系数 (filter_alpha)
- **0.1-0.2**: 极其平滑，适合慢速精细操作
- **0.3-0.4**: 平衡（推荐）
- **0.5-0.7**: 快速响应，适合高速操作
- **0.8-1.0**: 几乎无滤波，保留原始数据

### 位置缩放 (position_scale)
- **0.3-0.5**: 慢速模式，精细操作
- **1.0**: 1:1映射（推荐）
- **1.5-2.0**: 快速模式，大范围操作

### 速度限制
- **保守**: max_linear_vel=0.3, max_angular_vel=0.5
- **标准**: max_linear_vel=0.5, max_angular_vel=1.0（推荐）
- **激进**: max_linear_vel=0.8, max_angular_vel=1.5

## 🔗 相关文档

- [TF_design.md](../TF_design.md) - VR遥操作架构设计
- [qyh_dual_arm_ik_solver/README.md](../qyh_dual_arm_ik_solver/README.md) - IK求解器
- [qyh_vr_bridge/README.md](../qyh_vr_bridge/README.md) - VR数据接收

## 📐 坐标系定义

```
VR坐标系（输入）:
  X → 右
  Y → 上
  Z → 后（朝向人）

人手语义坐标系（输出）:
  X → 前（手指方向）
  Y → 左（手掌左侧）
  Z → 上（手掌向上）
```

这个转换确保VR手柄的"向前推"映射为机械臂的"向前移动"。
