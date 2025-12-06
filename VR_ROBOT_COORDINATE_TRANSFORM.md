# VR 到机器人坐标变换说明

## 概述

本文档说明如何将 **PICO 4 VR 手柄的位姿数据** 转换为 **机械臂末端的目标位姿**。

### 架构更新 (v2.0 - TF 坐标变换)

从 v2.0 开始，坐标变换在 `vr_bridge_node` (C++) 中完成：

```
┌─────────────────────────────────────────────────────────────────────┐
│                         vr_bridge_node (C++)                        │
├─────────────────────────────────────────────────────────────────────┤
│  UDP 9999 ─→ 解析 VR 数据包                                         │
│                  │                                                   │
│                  ├─→ /vr/left_hand/pose (原始 VR 坐标系)            │
│                  │   /vr/right_hand/pose                            │
│                  │                                                   │
│                  └─→ /vr/left_hand/pose_ros (ROS 坐标系)  ← 推荐    │
│                      /vr/right_hand/pose_ros                        │
│                                                                      │
│  变换内容:                                                           │
│  1. 坐标轴映射: R_z(-90°) · R_x(90°)                                │
│  2. Grip 补偿: R_x(-35°) (手柄自然握持角度)                          │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      vr_clutch_node (Python)                        │
├─────────────────────────────────────────────────────────────────────┤
│  订阅: /vr/xxx/pose_ros (已在 ROS 坐标系)                           │
│                                                                      │
│  功能:                                                               │
│  - Clutch 状态机 (按 Grip 跟踪，松开保持)                            │
│  - 增量累加 (位置在 base_link，旋转在末端)                           │
│  - 安全限制 (死区、最大增量)                                         │
│                                                                      │
│  发布: /sim/xxx_target_pose 或 /vr/xxx_target_pose                  │
└─────────────────────────────────────────────────────────────────────┘
```

**优势:**
- 坐标变换在 C++ 中完成，性能更好
- Python 层代码更简洁
- 符合 ROS TF 标准实践
- 参数可在 vr_bridge_node 中调整 (enable_transform, grip_offset_deg)

---

VR 遥操作需要完成三类转换：
1. **坐标系转换** - VR 坐标轴与机器人坐标轴的映射
2. **位置缩放** - 人臂长度与机械臂长度的比例
3. **姿态转换** - 四元数在不同坐标系间的变换

---

## 1. PICO VR 坐标系定义

### 1.1 原点和参考系
- **原点**: 头显开机时头部所在位置
- **初始方向**: 头显开机时头部面朝的方向
- **参考姿态**: 手柄柄部水平、指向与头部前方一致时，RPY = (0°, 0°, 0°)

### 1.2 VR 坐标轴定义 (右手坐标系)
```
        Y (上)
        ↑
        |
        |
        +------→ X (右)
       /
      /
     ↓
    Z (后)
    
注意: -Z 方向是"前"（头部面朝方向）
```

| 轴 | 正方向 | 说明 |
|---|--------|------|
| X | 右 | 用户右手边 |
| Y | 上 | 天空方向 |
| Z | 后 | **-Z 是前方（面朝方向）** |

### 1.3 VR 旋转定义 (RPY)
以手柄柄部水平、指向前方为零位：

| 旋转 | 轴 | 正方向动作 (人类直觉) |
|------|---|----------------------|
| Roll | X轴 | 手柄顺时针翻滚（从后往前看） |
| Pitch | Y轴 | 手柄抬头（柄部向上翘） |
| Yaw | Z轴 | 手柄向左转（俯视逆时针） |

> ⚠️ **重要警告**: 上表是"人类直觉"描述。PICO SDK 实际输出的欧拉角遵循**数学右手旋转定义**，可能与直觉相反：
> - Roll: 绕 X 右手旋转 = 向左倾斜为 +
> - Pitch: 绕 Y 右手旋转 = 向上抬头可能为 -
> - Yaw: 绕 Z 右手旋转 = 向右转可能为 +
>
> **建议**: 使用四元数直接变换，避免欧拉角方向歧义。

### 1.4 VR 数据格式
- **位置**: 米 (m)
- **姿态**: 四元数 (x, y, z, w)

---

## 2. 机器人坐标系定义 (ROS 标准)

### 2.1 机器人坐标轴 (右手坐标系)
```
        Z (上)
        ↑
        |
        |
        +------→ Y (左)
       /
      /
     ↓
    X (前)
```

| 轴 | 正方向 | 说明 |
|---|--------|------|
| X | 前 | 机器人前进方向 |
| Y | 左 | 机器人左侧 |
| Z | 上 | 天空方向 |

### 2.2 常见机械臂坐标系对比

| 系统 | X | Y | Z |
|------|---|---|---|
| PICO VR 手柄 | 右 | 上 | 后(-Z=前) |
| ROS 标准 / 本项目 | 前 | 左 | 上 |
| UR 机械臂 | 前 | 左 | 上 |
| JAKA 机械臂 | 前 | 左 | 上 |

---

## 3. 坐标变换理论

### 3.1 完整变换链

理论上，VR 手柄到机器人末端的完整变换链为：

$$
{}^{robot}T_{ee} = {}^{robot}T_{world} \cdot {}^{world}T_{HMD} \cdot {}^{HMD}T_{controller} \cdot T_{align}
$$

其中：
- ${}^{robot}T_{world}$ : 机器人基座相对世界坐标系
- ${}^{world}T_{HMD}$ : VR 头显相对世界坐标系
- ${}^{HMD}T_{controller}$ : 手柄相对头显
- $T_{align}$ : 手柄到末端的对齐旋转

### 3.2 本项目简化方案

由于我们使用 **Clutch 增量模式**，不需要绝对位置映射，只需要：

1. **坐标轴对齐矩阵** $R_{align}$
2. **位置缩放系数** $s$

增量变换公式：

$$
\Delta P_{robot} = R_{align} \cdot \Delta P_{vr} \cdot s
$$

$$
\Delta R_{robot} = R_{align} \cdot \Delta R_{vr} \cdot R_{align}^{-1}
$$

---

## 4. 坐标轴对齐矩阵推导

### 4.1 轴映射关系

| VR 轴 | VR 方向 | 对应机器人 | 机器人方向 | 映射 |
|-------|---------|-----------|-----------|------|
| X | 右 (+X) | -Y | 右 (-Y) | Robot_Y = -VR_X |
| Y | 上 (+Y) | +Z | 上 (+Z) | Robot_Z = +VR_Y |
| Z | 后 (+Z) | -X | 后 (-X) | Robot_X = -VR_Z |

### 4.2 对齐矩阵

$$
R_{align} = \begin{bmatrix} 
0 & 0 & -1 \\
-1 & 0 & 0 \\
0 & 1 & 0 
\end{bmatrix}
$$

这个矩阵满足：
- 行列式 = +1 (正交旋转矩阵 ✓)
- $R_{align}^{-1} = R_{align}^T$

### 4.3 等效旋转表示

该对齐矩阵等效于：

$$
R_{align} = R_z(-90°) \cdot R_x(90°)
$$

或用欧拉角表示（XYZ顺序）：
- Roll = 90°, Pitch = 0°, Yaw = -90°

---

## 5. 位置变换公式

### 5.1 增量位置变换

$$
\begin{bmatrix} \Delta X_{robot} \\ \Delta Y_{robot} \\ \Delta Z_{robot} \end{bmatrix} = 
R_{align} \cdot
\begin{bmatrix} \Delta X_{vr} \\ \Delta Y_{vr} \\ \Delta Z_{vr} \end{bmatrix}
\cdot s
$$

展开为：
$$
\begin{aligned}
\Delta X_{robot} &= -\Delta Z_{vr} \cdot s & \text{(VR前 → 机器人前)} \\
\Delta Y_{robot} &= -\Delta X_{vr} \cdot s & \text{(VR左 → 机器人左)} \\
\Delta Z_{robot} &= +\Delta Y_{vr} \cdot s & \text{(VR上 → 机器人上)}
\end{aligned}
$$

### 5.2 缩放系数

$$
s = \frac{L_{robot}}{L_{human}} = \frac{1.2m}{0.6m} \approx 2.0
$$

| 参数 | 值 | 说明 |
|------|-----|------|
| $L_{robot}$ | ~1.2 m | JAKA ZU7 机械臂工作半径 |
| $L_{human}$ | ~0.6 m | 人类手臂长度 |
| $s$ | 2.0 | 缩放系数 |

### 5.3 坐标原点偏移 (VR头显 vs 机械臂基座)

VR 坐标系原点在**头显位置**，而机械臂基座在**肩膀根部**，存在物理偏移：

```
侧视图 (从右边看):
                     
    [头显] ←─ VR原点
       |
       | 18cm (向下)
       ↓
    [肩膀] ←─ 机械臂基座 (需要向后 14cm)
       |
      手臂
       |
      手
```

**偏移量** (在 VR 坐标系中):
| 方向 | VR 轴 | 偏移值 | 说明 |
|------|-------|--------|------|
| 向下 | -Y | -0.18 m | 头显比肩膀高 18cm |
| 向后 | +Z | +0.14 m | 肩膀比脸部后退 14cm |

**转换到机器人坐标系的偏移**:
$$
\text{offset}_{robot} = R_{align} \cdot \text{offset}_{vr} = R_{align} \cdot \begin{bmatrix} 0 \\ -0.18 \\ 0.14 \end{bmatrix}
$$

计算结果:
$$
\text{offset}_{robot} = \begin{bmatrix} -0.14 \\ 0 \\ -0.18 \end{bmatrix} \text{ (m)}
$$

即：机械臂基座相对于 VR 原点，在机器人坐标系中位于 **X=-0.14m (后方), Z=-0.18m (下方)**。

> ⚠️ **注意**: 在 Clutch 增量模式下，此偏移**不影响控制**，因为我们只跟踪增量。
> 但如果实现绝对位置映射或工作空间可视化时，需要考虑此偏移。

---

## 6. 姿态变换公式

### 6.1 增量旋转变换

VR 手柄的增量旋转需要变换到机器人坐标系：

$$
\Delta q_{robot} = R_{align} \cdot \Delta q_{vr} \cdot R_{align}^{-1}
$$

其中 $\Delta q$ 表示旋转增量（四元数形式）。

### 6.2 旋转向量变换

等效地，可以用旋转向量：

$$
\Delta \omega_{robot} = R_{align} \cdot \Delta \omega_{vr}
$$

### 6.3 ⚠️ 四元数变换注意事项

**四元数表示方式**：不同 SDK 输出的四元数含义可能不同：
- `q_world_controller` (主动旋转 Active Rotation)
- `q_controller_world` (被动旋转 Passive Rotation)

本文档假设 PICO SDK 输出的是 **主动旋转** 形式的四元数。

> ⚠️ **验证方法**：
> 1. 让手柄保持 "零姿态"（柄部水平、指向前方）
> 2. 检查此时四元数是否接近 (0, 0, 0, 1)
> 3. 如果四元数明显不是单位四元数，可能需要取逆或调整变换顺序

---

## 7. 代码实现

### 7.1 配置参数

```python
# 位置轴映射
axis_mapping = [2, 0, 1]   # Robot[i] 来自 VR[axis_mapping[i]]
axis_signs = [-1, -1, 1]   # 对应的符号

# 含义:
# Robot_X = VR[2] * (-1) = -VR_Z  (前)
# Robot_Y = VR[0] * (-1) = -VR_X  (左)  
# Robot_Z = VR[1] * (+1) = +VR_Y  (上)

# 缩放
position_scale = 2.0
rotation_scale = 1.0

# VR头显到肩膀的偏移 (VR坐标系, 单位: 米)
# 头显比肩膀高18cm, 肩膀比脸部后退14cm
vr_to_shoulder_offset = [0, -0.18, 0.14]  # [x, y, z] in VR coords
```

### 7.2 构建变换矩阵 (Python)

```python
import numpy as np
from scipy.spatial.transform import Rotation

def build_transform_matrix(axis_mapping, axis_signs):
    """根据轴映射构建坐标变换矩阵"""
    transform = np.zeros((3, 3))
    for robot_axis in range(3):
        vr_axis = axis_mapping[robot_axis]
        sign = axis_signs[robot_axis]
        transform[robot_axis, vr_axis] = sign
    return transform

# 构建对齐矩阵
R_align = build_transform_matrix([2, 0, 1], [-1, -1, 1])
# 结果:
# [[ 0,  0, -1],
#  [-1,  0,  0],
#  [ 0,  1,  0]]
```

### 7.3 位置增量变换

```python
def transform_position_delta(delta_vr, R_align, scale):
    """变换位置增量"""
    delta_robot = R_align @ delta_vr * scale
    return delta_robot
```

### 7.4 姿态增量变换

```python
def transform_rotation_delta(delta_q_vr, R_align_rot):
    """变换旋转增量"""
    # delta_q_vr: 四元数 [x, y, z, w]
    delta_rot_vr = Rotation.from_quat(delta_q_vr)
    
    # R_robot = R_align * R_vr * R_align^(-1)
    delta_rot_robot = R_align_rot * delta_rot_vr * R_align_rot.inv()
    
    return delta_rot_robot.as_quat()
```

---

## 8. 动作映射表

### 8.1 位置移动

| 人的动作 | VR 数据变化 | 机器人动作 |
|---------|------------|-----------|
| 手向前推 | Z 减小 (-ΔZ) | X 增大 → 向前 |
| 手向后拉 | Z 增大 (+ΔZ) | X 减小 → 向后 |
| 手向右移 | X 增大 (+ΔX) | Y 减小 → 向右 |
| 手向左移 | X 减小 (-ΔX) | Y 增大 → 向左 |
| 手向上抬 | Y 增大 (+ΔY) | Z 增大 → 向上 |
| 手向下压 | Y 减小 (-ΔY) | Z 减小 → 向下 |

### 8.2 姿态旋转

| 人的动作 | VR RPY 变化 | 机器人 RPY 变化 |
|---------|-------------|----------------|
| 手腕内旋（顺时针翻滚） | Roll + | Robot Roll - |
| 手腕外旋（逆时针翻滚） | Roll - | Robot Roll + |
| 手柄抬头（柄向上翘） | Pitch + | Robot Pitch - |
| 手柄低头（柄向下压） | Pitch - | Robot Pitch + |
| 手柄左转（俯视逆时针） | Yaw + | Robot Yaw + |
| 手柄右转（俯视顺时针） | Yaw - | Robot Yaw - |

---

## 9. Clutch 模式工作原理

### 9.1 为什么使用增量模式？

绝对位置映射的问题：
- 需要精确的 VR-机器人空间标定
- 工作空间不匹配（人臂 vs 机械臂）
- 安全性问题

**增量模式优势**：
- 无需标定
- 可以随时"重置"映射关系
- 更安全、更灵活

### 9.2 ⚠️ 重要：位置和姿态使用不同的参考系

**问题分析**：
- 位置：用户想的是"机器人往哪走" → **世界坐标概念**
- 姿态：用户想的是"末端怎么转" → **末端局部概念**

**解决方案**：

| 增量类型 | 参考坐标系 | 用户直觉 |
|---------|-----------|---------|
| 位置增量 | base_link (世界) | 向前推 → 机器人向 X+ 走 |
| 姿态增量 | 末端 (局部) | 旋转手腕 → 末端跟着转 |

**代码实现差异**：

```python
# 位置增量 - base_link 坐标系（左乘）
target_pos += delta_pos_base * scale

# 姿态增量 - 末端坐标系（右乘）
# target_new = target_old * delta_rot  (在末端局部坐标系下旋转)
target_rot = target_rot * delta_rot
```

**为什么姿态用右乘？**
- 左乘 `delta * target`：在世界坐标系下旋转（位置用这个）
- 右乘 `target * delta`：在局部坐标系下旋转（姿态用这个）

**效果**：
- 无论末端当前朝向哪里，用户"顺时针旋转手腕" → 末端顺时针旋转
- 符合操作直觉，像是"抓着末端在转"

### 9.3 状态机

```
┌─────────┐  grip > 0.8   ┌───────────┐
│  IDLE   │ ────────────→ │ ENGAGING  │
│(不跟踪) │               │ (初始化)  │
└─────────┘               └─────┬─────┘
     ↑                          │ 下一帧
     │                          ↓
     │    grip < 0.2     ┌───────────┐
     └─────────────────── │ TRACKING  │
                          │ (增量跟踪)│
                          └───────────┘
```

### 9.4 增量计算流程（混合坐标系基准）

```python
# 每帧执行
if clutch_engaged:
    # 1. VR数据转换到base_link坐标系
    vr_pos_base = R_align @ vr_pos_vr
    vr_rot_base = R_align * vr_rot_vr * R_align.inv()
    
    # 2. 位置增量 - 在 base_link 坐标系下
    delta_pos_base = vr_pos_base - prev_vr_pos_base
    target_pos += delta_pos_base * scale  # 世界坐标增量
    
    # 3. 姿态增量 - 在末端坐标系下
    delta_rot_base = vr_rot_base * prev_vr_rot_base.inv()
    target_rot = target_rot * delta_rot_base  # 右乘=局部坐标旋转
    
    # 4. 更新上一帧
    prev_vr_pos_base = vr_pos_base
    prev_vr_rot_base = vr_rot_base
```

### 9.5 防抖动策略

| 策略 | 参数 | 说明 |
|------|------|------|
| 位置死区 | 2 mm | 小于此值的增量被累积 |
| 旋转死区 | 0.5° | 小于此值的增量被累积 |
| 累积式应用 | - | 多帧累积超过阈值后应用 |
| 单帧限幅 | 2 cm / 3° | 防止跳变 |
| 低通滤波 | α=0.5 | 平滑运动 |

---

## 10. 参数配置参考

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `position_scale` | 2.0 | 位置缩放 (机械臂/人臂) |
| `rotation_scale` | 1.0 | 旋转缩放 |
| `axis_mapping` | [2, 0, 1] | 轴映射 |
| `axis_signs` | [-1, -1, 1] | 轴符号 |
| `vr_to_shoulder_offset` | [0, -0.18, 0.14] | VR原点到肩膀偏移 (VR坐标系, m) |
| `engage_threshold` | 0.8 | Grip 接合阈值 |
| `release_threshold` | 0.2 | Grip 释放阈值 |
| `position_deadzone` | 0.002 m | 位置死区 |
| `rotation_deadzone` | 0.01 rad | 旋转死区 |
| `max_position_delta` | 0.02 m | 单帧最大位移 |
| `max_rotation_delta` | 0.05 rad | 单帧最大旋转 |

---

## 11. 调试命令

```bash
# 查看原始 VR 数据 (xyz + rpy 在 vr_bridge_node 日志中)
ros2 run qyh_vr_bridge vr_bridge_node

# 查看 VR 话题
ros2 topic echo /vr/left_hand/pose --once

# 查看转换后的目标 (xyz + rpy 在 vr_clutch_node 日志中)
ros2 run qyh_vr_calibration vr_clutch_node --ros-args -p simulation_mode:=true

# 查看目标话题
ros2 topic echo /sim/left_target_pose --once

# 查看 Clutch 状态
ros2 topic echo /vr/left_clutch_engaged --once
```

---

## 附录 A：变换矩阵验证

### 验证正交性

```python
R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

# 检查行列式
print(f"det(R) = {np.linalg.det(R)}")  # 应为 +1

# 检查正交性
print(f"R @ R.T = \n{R @ R.T}")  # 应为单位矩阵

# 检查逆矩阵
print(f"R.T = R^(-1): {np.allclose(R.T, np.linalg.inv(R))}")  # 应为 True
```

### 验证轴映射

```python
# VR 向前 (0, 0, -1) → Robot 向前 (1, 0, 0)
vr_forward = np.array([0, 0, -1])
robot_forward = R @ vr_forward
print(f"VR前 → Robot: {robot_forward}")  # [1, 0, 0] ✓

# VR 向右 (1, 0, 0) → Robot 向右 (0, -1, 0)
vr_right = np.array([1, 0, 0])
robot_right = R @ vr_right
print(f"VR右 → Robot: {robot_right}")  # [0, -1, 0] ✓

# VR 向上 (0, 1, 0) → Robot 向上 (0, 0, 1)
vr_up = np.array([0, 1, 0])
robot_up = R @ vr_up
print(f"VR上 → Robot: {robot_up}")  # [0, 0, 1] ✓
```

---

## 附录 B：其他机械臂适配

如果使用不同的机械臂，可能需要调整 `axis_mapping` 和 `axis_signs`：

| 机械臂 | axis_mapping | axis_signs | 说明 |
|--------|--------------|------------|------|
| JAKA / UR / ROS标准 | [2, 0, 1] | [-1, -1, 1] | X前Y左Z上 |
| 某些旧版机械臂 | [2, 0, 1] | [-1, 1, 1] | X前Y右Z上 |

修改参数后重启节点即可生效。

---

## 附录 C：坐标系验证测试程序

### C.1 一键测试脚本

```python
#!/usr/bin/env python3
"""
VR 到机器人坐标系验证测试程序
用于确认坐标变换是否正确
"""
import numpy as np
from scipy.spatial.transform import Rotation

# 对齐矩阵
R_align = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]
])

def verify_position_transform():
    """验证位置变换"""
    print("=" * 50)
    print("位置变换验证")
    print("=" * 50)
    
    test_cases = [
        ("VR 向前 (0,0,-1)", np.array([0, 0, -1]), np.array([1, 0, 0]), "Robot 前"),
        ("VR 向后 (0,0,+1)", np.array([0, 0, 1]), np.array([-1, 0, 0]), "Robot 后"),
        ("VR 向右 (+1,0,0)", np.array([1, 0, 0]), np.array([0, -1, 0]), "Robot 右"),
        ("VR 向左 (-1,0,0)", np.array([-1, 0, 0]), np.array([0, 1, 0]), "Robot 左"),
        ("VR 向上 (0,+1,0)", np.array([0, 1, 0]), np.array([0, 0, 1]), "Robot 上"),
        ("VR 向下 (0,-1,0)", np.array([0, -1, 0]), np.array([0, 0, -1]), "Robot 下"),
    ]
    
    all_pass = True
    for name, vr_vec, expected, desc in test_cases:
        result = R_align @ vr_vec
        passed = np.allclose(result, expected)
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{name} → {result} (期望: {expected} {desc}) {status}")
        if not passed:
            all_pass = False
    
    return all_pass

def verify_rotation_transform():
    """验证旋转变换"""
    print("\n" + "=" * 50)
    print("旋转变换验证")
    print("=" * 50)
    
    R_align_rot = Rotation.from_matrix(R_align)
    
    # 测试：VR 绕 Z 轴旋转 90° → Robot 绕 X 轴旋转
    test_rotations = [
        ("VR Yaw +90° (绕Z)", Rotation.from_euler('z', 90, degrees=True)),
        ("VR Pitch +90° (绕Y)", Rotation.from_euler('y', 90, degrees=True)),
        ("VR Roll +90° (绕X)", Rotation.from_euler('x', 90, degrees=True)),
    ]
    
    for name, vr_rot in test_rotations:
        robot_rot = R_align_rot * vr_rot * R_align_rot.inv()
        euler_robot = robot_rot.as_euler('xyz', degrees=True)
        print(f"{name} → Robot RPY: {euler_robot.round(1)}")

def test_with_real_data(vr_quat):
    """
    用实际 VR 数据测试
    vr_quat: [x, y, z, w] 格式的四元数
    """
    print("\n" + "=" * 50)
    print("实际数据测试")
    print("=" * 50)
    
    R_align_rot = Rotation.from_matrix(R_align)
    vr_rot = Rotation.from_quat(vr_quat)
    
    # 方法1: R * q * R^(-1)
    robot_rot_1 = R_align_rot * vr_rot * R_align_rot.inv()
    
    # 方法2: R * q * R^T (等价，因为 R 是正交阵)
    robot_rot_2 = R_align_rot * vr_rot * R_align_rot.inv()
    
    # 如果 SDK 是 passive rotation，需要取逆
    robot_rot_inv = R_align_rot * vr_rot.inv() * R_align_rot.inv()
    
    print(f"VR 四元数: {vr_quat}")
    print(f"VR 欧拉角 (XYZ): {vr_rot.as_euler('xyz', degrees=True).round(2)}")
    print(f"Robot 四元数 (主动): {robot_rot_1.as_quat().round(4)}")
    print(f"Robot 欧拉角 (主动): {robot_rot_1.as_euler('xyz', degrees=True).round(2)}")
    print(f"Robot 欧拉角 (被动): {robot_rot_inv.as_euler('xyz', degrees=True).round(2)}")

if __name__ == "__main__":
    # 验证位置
    pos_ok = verify_position_transform()
    
    # 验证旋转
    verify_rotation_transform()
    
    # 用零姿态测试
    print("\n--- 零姿态测试 ---")
    test_with_real_data([0, 0, 0, 1])
    
    # 示例：用实际 VR 数据测试（替换为你的数据）
    # test_with_real_data([0.1, 0.2, 0.3, 0.9])
    
    print("\n" + "=" * 50)
    if pos_ok:
        print("✅ 位置变换验证通过！")
    else:
        print("❌ 位置变换有误，请检查！")
```

### C.2 ROS2 实时验证节点

```python
#!/usr/bin/env python3
"""
ROS2 节点：实时显示 VR 数据并验证坐标变换
运行: ros2 run your_package vr_transform_test
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation

class VRTransformTest(Node):
    def __init__(self):
        super().__init__('vr_transform_test')
        
        self.R_align = np.array([
            [0, 0, -1],
            [-1, 0, 0],
            [0, 1, 0]
        ])
        self.R_align_rot = Rotation.from_matrix(self.R_align)
        
        self.sub = self.create_subscription(
            PoseStamped,
            '/vr/left_hand/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("VR Transform Test Node Started")
        self.get_logger().info("Move VR controller and observe the output...")
    
    def pose_callback(self, msg):
        # VR 位置
        vr_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # VR 四元数
        vr_quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        vr_rot = Rotation.from_quat(vr_quat)
        vr_euler = vr_rot.as_euler('xyz', degrees=True)
        
        # 变换到 Robot
        robot_pos = self.R_align @ vr_pos
        robot_rot = self.R_align_rot * vr_rot * self.R_align_rot.inv()
        robot_euler = robot_rot.as_euler('xyz', degrees=True)
        
        # 打印
        self.get_logger().info(
            f"\n--- VR ---\n"
            f"  Pos: X={vr_pos[0]:.3f} Y={vr_pos[1]:.3f} Z={vr_pos[2]:.3f}\n"
            f"  RPY: R={vr_euler[0]:.1f}° P={vr_euler[1]:.1f}° Y={vr_euler[2]:.1f}°\n"
            f"--- Robot ---\n"
            f"  Pos: X={robot_pos[0]:.3f} Y={robot_pos[1]:.3f} Z={robot_pos[2]:.3f}\n"
            f"  RPY: R={robot_euler[0]:.1f}° P={robot_euler[1]:.1f}° Y={robot_euler[2]:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = VRTransformTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C.3 验证步骤

1. **位置验证**:
   - 手向前推 → 观察 Robot X 是否增大
   - 手向右移 → 观察 Robot Y 是否减小
   - 手向上抬 → 观察 Robot Z 是否增大

2. **旋转验证**:
   - 手柄向左转 (Yaw) → 观察 Robot 的哪个轴变化
   - 手柄抬头 (Pitch) → 观察 Robot 的哪个轴变化
   - 手柄翻滚 (Roll) → 观察 Robot 的哪个轴变化

3. **如果发现方向相反**:
   - 位置: 调整对应的 `axis_signs`
   - 旋转: 可能需要将 `vr_rot.inv()` 或调整变换顺序
