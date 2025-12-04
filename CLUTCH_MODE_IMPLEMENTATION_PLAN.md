# VR遥操作 Clutch Mode 实现计划

## 📊 现有实现分析

### 当前架构

```
┌──────────────┐     ┌─────────────────────┐     ┌──────────────────────┐
│ qyh_vr_bridge│────>│ qyh_vr_calibration  │────>│ qyh_teleoperation    │
│ (UDP接收)    │     │ (vr_interface_node) │     │ _controller          │
└──────────────┘     └─────────────────────┘     └──────────────────────┘
      │                       │                           │
      │ 发布:                 │ 订阅/发布:                │ 订阅:
      │ /vr/left_hand/pose    │ /vr/left_hand/pose       │ /vr/left_target_pose
      │ /vr/right_hand/pose   │ /vr/right_hand/pose      │ /vr/right_target_pose
      │ /vr/*/joy             │ ──────────────>          │
      │                       │ /vr/left_target_pose     │ 发布:
      │                       │ /vr/right_target_pose    │ /left_arm/joint_command
      │                       │                          │ /right_arm/joint_command
```

### 现有代码问题分析

#### 1. qyh_vr_bridge (✅ 基本OK，需小改)
- ✅ UDP接收VR数据，发布ROS话题
- ✅ 发布 `/vr/*/pose`, `/vr/*/joy`, `/vr/*/active`
- ⚠️ **需要**: 发布grip值作为clutch按钮状态

#### 2. qyh_vr_calibration/vr_interface_node.py (❌ 需要大改)
- ❌ 当前的"hybrid mapping"不是真正的clutch模式
- ❌ `direct_mapping` 直接映射VR绝对位置 → 会导致初始跳变
- ❌ `incremental_mapping` 只跟随增量 → 但没有clutch按钮控制
- ❌ `hybrid_mapping` 混合模式 → 复杂且不直觉
- **需要重写**: 实现真正的Clutch Mode

#### 3. qyh_teleoperation_controller (⚠️ 需要适配)
- ✅ 差分IK求解基本正确
- ✅ 轨迹平滑和安全检查存在
- ⚠️ 但直接接收 `/vr/left_target_pose` 作为目标位姿
- **问题**: 如果vr_interface直接发布绝对位姿，差分IK会计算巨大速度
- **需要**: 确保接收的是经过clutch处理后的目标位姿

---

## 🎯 修改方案

### 方案选择: 在 `vr_interface_node` 中实现 Clutch Mode

**原因**:
1. 不需要修改底层的 `qyh_vr_bridge`（UDP协议不变）
2. 不需要修改 `qyh_teleoperation_controller`（仍然接收目标位姿）
3. 只需要重写 `vr_interface_node` 的映射逻辑
4. 网页前端只需要显示clutch状态，不需要大改

### 新架构

```
┌──────────────┐     ┌─────────────────────┐     ┌──────────────────────┐
│ qyh_vr_bridge│────>│ vr_interface_node   │────>│ teleoperation        │
│ (UDP接收)    │     │ (Clutch Controller) │     │ _controller          │
└──────────────┘     └─────────────────────┘     └──────────────────────┘
      │                       │                           │
      │ /vr/left_hand/pose    │ 状态机:                   │ 差分IK
      │ /vr/right_hand/pose   │ IDLE→ENGAGING→TRACKING   │ 轨迹平滑
      │ /vr/left_hand/joy     │ →RELEASING→IDLE         │ 安全检查
      │   axes[3]=grip        │                          │
      │                       │ Clutch逻辑:              │
      │                       │ grip>0.8: 建立参考       │
      │                       │ 跟随VR相对增量           │
      │                       │ grip<0.2: 保持位置       │
```

---

## 📝 详细修改清单

### Phase 1: 消息定义 (✅ 不需要大改)

#### 1.1 qyh_teleoperation_msgs/msg/TeleopStatus.msg
**状态**: 需要添加Clutch状态

```diff
  std_msgs/Header header

  uint8 IDLE = 0
  uint8 TRACKING = 1
  uint8 PAUSED = 2
  uint8 ERROR = 3
+ uint8 CLUTCH_ENGAGING = 4
+ uint8 CLUTCH_RELEASING = 5

  uint8 status
  string message
  float64 tracking_quality  # 0.0-1.0
+ 
+ # Clutch state for each arm
+ bool left_clutch_engaged
+ bool right_clutch_engaged
```

---

### Phase 2: vr_interface_node 重写 (核心修改)

#### 2.1 新建 `vr_clutch_controller.py`

**位置**: `qyh_vr_calibration/qyh_vr_calibration/vr_clutch_controller.py`

**功能**:
- 实现Clutch状态机
- 处理VR位姿 → 机器人目标位姿的映射
- 按住grip键建立参考，跟随增量

#### 2.2 重写 `vr_interface_node.py`

**改动**:
1. 移除 `direct_mapping`, `incremental_mapping`, `hybrid_mapping`
2. 使用 `VRClutchController` 替代
3. 订阅 `/vr/*/joy` 获取grip值作为clutch按钮
4. 发布clutch状态到 `/vr/clutch_status`

---

### Phase 3: 配置文件更新

#### 3.1 `vr_interface_params.yaml`

```yaml
vr_interface_node:
  ros__parameters:
    # Clutch Mode 配置
    clutch:
      engage_threshold: 0.8    # grip > 0.8 时接合clutch
      release_threshold: 0.2   # grip < 0.2 时释放clutch
      
      # 位置缩放
      position_scale: 1.0      # VR位移到机器人位移的缩放
      rotation_scale: 1.0      # VR旋转到机器人旋转的缩放
      
      # 单步最大增量限制
      max_position_delta: 0.05  # m/step
      max_rotation_delta: 0.1   # rad/step
    
    # 平滑滤波
    smoothing:
      position_window: 5
      orientation_window: 3
      
    # 死区
    deadzone:
      position: 0.002  # 2mm
      orientation: 0.01  # rad
```

---

### Phase 4: teleoperation_controller 小调整

#### 4.1 确保正确处理状态切换

当clutch释放时，需要确保：
1. 差分IK不会因为目标不变而继续发送指令
2. 安全检查通过时才发送指令

**修改** `teleoperation_node.cpp`:
- 添加对clutch状态的订阅
- 当clutch未接合时，不发送指令或发送当前位置保持指令

---

### Phase 5: 前端网页修改

#### 5.1 显示Clutch状态

在遥操作控制面板添加：
- 左/右手 Clutch 状态指示灯
- 实时显示 "接合中" / "已释放"

#### 5.2 可能的配置界面

- Clutch阈值配置
- 位置/旋转缩放配置

---

## 🔄 实施顺序

### 阶段 1: 核心Clutch逻辑 (优先) ✅ 已完成
1. ✅ 创建 `vr_clutch_controller.py` 
2. ✅ 创建 `vr_clutch_node.py` (新节点，替代vr_interface_node)
3. ✅ 创建 `vr_clutch_params.yaml`
4. ✅ 创建 `vr_clutch.launch.py`
5. ✅ 更新 `setup.py` 添加入口点
6. 🧪 待测试: 运行 vr_bridge + vr_clutch_node，用 `ros2 topic echo` 验证

### 阶段 2: 消息和状态 ✅ 已完成
7. ✅ 更新 `TeleopStatus.msg` 添加clutch字段
8. ✅ vr_clutch_node 发布clutch状态到 `/vr/*_clutch_engaged`

### 阶段 3: teleoperation_controller 适配 ✅ 已完成
9. ✅ teleoperation_node 订阅clutch状态 `/vr/*_clutch_engaged`
10. ✅ 添加 `std_msgs/msg/Bool` 支持
11. 🧪 待测试: 仿真中测试完整流程

### 阶段 4: 前端网页 ✅ 已完成
12. ✅ 后端API添加clutch状态 (`vr_teleoperation.py`)
13. ✅ ROS2桥接添加VR状态订阅 (`bridge.py`)
14. ✅ 前端显示clutch指示器 (`VRTeleoperationPanel.vue`)
15. ✅ 注册面板到布局系统 (`layout.ts`, `PanelContainer.vue`)
16. 🧪 测试: 完整系统测试

### 阶段 5: 真机测试和调优 ⏳ 待执行
15. 🤖 真机测试
16. 📊 参数调优
17. 📝 文档更新

---

## 📋 代码修改详情

### 1. 新文件: `vr_clutch_controller.py`

```python
#!/usr/bin/env python3
"""
VR Clutch Controller - 实现离合器模式的VR遥操作

工作原理:
1. 当grip按钮按下时(>0.8), 记录VR和机器人当前位姿作为参考
2. 之后跟踪VR相对于参考的增量变化
3. 当grip按钮松开时(<0.2), 保持机器人最后位置
"""

import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation


class ClutchState(Enum):
    IDLE = 0        # Clutch未接合，机器人保持位置
    ENGAGING = 1    # Clutch刚接合，建立参考
    TRACKING = 2    # Clutch接合中，跟踪VR增量
    RELEASING = 3   # Clutch刚释放


class VRClutchController:
    """单臂的Clutch控制器"""
    
    def __init__(self, config):
        self.config = config
        self.state = ClutchState.IDLE
        
        # 参考位姿
        self.vr_reference_pos = None
        self.vr_reference_ori = None
        self.robot_reference_pos = None
        self.robot_reference_ori = None
        
        # 当前目标
        self.last_robot_target_pos = None
        self.last_robot_target_ori = None
        
    def update(self, vr_pos, vr_ori, robot_current_pos, robot_current_ori, grip_value):
        """
        更新Clutch控制器
        
        Args:
            vr_pos: VR手柄位置 [x, y, z]
            vr_ori: VR手柄姿态 [x, y, z, w] quaternion
            robot_current_pos: 机器人当前末端位置 [x, y, z]
            robot_current_ori: 机器人当前末端姿态 [x, y, z, w]
            grip_value: Grip按钮值 [0.0, 1.0]
            
        Returns:
            (target_pos, target_ori, state): 机器人目标位姿和当前状态
        """
        engage_threshold = self.config.get('engage_threshold', 0.8)
        release_threshold = self.config.get('release_threshold', 0.2)
        
        clutch_pressed = grip_value > engage_threshold
        clutch_released = grip_value < release_threshold
        
        # 状态机转换
        if self.state == ClutchState.IDLE:
            if clutch_pressed:
                self._engage_clutch(vr_pos, vr_ori, robot_current_pos, robot_current_ori)
                self.state = ClutchState.ENGAGING
            # IDLE状态：保持最后目标位置
            return self._get_hold_target(robot_current_pos, robot_current_ori)
        
        elif self.state == ClutchState.ENGAGING:
            self.state = ClutchState.TRACKING
            # Fall through to TRACKING
        
        if self.state == ClutchState.TRACKING:
            if clutch_released:
                self.state = ClutchState.RELEASING
                return self._get_hold_target(robot_current_pos, robot_current_ori)
            
            # 计算VR增量并映射到机器人
            target_pos, target_ori = self._compute_target(vr_pos, vr_ori)
            self.last_robot_target_pos = target_pos
            self.last_robot_target_ori = target_ori
            return target_pos, target_ori, self.state
        
        elif self.state == ClutchState.RELEASING:
            if clutch_pressed:
                # 重新建立参考
                self._engage_clutch(vr_pos, vr_ori, robot_current_pos, robot_current_ori)
                self.state = ClutchState.TRACKING
                return self._compute_target(vr_pos, vr_ori) + (self.state,)
            
            self.state = ClutchState.IDLE
            return self._get_hold_target(robot_current_pos, robot_current_ori)
        
        return self._get_hold_target(robot_current_pos, robot_current_ori)
    
    def _engage_clutch(self, vr_pos, vr_ori, robot_pos, robot_ori):
        """建立VR和机器人的参考映射"""
        self.vr_reference_pos = np.array(vr_pos).copy()
        self.vr_reference_ori = np.array(vr_ori).copy()
        self.robot_reference_pos = np.array(robot_pos).copy()
        self.robot_reference_ori = np.array(robot_ori).copy()
        self.last_robot_target_pos = self.robot_reference_pos.copy()
        self.last_robot_target_ori = self.robot_reference_ori.copy()
    
    def _compute_target(self, vr_pos, vr_ori):
        """计算机器人目标位姿"""
        vr_pos = np.array(vr_pos)
        vr_ori = np.array(vr_ori)
        
        # 计算VR位置增量
        vr_delta_pos = vr_pos - self.vr_reference_pos
        
        # 应用位置缩放
        pos_scale = self.config.get('position_scale', 1.0)
        robot_delta_pos = vr_delta_pos * pos_scale
        
        # 限制单步最大位移
        max_delta = self.config.get('max_position_delta', 0.05)
        delta_norm = np.linalg.norm(robot_delta_pos)
        if delta_norm > max_delta:
            robot_delta_pos = robot_delta_pos / delta_norm * max_delta
        
        # 计算机器人目标位置
        target_pos = self.robot_reference_pos + robot_delta_pos
        
        # 计算VR姿态增量
        vr_ref_rot = Rotation.from_quat(self.vr_reference_ori)
        vr_cur_rot = Rotation.from_quat(vr_ori)
        vr_delta_rot = vr_cur_rot * vr_ref_rot.inv()
        
        # 应用旋转缩放
        rot_scale = self.config.get('rotation_scale', 1.0)
        if rot_scale != 1.0:
            # 缩放旋转角度
            rotvec = vr_delta_rot.as_rotvec()
            rotvec *= rot_scale
            vr_delta_rot = Rotation.from_rotvec(rotvec)
        
        # 限制单步最大旋转
        max_rot = self.config.get('max_rotation_delta', 0.1)
        rotvec = vr_delta_rot.as_rotvec()
        rot_angle = np.linalg.norm(rotvec)
        if rot_angle > max_rot:
            rotvec = rotvec / rot_angle * max_rot
            vr_delta_rot = Rotation.from_rotvec(rotvec)
        
        # 计算机器人目标姿态
        robot_ref_rot = Rotation.from_quat(self.robot_reference_ori)
        target_rot = vr_delta_rot * robot_ref_rot
        target_ori = target_rot.as_quat()
        
        return target_pos, target_ori
    
    def _get_hold_target(self, robot_current_pos, robot_current_ori):
        """返回保持位置的目标"""
        if self.last_robot_target_pos is not None:
            return self.last_robot_target_pos, self.last_robot_target_ori, self.state
        return robot_current_pos, robot_current_ori, self.state
    
    def reset(self):
        """重置控制器状态"""
        self.state = ClutchState.IDLE
        self.vr_reference_pos = None
        self.vr_reference_ori = None
        self.robot_reference_pos = None
        self.robot_reference_ori = None
        self.last_robot_target_pos = None
        self.last_robot_target_ori = None
```

---

### 2. 重写 `vr_interface_node.py`

关键改动:
- 使用 `VRClutchController` 替代原有的映射方法
- 订阅 `/vr/*/joy` 获取grip值
- 发布clutch状态

---

### 3. teleoperation_node.cpp 修改

添加:
- 订阅 `/vr/clutch_status` 话题
- 当clutch未接合时，不发送关节指令（或发送0速度保持指令）

---

## ❓ 需要确认的问题

### Q1: 机器人当前末端位姿从哪里获取?

**选项A**: 从 `/joint_states` 通过FK计算
- 优点: 最准确
- 缺点: 需要在vr_interface中做FK计算

**选项B**: 订阅 teleoperation_controller 发布的末端位姿
- 优点: 简单
- 缺点: 需要teleoperation_controller发布

**选项C**: 使用目标位姿作为"当前位姿"的近似
- 优点: 最简单
- 缺点: 在快速运动时有偏差

**推荐**: 选项B，让teleoperation_controller发布当前末端位姿

### Q2: 坐标系变换如何处理?

VR坐标系 ≠ 机器人坐标系

**方案**: 在Clutch模式下，VR增量需要变换到机器人坐标系
- 可以复用现有的 `qyh_vr_calibration` 的校准数据
- 或者使用固定的变换矩阵

### Q3: 双臂是否独立控制?

**推荐**: 独立控制
- 左右手各自独立的Clutch状态
- 左手握紧控制左臂，右手握紧控制右臂
- 可以单独操作一只臂

---

## 📅 时间估算

| 阶段 | 任务 | 估计时间 |
|------|------|----------|
| Phase 1 | Clutch核心逻辑 | 2-3小时 |
| Phase 2 | 消息和状态 | 0.5小时 |
| Phase 3 | teleoperation适配 | 1小时 |
| Phase 4 | 前端网页 | 2小时 |
| Phase 5 | 真机测试调优 | 2-4小时 |
| **合计** | | **8-11小时** |

---

## ✅ 确认清单

在开始修改前，请确认:

- [ ] 对Clutch模式的理解是否正确
- [ ] Q1: 机器人末端位姿获取方式
- [ ] Q2: 坐标系变换方案
- [ ] Q3: 双臂控制方式
- [ ] 是否需要保留原有的calibration功能
- [ ] 前端网页的具体需求

---

**文档版本**: v1.0
**创建日期**: 2025-01-21
