# VR 双臂遥操作 TF 设计文档

## 📋 概述

本文档定义了从 PICO4 VR 手柄到双臂机器人末端执行器的完整坐标变换链路。

**核心思想：** VR → 人体语义坐标系 → 机器人 base_link → 左右臂各自的 base → 末端执行器

---

## 1️⃣ 完整 TF 树结构

```
map (可选，用于导航)
│
└── base_link                          ← 双臂中心，机器人本体根节点
    ├── base_link_left                 ← 左臂安装点（带校准偏移）
    │   └── l1 → l2 → ... → l7 → lt   ← 左臂末端执行器
    │
    ├── base_link_right                ← 右臂安装点（带校准偏移）
    │   └── r1 → r2 → ... → r7 → rt   ← 右臂末端执行器
    │
    ├── base_vr                        ← VR参考坐标系（已存在）
    │
    └── teleop_base                    ← ★人体语义坐标系（核心中间层）
        └── vr_origin                  ← VR零位锁定点
            ├── vr_left_controller     ← PICO4左手柄原始位姿
            ├── vr_right_controller    ← PICO4右手柄原始位姿
            ├── human_left_hand        ← 左手语义坐标（轴对齐后）
            └── human_right_hand       ← 右手语义坐标（轴对齐后）
```

---

## 2️⃣ 各坐标系详细定义

### 📍 `base_link` - 机器人本体中心
- **位置：** 双臂机械结构的中心点
- **轴向：**
  - X：向前（机器人正前方）
  - Y：向左
  - Z：向上
- **来源：** URDF定义（已有）
- **特点：** 带惯性参数（有警告但不影响使用）

### 📍 `base_link_left` / `base_link_right` - 左右臂安装点
- **位置：** 
  - 左臂：`xyz=[-0.0004, 0.08522, 0.0030]` `rpy=[0, 0, -30°]`
  - 右臂：`xyz=[-0.001080, -0.083950, 0.000951]` `rpy=[0, 0, 30°]`
- **特点：** 
  - ⭐ 包含实测校准偏移
  - 补偿URDF计算与真实机器人的差异
  - 是真实机械臂的准确参考点

### 📍 `lt` / `rt` - 末端执行器
- **作用：** 机械臂工具坐标系（TCP）
- **位置：** 相对于各自的 `base_link_left/right`
- **轴向：** ⚠️ **重要差异**
  - **X：向左**（而不是向前！）
  - **Y：向上**
  - **Z：向后**
- **重要：** ⭐⭐⭐ **这是最终发送给机械臂控制器的目标位姿**
- **影响：** 需要额外的坐标旋转来匹配人手语义

### 📍 `teleop_base` - 人体语义坐标系 ⭐核心
- **位置：** 相对于 `base_link`，可配置偏移
- **轴向：**
  - X：人体正前方（胸口朝向）
  - Y：人体左侧
  - Z：向上
- **作用：** 
  - 隔离VR设备特性
  - 提供稳定的操作参考
  - 支持操作缩放、滤波

### 📍 `vr_origin` - VR零位锁定点
- **作用：** 记录"开始遥控"时的VR位姿
- **更新时机：** 用户按下"Start Teleop"按钮
- **特点：** 
  - 消除VR设备漂移
  - 支持重新站位、转头
  - 相对变换始终有效

### 📍 `vr_left/right_controller` - VR手柄原始坐标
- **来源：** PICO4 SDK直接输出
- **轴向（右手系）：**
  - X：向右
  - Y：向上
  - Z：向后（朝向人）
- **特点：** 不修改，保持SDK原始数据

### 📍 `human_left/right_hand` - 人手语义坐标 ⭐关键
- **轴向：**
  - X：手指朝向（前）
  - Y：手掌左侧
  - Z：手掌向上
- **作用：** 
  - 完成VR坐标到人体语义的轴对齐
  - 应用位置缩放
  - 数据滤波处理
- **注意：** 与机械臂末端坐标系（lt/rt）的方向**不一致**，需要在IK层处理

---

## 3️⃣ 坐标变换数学关系

### 🔄 变换链路

```
目标：求 T(base_link_left → lt) 和 T(base_link_right → rt)

完整链路：
T(base_link_left → lt_target) = 
    T(base_link_left → base_link)^-1
    · T(base_link → teleop_base)
    · T(teleop_base → vr_origin)
    · T(vr_origin → vr_left_controller)
    · T(vr_left_controller → human_left_hand)
    · T(human_left_hand → lt)  ← ⚠️ 末端坐标系校正（X轴旋转）
```

### 📐 关键变换矩阵

#### A. PICO SDK → ROS标准坐标系（在vr_bridge中完成）⭐

**已在节点1实现，无需在后续节点重复**

PICO SDK坐标系：`[X右, Y上, -Z前]` → ROS标准坐标系：`[X前, Y左, Z上]`

```python
# vr_bridge_node.cpp 中的实现
def map_position(vr_x, vr_y, vr_z):
    ros_x = -vr_z   # PICO的-Z(前) → ROS的X(前)
    ros_y = -vr_x   # PICO的-X(左) → ROS的Y(左)
    ros_z = vr_y    # PICO的Y(上)  → ROS的Z(上)
    return ros_x, ros_y, ros_z

# 对应旋转矩阵
R_ros_pico = [
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0]
]
```

**结果**：`vr_left_controller` 和 `vr_right_controller` 输出已经是ROS标准坐标

#### B. base_link → teleop_base（可配置）

```yaml
# 推荐配置
teleop_offset:
  x: 0.5   # 人到机器人的虚拟距离（米）
  y: 0.0
  z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0  # 可调整人的面向角度
```

#### C. 零位校准（运行时动态）

```python
# 当按下"Start Teleop"
T_vr_origin_at_start = getCurrentVRPose()

# 后续每帧
T_vr_origin_to_controller = 
   

#### D. 末端坐标系校正（human_hand → lt/rt）⚠️ 重要

**问题：** 机械臂末端坐标系方向与人手语义不一致
- `human_hand`：X向前（手指方向）
- `lt/rt`：X向左

**解决方案：** 添加固定旋转

```python
# 左臂：human_left_hand → lt
# X_lt(左) = Y_human(左)
# Y_lt(上) = Z_human(上)  
# Z_lt(后) = -X_human(前)

R_lt_human_left = [
    [ 0,  1,  0],
    [ 0,  0,  1],
    [-1,  0,  0]
]

# 右臂：human_right_hand → rt
# 坐标系一致性处理
R_rt_human_right = [
    [ 0, -1,  0],
    [ 0,  0,  1],
    [ 1,  0,  0]
]
```

**说明：**
- 这个旋转是**固定的、确定的**
- 在IK求解前应用
- 确保人手"向前推"对应机械臂末端"向前移动" T_vr_origin_at_start.inverse() * T_current_controller
```

---

## 4️⃣ ROS2 节点职责划分

### 🟦 节点1: `vr_bridge_node` (已实现)
**功能：** VR设备原始数据接收 + 底层坐标对齐

**职责：**
- 监听UDP端口接收PICO4数据包
- **⭐ 底层坐标对齐：** PICO SDK坐标系 → ROS标准坐标系 (REP-103)
  - **输入** PICO: `X-右, Y-上, -Z-前` (OpenXR标准)
  - **输出** ROS: `X-前, Y-左, Z-上` (REP-103标准)
  - **实现**: `ros_x = -vr_z`, `ros_y = -vr_x`, `ros_z = vr_y`
- 发布TF：`vr_origin → vr_head`
- 发布TF：`vr_origin → vr_left_controller` (已对齐到ROS坐标系)
- 发布TF：`vr_origin → vr_right_controller` (已对齐到ROS坐标系)
- 发布话题：`/vr/*/pose`, `/vr/*/joy`, `/vr/*/active`

**⚠️ 重要说明：**
- ✅ **已完成VR → ROS的轴对齐**，后续节点接收到的 `vr_*_controller` 已经是ROS标准坐标
- ✅ `vr_*_controller` 输出：X-前（手指向前）, Y-左（手掌左侧）, Z-上（手掌向上）
- ❌ 不包含：握持补偿、位置缩放、滤波、零位校准（由后续节点处理）

**输出频率：** 取决于PICO4发送频率（通常60-100Hz）

**状态：** ✅ 已实现并简化

---

### 🟦 节点2: `teleop_manager_node`
**功能：** 遥操作状态管理

**职责：**
- 监听"Start/Stop Teleop"按钮
- 管理 `vr_origin` 的位姿更新
- 发布TF：`teleop_base → vr_origin` (动态)
- 发布服务：`/teleop/start`、`/teleop/stop`、`/teleop/recenter`

**关键功能：**
```python
on_start_button():
    记录当前VR位姿作为零点
    激活遥操作模式
    
on_recenter_button():
    重新设置vr_origin
```

---

### 🟦 节点3: `coordinate_mapper_node` ⭐核心
**功能：** 握持补偿与数据处理

**职责：**
- 订阅：`vr_*_controller` TF (已经是ROS标准坐标系)
- 应用握持补偿旋转（35度pitch）
- 位置缩放（VR空间 → 机器人工作空间）
- 低通滤波（平滑抖动）
- 速度/加速度限制
- 发布TF：`vr_*_controller → human_*_hand`
- **发布目标位姿话题：frame_id = "vr_origin"**

**重要说明：**
- ✅ 发布的位姿在`vr_origin`坐标系下（人体语义空间）
- ✅ 不负责转换到机器人坐标系（由IK求解器完成）
- ✅ 保持独立性，不依赖机器人模型

**代码示例：**
```python
def solve_left_arm_ik(human_left_hand_pose):
    # Step 1: 查询TF
    T_base_to_human = tf_buffer.lookup_transform(
        'base_link_left', 'human_left_hand', time)
    
    # Step 2: 应用末端坐标系校正 ⚠️ 关键步骤
    R_correction = np.array([
        [ 0,  1,  0],
        [ 0,  0,  1],
        [-1,  0,  0]
    ])
    T_corrected = apply_rotation_correction(T_base_to_human, R_correction)
    
    # Step 3: 调用IK求解器
    joint_angles = jaka_ik_solver(T_corrected, current_joints)
    
    return joint_angles滤波**（消抖）
  4. **速度/加速度限制**
- 发布TF：`vr_*_controller → human_*_hand`
- 发布话题：`/teleop/left_hand/target`、`/teleop/right_hand/target`

**可配置参数：**
```yaml
coordinate_mapper:
  position_scale: 1.0      # 位置缩放因子
  rotation_scale: 1.0      # 旋转缩放因子
  filter_alpha: 0.3        # 低通滤波系数
  max_linear_vel: 0.5      # m/s
  max_angular_vel: 1.0     # rad/s
```

---

### 🟦 节点4: `dual_arm_ik_solver_node`
**功能：** 逆运动学求解

**职责：**
- 订阅：`/teleop/left_hand/target`、`/teleop/right_hand/target` (frame_id="vr_origin")
- **⭐ 坐标系转换**：通过TF查询将目标位姿转换到机器人坐标系
  - `T(base_link_left → human_left_hand)` = TF查询完整变换链
  - `T(base_link_right → human_right_hand)` = TF查询完整变换链
- **⭐ 末端坐标系校正**：应用`human_hand → lt/rt`旋转（X前→X左）
- 调用IK求解器（JAKA SDK）
- 关节限位和速度检查
- 发布关节指令

**IK求解流程：**
```python
def solve_left_arm_ik(target_pose_stamped):  # frame_id="vr_origin"
    # Step 1: ⭐ 坐标系转换 - 通过TF查询完整变换链
    # 自动包含: vr_origin → teleop_base → base_link → base_link_left
    target_in_base_left = tf_buffer.transform(
        target_pose_stamped, 
        "base_link_left",  # 目标坐标系
        timeout=0.1
    )
    
    # Step 2: ⭐ 末端坐标系校正 (human_hand坐标系 → lt坐标系)
    # human_hand: [X前, Y左, Z上] → lt: [X左, Y上, Z后]
    R_correction = [[ 0,  1,  0],
                    [ 0,  0,  1],
                    [-1,  0,  0]]
    pose_corrected = apply_rotation(target_in_base_left, R_correction)
    
    # Step 3: 调用JAKA IK求解
    joint_angles = robot.kine_inverse(
        robot_id=0,  # 左臂
        ref_joints=current_joints,
        target_pose=pose_corrected  # 已经在base_link_left坐标系
    )
    
    # Step 4: 安全检查
    if check_joint_limits(joint_angles):
        return joint_angles
```

**重要说明：**
- ✅ 接收`vr_origin`坐标系的目标位姿
- ✅ 使用TF查询自动处理完整变换链（包括teleop_base, base_link等）
- ✅ 应用末端坐标系校正（人手语义→机械臂末端）
- ✅ JAKA IK输入是相对于`base_link_left/right`的位姿

---

### 🟦 节点5: `arm_controller_node` + `robot_state_publisher`
**功能：** 机械臂底层控制 + TF树发布

**职责：**
- 订阅：`/left_arm/joint_trajectory`、`/right_arm/joint_trajectory`
- 通过JAKA SDK发送关节指令
- 发布 `/joint_states` (14个关节)
- 监控机械臂状态
- 安全检查（碰撞、奇异点）

**⭐ 必须同时启动 `robot_state_publisher`：**
```bash
# 在launch文件中添加
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description_content}]
)
```

**TF发布说明：**
- **URDF定义**：`base_link → base_link_left/right`（包含校准偏移）
- **robot_state_publisher发布**：
  - 静态TF：`base_link → base_link_left/right`
  - 动态TF：`base_link_left → l1 → l2 → ... → lt` (根据/joint_states)
  - 动态TF：`base_link_right → r1 → r2 → ... → rt`

**特别注意：**
- 最终发送的位姿是 **`lt` 相对于 `base_link_left`** 和 **`rt` 相对于 `base_link_right`**
- 校准偏移已经包含在 `base_link_left/right` 的定义中
- **如果robot_state_publisher未启动**：TF树不完整，IK求解器将失败

---

## 5️⃣ 数据流向图

```
PICO4 VR设备
    ↓ UDP
[pico4_bridge_node]
    ↓ TF: vr_origin → vr_*_controller
    ↓ Topic: /vr/*/pose
    
用户按钮 → [teleop_manager_node]
    ↓ TF: teleop_base → vr_origin
    ↓ Service: /teleop/start
    
[coordinate_mapper_node] ⭐
    ↓ TF: vr_*_controller → human_*_hand
    ↓ Topic: /teleop/*/target
    ↓ (应用：轴对齐 + 缩放 + 滤波)
    
[dual_arm_ik_solver_node]
    ↓ 查询TF: base_link_* → human_*_hand
    ↓ IK求解
    ↓ Topic: /*/joint_trajectory
    
[arm_controller_node]
    ↓ JAKA SDK
    ↓ 发送位姿: T(base_link_left → lt), T(base_link_right → rt)
机械臂运动
```

---

## 6️⃣ 静态TF配置（launch文件）

```python
# 在launch文件中配置静态TF
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 配置 teleop_base 相对于 base_link 的位置
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='teleop_base_publisher',
        arguments=[
            '0.5', '0', '0',      # xyz偏移（人在机器人前方0.5米）
            '0', '0', '0', '1',   # 四元数(无旋转)
            'base_link',
            'teleop_base'
        ]
    )
    
    return LaunchDescription([
        static_tf_publisher,
        # ... 其他节点
    ])
```

---

## 7️⃣ 关键实现细节

### ✅ 零位校准的正确实现

```python
class TeleopManager:
    def __init__(self):
        self.vr_origin_set = False
        self.T_start_left = None
        self.T_start_right = None
    
    def on_start_teleop(self):
        # 记录当前VR手柄位姿
        self.T_start_left = self.get_current_vr_pose('left')
        self.T_start_right = self.get_current_vr_pose('right')
        self.vr_origin_set = True
    
    def publish_vr_origin(self):
        if not self.vr_origin_set:
            # 未校准时，vr_origin = teleop_base
            self.publish_identity_tf('teleop_base', 'vr_origin')
        else:
            # 已校准，发布校准变换
            # vr_origin固定，controller相对移动
            T_current_left = self.get_current_vr_pose('left')
            T_relative_left = self.T_start_left.inverse() * T_current_left
            self.publish_tf('vr_origin', 'vr_left_controller', T_relative_left)
```

### ✅ 坐标轴对齐实现

```python
import numpy as np
from scipy.spatial.transform import Rotation

def vr_to_human_transform(vr_pose):
    """VR坐标系 → 人手语义坐标系"""
    # VR: [X右, Y上, Z后] → Human: [X前, Y左, Z上]
    R_align = np.array([
        [ 0,  0, -1],
        [-1,  0,  0],
        [ 0,  1,  0]
    ])
    
    # 提取VR位姿的位置和旋转
    p_vr = vr_pose.position  # [x, y, z]
    R_vr = Rotation.from_quat(vr_pose.orientation).as_matrix()
    
    # 位置变换
    p_human = R_align @ p_vr
    
    # 旋转变换
    R_human = R_align @ R_vr @ R_align.T
    
    # 构造新位姿
    q_human = Rotation.from_matrix(R_human).as_quat()
    return Pose(p_human, q_human)
```

### ✅ 低通滤波实现

```python
class PoseFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha  # 滤波系数，越小越平滑
        self.prev_pose = None
    
    def filter(self, new_pose):
        if self.prev_pose is None:
            self.prev_pose = new_pose
            return new_pose
        
        # 位置滤波（指数移动平均）
        filtered_pos = (self.alpha * new_pose.position + 
                       (1 - self.alpha) * self.prev_pose.position)
        
        # 旋转滤波（球面插值）
        from scipy.spatial.transform import Slerp
        r0 = Rotation.from_quat(self.prev_pose.orientation)
        r1 = Rotation.from_quat(new_pose.orientation)
        slerp = Slerp([0, 1], Rotation.concatenate([r0, r1]))
        filtered_rot = slerp(self.alpha).as_quat()
        
        filtered = Pose(filtered_pos, filtered_rot)
        self.prev_pose = filtered
        return filtered
```

### ✅ 速度限制实现

```python
class VelocityLimiter:
    def __init__(self, max_linear_vel=0.5, max_angular_vel=1.0, dt=0.01):
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.dt = dt
        self.prev_pose = None
    
    def limit(self, target_pose):
        if self.prev_pose is None:
            self.prev_pose = target_pose
            return target_pose
        
        # 计算线速度
        dp = target_pose.position - self.prev_pose.position
        linear_vel = np.linalg.norm(dp) / self.dt
        
        if linear_vel > self.max_linear_vel:
            # 限制线速度
            scale = self.max_linear_vel / linear_vel
            target_pose.position = (self.prev_pose.position + 
                                   dp * scale)
        
        # 计算角速度并限制（类似处理）
        # ...
        
        self.prev_pose = target_pose
        return target_pose
```

---

## 8️⃣ 配置参数参考

```yaml
# config/teleop_params.yaml
teleop:
  # 人体坐标系相对base_link的偏移
  teleop_base_offset:
    x: 0.5
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  
  # 坐标映射参数
  coordinate_mapping:
    position_scale: 1.0       # 位置缩放因子
    rotation_scale: 1.0       # 旋转缩放因子
    filter_alpha: 0.3         # 低通滤波系数 (0-1)
    
  # 安全限制
  safety:
    max_linear_velocity: 0.5    # m/s
    max_angular_velocity: 1.0   # rad/s
    max_linear_accel: 2.0       # m/s²
    
    # 工作空间限制（相对于base_link）
    workspace_limits:
      x_min: -0.5
      x_max: 0.8
      y_min: -0.6
      y_max: 0.6
      z_min: 0.0
      z_max: 1.0
    
  # IK求解器参数
  ik_solver:
    solver_type: "jaka_sdk"  # or "kdl"
    max_iterations: 100
    tolerance: 0.001
```
├─ [5] T(vr_left_controller → human_left_hand) [轴对齐+滤波]
    │
    └─ [6] T(human_left_hand → lt)                [⚠️ 末端坐标系校正]

然后通过IK求解器：
    # 注意：IK输入是带校正后的末端位姿
## 9️⃣ 调试工具

### 查看TF树
```bash
# 生成TF树可视化图
ros2 run tf2_tools view_frames

# 实时查看特定变换
ros2 run tf2_ros tf2_echo base_link human_left_hand

# 查看所有TF
ros2 run tf2_ros tf2_monitor
```

### RViz配置
添加以下显示：
- **TF Display** - 显示所有坐标系
  - 勾选关键frames：
    - base_link, base_link_left, base_link_right
    - teleop_base, vr_origin
    - human_left_hand, human_right_hand
    - lt, rt (末端执行器)
- **RobotModel** - 显示机械臂
- **Axes** - 显示各坐标系的轴向

### 发布测试数据
```bash
# 手动发布VR位姿进行测试
ros2 topic pub /vr/left_controller/pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'vr_origin'}, 
    pose: {position: {x: 0.3, y: 0, z: 0.5}, 
           orientation: {x: 0, y: 0, z: 0, w: 1}}}"
```

---

## 🎯 总结：完整变换链

### 最终目标位姿计算

```
目标：计算 T(base_link_left → lt_target) 和 T(base_link_right → rt_target)

完整变换链：
T(base_link_left → lt_target) = 
    ┌─ [1] T(base_link_left → base_link)^-1       [URDF固定，含校准偏移]
    │
    ├─ [2] T(base_link → teleop_base)             [静态配置，可调整]
    │
   ⚠️ **末端校正：** `lt/rt` 的X轴向左，需要额外旋转匹配人手语义
6.  ├─ [3] T(teleop_base → vr_origin)             [零位校准，动态]
    │
    ├─ [4] T(vr_origin → vr_left_controller)      [VR SDK，100Hz]
    │
    └─ [5] T(vr_left_controller → human_left_hand) [轴对齐+滤波]

然后通过IK求解器：
    joint_angles = IK_solver(T_target, current_joints)

最后发送给机械臂（关键！）：
    # 注意：发送的是相对于各自base_link的位姿
    left_arm_controller.move_to_pose(
        T(base_link_left → lt_target)
    )
    right_arm_controller.move_to_pose(
        T(base_link_right → rt_target)
    )
```

### 关键点总结

1. ✅ **中间层隔离：** `teleop_base` 是VR和机器人之间的关键桥梁
2. ✅ **零位校准：** `vr_origin` 消除VR设备漂移和姿态变化
3. ✅ **轴对齐：** `human_*_hand` 完成VR到人体语义的坐标转换
4. ✅ **校准偏移：** `base_link_left/right` 已包含实测校准数据
5. ✅ **最终输出：** IK结果是相对于各臂base的位姿，直接发给机械臂控制器

---

## 📚 参考资料

- [Mobile ALOHA VR Teleoperation](https://mobile-aloha.github.io/)
- [ROS2 TF2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [JAKA Zu7 机械臂手册](https://www.jaka.com/)
- [PICO4 SDK文档](https://developer.picoxr.com/)

---

## 🔧 下一步开发计划

1. [ ] 实现 `pico4_bridge_node` - VR数据接收
2. [ ] 实现 `teleop_manager_node` - 状态管理和零位校准
3. [ ] 实现 `coordinate_mapper_node` - 坐标对齐和滤波（核心）
4. [ ] 实现 `dual_arm_ik_solver_node` - IK求解
5. [ ] 集成 `arm_controller_node` - 与JAKA SDK对接
6. [ ] 测试和调优 - 真机验证

