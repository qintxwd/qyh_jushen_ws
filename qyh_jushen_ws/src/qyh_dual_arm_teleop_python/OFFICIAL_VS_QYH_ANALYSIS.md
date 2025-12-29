# 官方示例 vs QYH实现 对比分析

## 一、核心差异总结

| 方面 | 官方示例 | QYH实现 |
|------|----------|---------|
| VR数据接收 | TCP Socket + 字符串解析 | UDP Socket + 二进制结构体 |
| VR数据格式 | `"x,y,z,rx,ry,rz"` 字符串 | `PoseStamped` + `Joy` ROS消息 |
| 坐标系 | 保留原始VR坐标，在映射时处理 | VR Bridge已做PICO→ROS转换 |
| 30°处理 | Python矩阵计算 `rotation_matrix_z(-30)` | URDF TF变换 + 矩阵 `T_base_base_left` |
| 遥操控制 | 服务调用 `/vr_robot_pose_converter` | Grip按钮状态机 (IDLE→ENGAGING→TRACKING) |
| 发送格式 | xyz(mm) + rpy(弧度) → servo_p | xyz(米→mm) + rpy(弧度→度) |

---

## 二、VR数据格式差异

### 2.1 官方VR数据流
```
PICO App (TCP字符串) 
    → vr_data_pub.py (接收) 
    → /vr_raw_data (String) 
    → vr_data_distributer.py 
    → /left_arm/vr_pose (String: "x,y,z,rx,ry,rz")
```

官方VR数据格式（从vr_data_distributer.py看）：
- 位置: 米
- 姿态: 度 (degree)
- 欧拉角顺序: YXZ（从`euler_to_rotation_matrix`看：`R = Ry @ Rx @ Rz`）
- 转换时取负: `-rx, -ry, rz`

### 2.2 QYH VR数据流
```
PICO App (UDP二进制)
    → vr_bridge_node.cpp (接收 + PICO→ROS坐标转换)
    → /vr/left_controller/pose (PoseStamped)
    → qyh_teleop.py
```

QYH VR Bridge已做的坐标转换（PICO SDK → ROS）:
```cpp
// 位置映射
ros_x = -vr_z;   // PICO的-Z(前) → ROS的X(前)
ros_y = -vr_x;   // PICO的-X(左) → ROS的Y(左)
ros_z = vr_y;    // PICO的Y(上)  → ROS的Z(上)

// 四元数映射
ros_qx = -vr_qz, ros_qy = -vr_qx, ros_qz = vr_qy, ros_qw = vr_qw
```

### 2.3 关键问题
**QYH的VR Bridge已经将VR数据转换到ROS标准坐标系（X前Y左Z上），而官方保留原始VR坐标。**

这意味着你的`qyh_teleop.py`收到的数据已经是ROS坐标系下的了，但你代码中的变换逻辑可能假设收到的是原始VR坐标。

---

## 三、30°偏移处理差异

### 3.1 官方处理方式
在`vr_robot_pose_converter_30degree.py`中：
```python
# 对于左臂，应用-30度旋转
z_rot = self.rotation_matrix_z(-30)  # 绕Z轴旋转-30°
vr_rot = np.dot(z_rot, vr_rot)       # 修改VR的坐标映射基础
left_diff_base = np.dot(vr_rot, np.dot(left_rotvr_diff, np.linalg.inv(vr_rot)))
```

### 3.2 QYH处理方式
在URDF中定义固定变换：
```xml
<joint name="base_to_left" type="fixed">
    <origin xyz="-0.0004 0.08522 0.0030" rpy="0 0 -0.5236" />
    <!-- -0.5236 rad = -30° -->
</joint>
```

在`qyh_teleop.py`中使用矩阵：
```python
self.T_base_base_left = self.make_transform_matrix(
    translation=[-0.0004, 0.08522, 0.0030],
    rpy=[0, 0, -math.pi / 6]  # -30°
)
```

### 3.3 建议
两种方式都可以，但需要保持一致：
- 如果用URDF TF变换，确保Python代码正确读取并应用
- 目前你的`T_base_base_left`变换是正确的

---

## 四、坐标映射差异

### 4.1 官方VR到机械臂映射
```python
# VR坐标系映射基础矩阵
vr_rot = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]
])

# 位置增量映射
next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]   # VR_z → arm_x
next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]   # -VR_x → arm_y
next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]   # VR_y → arm_z
```

官方的映射关系：
- VR Z → 机械臂 X（前）
- -VR X → 机械臂 Y（左）
- VR Y → 机械臂 Z（上）

### 4.2 QYH映射
```python
# VR控制器坐标系(x前y左z上) → 机械臂末端坐标系(x左y上z前)
self.R_vr_to_arm = np.array([
    [0, 1, 0],  # lt_x = VR_y (左)
    [0, 0, 1],  # lt_y = VR_z (上)
    [1, 0, 0]   # lt_z = VR_x (前)
])
```

QYH映射（假设VR已转换到ROS坐标系后）：
- VR X(前) → lt Z(前)
- VR Y(左) → lt X(左)
- VR Z(上) → lt Y(上)

### 4.3 问题分析
由于VR Bridge已经做了PICO→ROS坐标转换，你收到的VR数据已经是：
- VR X = 前（ROS X）
- VR Y = 左（ROS Y）
- VR Z = 上（ROS Z）

而机械臂末端`lt`的坐标系定义：
- lt X = 左
- lt Y = 上
- lt Z = 前

所以你的`R_vr_to_arm`映射是正确的。

---

## 五、遥操控制逻辑差异

### 5.1 官方控制流程
```
1. 调用服务 /left_arm/servo_move_enable (启用伺服)
2. 调用服务 /left_arm/vr_robot_pose_converter (启用VR遥操)
3. 持续发送servo_p命令
4. 调用服务关闭遥操 → 返回默认位姿
```

官方是通过服务调用来控制遥操的开始和结束。

### 5.2 QYH控制流程
```
1. 调用服务启用伺服模式
2. 按下grip按钮 → ENGAGING → 冻结VR和机械臂起始位姿
3. TRACKING → 计算增量并发送servo_p
4. 松开grip按钮 → IDLE → 停止发送（保持当前位姿）
5. 再次按下grip → 重新对齐
```

QYH的clutch模式更适合需要频繁调整的场景。

### 5.3 建议
你的clutch模式设计是合理的，但需要注意：
- 确保IDLE状态时机械臂不会因为没有命令而自动停止伺服
- 考虑添加超时保护

---

## 六、发送格式差异

### 6.1 官方servo_p格式
```python
# vr_robot_pose_converter_30degree_servo_p.py
next_pose = [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
self.publish_servo_p_command(next_pose)
```

从驱动代码看：
```cpp
// dual_arm_driver_template.cpp
pose_left.tran.x = joint_positions[0];  // mm
pose_left.tran.y = joint_positions[1];  // mm
pose_left.tran.z = joint_positions[2];  // mm
pose_left.rpy.rx = joint_positions[3];  // rad
pose_left.rpy.ry = joint_positions[4];  // rad
pose_left.rpy.rz = joint_positions[5];  // rad
```

**官方格式: xyz(mm) + rpy(弧度)**

### 6.2 QYH发送格式
```python
# qyh_teleop.py
out_pose = [
    target_position[0],      # x (m)
    target_position[1],      # y (m)
    target_position[2],      # z (m)
    np.degrees(target_rpy[0]),  # roll (degree)
    np.degrees(target_rpy[1]),  # pitch (degree)
    np.degrees(target_rpy[2])   # yaw (degree)
]

# 发送时转换
msg.position = [
    pose[0] * 1000.0,  # x: m → mm
    pose[1] * 1000.0,  # y: m → mm
    pose[2] * 1000.0,  # z: m → mm
    pose[3],           # roll (degree)
    pose[4],           # pitch (degree)
    pose[5]            # yaw (degree)
]
```

**QYH格式: xyz(mm) + rpy(度)**

### 6.3 ⚠️ 关键问题
**官方驱动期望的是弧度，但你发送的是度！**

你需要修改`publish_servo_p_command`：
```python
msg.position = [
    pose[0] * 1000.0,  # x: m → mm
    pose[1] * 1000.0,  # y: m → mm
    pose[2] * 1000.0,  # z: m → mm
    math.radians(pose[3]),  # roll: deg → rad
    math.radians(pose[4]),  # pitch: deg → rad
    math.radians(pose[5])   # yaw: deg → rad
]
```

或者检查你的`jaka_control_node`是否期望度。

---

## 七、已修复问题列表 ✅

### 7.1 已修复 - 高优先级
1. **✅ servo_p的RPY单位问题**：
   - 确认了`jaka_control_node.cpp`期望xyz(mm) + rpy(弧度)
   - 修改`publish_servo_p_command`直接发送弧度，不再转换为度

2. **✅ 修复日志中的变量名错误**：
   - 将未定义的`delta_rpy`修改为`delta_rpy_raw`和`delta_rpy_aligned`

3. **✅ 消息长度问题**：
   - `jaka_control_node`检查`msg->position.size() != 7`
   - 修改`publish_servo_p_command`发送7个值（第7个为占位值）

4. **✅ 取消注释的publish调用**：
   - 之前`self.left_servo_pub.publish(msg)`被注释掉了，现在已启用

### 7.2 待处理 - 中优先级
1. **Pico手柄姿态补偿**：`T_vr_human_align`的pitch补偿(-35°)可能需要根据实际测试调整

2. **位置增量映射**：当前直接使用base_link_left下的增量，可能需要额外的坐标变换

### 7.3 待处理 - 低优先级
1. 添加弯腰TF变换支持
2. 添加右臂遥操支持

---

## 八、修复代码说明

### 8.1 publish_servo_p_command 修复后的代码
```python
def publish_servo_p_command(self, pose):
    """
    发送servo_p命令到机械臂
    
    Args:
        pose: [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]
              位置单位: 米
              姿态单位: 弧度
    
    发送格式:
        jaka_control_node期望7个值: [x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad, unused]
        JAKA SDK的edg_servo_p期望: xyz(mm) + rpy(弧度)
    """
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()

    # 发送格式：xyz(mm) + rpy(弧度) + 1个占位值（共7个值）
    msg.position = [
        pose[0] * 1000.0,  # x: m -> mm
        pose[1] * 1000.0,  # y: m -> mm
        pose[2] * 1000.0,  # z: m -> mm
        pose[3],           # roll (弧度)
        pose[4],           # pitch (弧度)
        pose[5],           # yaw (弧度)
        0.0                # 占位值
    ]

    self.left_servo_pub.publish(msg)
```

### 8.2 目标位姿计算修复
```python
# 5️⃣ 发送目标位姿：xyz(m) + rpy(弧度)
out_pose = [
    target_position[0],      # x (m) - 将在publish函数中转换为mm
    target_position[1],      # y (m)
    target_position[2],      # z (m)
    target_rpy[0],           # roll (弧度) - 直接使用，不转换为度
    target_rpy[1],           # pitch (弧度)
    target_rpy[2]            # yaw (弧度)
]
```

---

## 八点五、四元数→RPY多解问题修复 ⚠️ 重要

### 问题描述
原代码存在两个问题：

1. **直接在RPY空间进行增量计算**（数学上不正确）
2. **从四元数获取机械臂起始RPY**（可能产生跳变）

### 解决方案：参考官方实现
参考 `vr_robot_pose_converter_30degree_servo_p.py`：

```python
# ✅ 官方做法（已实现）

# ENGAGING阶段：
# - VR起点使用旋转矩阵（用于计算增量）
self.vr_start_rotm = T_vr_in_left[:3, :3].copy()
# - 机械臂起点直接使用原始RPY（不从四元数转换！）
self.arm_start_rpy = np.array(self.cur_tcp_rpy).copy()

# TRACKING阶段：
# 1. 计算VR旋转增量（旋转矩阵）
delta_rotm_vr = vr_current_rotm @ np.linalg.inv(self.vr_start_rotm)

# 2. 从原始RPY构建机械臂起始旋转矩阵
arm_start_rotm = R.from_euler('xyz', self.arm_start_rpy).as_matrix()

# 3. 应用增量
target_rotm = delta_rotm_vr @ arm_start_rotm

# 4. 只在最后转换为RPY发送
target_rpy = R.from_matrix(target_rotm).as_euler('xyz')
```

### 关键改进

| 方面 | 原方案 | 新方案 |
|------|--------|--------|
| 机械臂起始姿态 | `cur_tcp_rotm`（从四元数） | `cur_tcp_rpy`（原始RPY） |
| VR增量计算 | RPY空间加减法 | 旋转矩阵乘法 |
| 最终转换 | 多次转换 | 只在发送时转换一次 |

### 优势
1. **使用机械臂的原始RPY**：`cur_tcp_rpy`是控制器直接报告的值，与servo_p期望一致
2. **旋转矩阵计算增量**：数学上正确，避免欧拉角加法问题
3. **最小化转换次数**：只在最后一步从旋转矩阵转换为RPY

---

## 九、测试验证步骤

1. **验证VR数据接收**：
   ```bash
   ros2 topic echo /vr/left_controller/pose
   ```
   检查位置和姿态是否符合预期（ROS坐标系）

2. **验证坐标变换**：
   在ENGAGING状态时，记录VR和机械臂的起始位姿，确认对齐正确

3. **验证增量计算**：
   小幅度移动VR手柄，检查delta_rpy和delta_pos是否正确

4. **验证servo_p发送**：
   ```bash
   ros2 topic echo /teleop/left/servo_p
   ```
   检查发送的数据单位是否正确

---

## 十、总结

主要需要关注的问题：
1. **RPY单位统一**：确保发送给机械臂的RPY单位与驱动期望一致
2. **坐标系变换链**：VR(原始) → VR(ROS) → base_link_left → 机械臂末端
3. **变量名修复**：代码中有未定义变量的使用

你的整体架构设计是合理的，clutch模式的遥操控制比官方的持续模式更灵活。

