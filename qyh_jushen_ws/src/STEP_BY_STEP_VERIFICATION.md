# VR双臂遥操作系统 - 分步验证手册

**⚠️ 安全第一！本文档用于真机测试前的逐步验证**

**验证日期**: _________  
**操作人员**: _________  
**设备状态**: 机械臂已上电 ☐  急停可触及 ☐  工作区域清空 ☐

---

## 🎯 验证总体策略

```
阶段0: 预检查（机械臂不动）
   ↓
阶段1: 单节点验证（仅数据流）
   ↓
阶段2: 双节点集成（仍不动机械臂）
   ↓
阶段3: 机械臂上电验证（示教模式）
   ↓
阶段4: 伺服控制验证（小范围运动）
   ↓
阶段5: VR遥操作验证（限制速度）
   ↓
阶段6: 完整系统验证（全功能）
```

**原则**:
- ✅ 每个阶段必须全部通过才能进入下一阶段
- ✅ 发现问题立即停止，不可强行继续
- ✅ 每个阶段都要有紧急停止预案
- ✅ 首次运动必须在机械臂正零位附近

---

## 📋 阶段0: 预检查（不启动任何节点）

### 0.1 硬件检查

**检查清单**:
```
☐ 机械臂外观完好，无损坏
☐ 电源线连接牢固
☐ 网络连接正常（Jetson ↔ JAKA控制器）
☐ 急停按钮功能正常
☐ 工作区域无障碍物
☐ 操作人员了解急停位置
☐ 有第二人在场监督
```

### 0.2 网络连接验证

```bash
# 在Jetson上执行
ping 192.168.2.200

# 预期输出: 正常ping通，延迟<10ms
# ✅ 通过: 延迟稳定
# ❌ 失败: 检查网络配置
```

### 0.3 ROS2环境检查

```bash
# 检查ROS2安装
ros2 --version
# 预期: ros2 humble

# 检查包是否编译
source ~/qyh_jushen_ws/install/setup.bash
ros2 pkg list | grep qyh

# 预期输出:
# qyh_dual_arm_ik_solver
# qyh_dual_arm_teleop
# qyh_jaka_control
# qyh_vr_bridge
```

**✅ 阶段0完成**: 全部检查通过 ☐

---

## 📡 阶段1: 单节点验证（数据流测试）

### 1.1 验证节点1: vr_bridge_node

**目的**: 验证VR数据接收和坐标转换

#### 启动节点
```bash
# 终端1
ros2 run qyh_vr_bridge vr_bridge_node
```

#### 验证步骤

**1.1.1 检查节点状态**
```bash
# 终端2
ros2 node list
# 预期输出: /vr_bridge_node

ros2 node info /vr_bridge_node
# 检查: Publishers和Subscribers列表
```

**1.1.2 检查发布的Topic**
```bash
# 查看所有VR相关topic
ros2 topic list | grep vr

# 预期输出:
# /vr/head/pose
# /vr/left_controller/pose
# /vr/left_controller/joy
# /vr/right_controller/pose
# /vr/right_controller/joy
```

**1.1.3 检查VR手柄数据**
```bash
# 监听左手柄位姿
ros2 topic echo /vr/left_controller/pose

# 预期输出 (PICO4开机后):
# header:
#   frame_id: 'vr_origin'
# pose:
#   position: {x: 0.xxx, y: 0.xxx, z: 0.xxx}
#   orientation: {x: 0.xxx, y: 0.xxx, z: 0.xxx, w: 0.xxx}

# ✅ 通过标准:
# - frame_id是vr_origin
# - 移动手柄时数值变化
# - X轴: 向前移动增大（ROS坐标系）
# - Y轴: 向左移动增大
# - Z轴: 向上移动增大
```

**1.1.4 验证坐标系对齐（位置）**
```bash
# 测试方法: 物理移动PICO手柄，观察数据
# 手柄向前推 → X增大 ✅
# 手柄向左移 → Y增大 ✅
# 手柄向上移 → Z增大 ✅

# 记录测试结果:
# 向前: X从 _____ 变为 _____ ☐
# 向左: Y从 _____ 变为 _____ ☐
# 向上: Z从 _____ 变为 _____ ☐
```

**1.1.5 验证姿态对齐（roll/pitch/yaw）⭐重要**
```bash
# 方法1: 观察四元数变化
ros2 topic echo /vr/left_controller/pose | grep -A 4 "orientation"

# 测试姿态变化:
# 1. 保持手柄位置不变，只旋转手腕
# 2. 观察四元数(x,y,z,w)的变化

# 基准姿态（手掌向下，手指向前）:
# orientation: {x: ___, y: ___, z: ___, w: ___}

# ✅ 通过标准: 旋转手柄时四元数变化
```

```bash
# 方法2: 使用Python脚本转换为欧拉角（更直观）
cat > /tmp/test_vr_orientation.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import numpy as np

class OrientationMonitor(Node):
    def __init__(self):
        super().__init__('orientation_monitor')
        self.sub = self.create_subscription(
            PoseStamped, '/vr/left_controller/pose', self.callback, 10)
        
    def callback(self, msg):
        q = msg.pose.orientation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        print(f"Roll: {roll:7.2f}° | Pitch: {pitch:7.2f}° | Yaw: {yaw:7.2f}°")

rclpy.init()
node = OrientationMonitor()
rclpy.spin(node)
EOF

# 运行监控脚本
python3 /tmp/test_vr_orientation.py

# 测试姿态（ROS标准坐标系）:
# Roll (绕X轴):  手掌左右翻转 → Roll变化 ☐
# Pitch (绕Y轴): 手腕上下摆动 → Pitch变化 ☐
# Yaw (绕Z轴):   手掌左右转动 → Yaw变化 ☐

# 记录测试:
# Roll:  从 _____° 变为 _____° ☐
# Pitch: 从 _____° 变为 _____° ☐
# Yaw:   从 _____° 变为 _____° ☐

# ✅ 通过标准:
# - 三个轴的旋转都能正确反映
# - 旋转方向符合右手系
# - 无万向节锁或奇异点
```

**1.1.6 验证姿态坐标系转换正确性**
```bash
# 重要: 验证PICO坐标系 → ROS坐标系的姿态转换

# 测试场景1: 手柄向前平放（手掌向下）
# PICO坐标系: 这是手柄的"自然"朝向
# ROS坐标系: 应该对应 Roll≈0°, Pitch≈0°, Yaw≈0° (或接近基准姿态)
# 记录: Roll=_____°, Pitch=_____°, Yaw=_____° ☐

# 测试场景2: 手柄向上翻转90°（手掌向上）
# 预期: Roll变化约±180°
# 记录: Roll=_____°, Pitch=_____°, Yaw=_____° ☐

# 测试场景3: 手腕向下垂90°（手指向下）
# 预期: Pitch变化约-90°
# 记录: Roll=_____°, Pitch=_____°, Yaw=_____° ☐

# 测试场景4: 手掌向左旋转90°
# 预期: Yaw变化约+90° (左转为正)
# 记录: Roll=_____°, Pitch=_____°, Yaw=_____° ☐

# ✅ 通过: 所有姿态变化符合ROS标准坐标系（REP-103）
# ❌ 失败: 检查vr_bridge中的四元数转换代码
```

**1.1.7 检查TF发布**
```bash
# 查看TF树
ros2 run tf2_ros tf2_echo vr_origin vr_left_controller

# 预期: 输出变换矩阵（包含位置和旋转）
# ✅ 通过: 有输出且随手柄移动/旋转变化
# ❌ 失败: 检查vr_bridge日志
```

**1.1.8 检查发布频率**
```bash
ros2 topic hz /vr/left_controller/pose

# 预期输出: 60-100 Hz
# ✅ 通过: 频率稳定在60Hz以上
# ❌ 失败: 检查UDP网络
```

**✅ 节点1验证完成**: 全部测试通过 ☐  
**停止节点**: Ctrl+C

---

### 1.2 验证节点2+3: teleop_manager + coordinate_mapper

**目的**: 验证遥操作管理和坐标映射

#### 启动节点
```bash
# 终端1: 继续运行vr_bridge
ros2 run qyh_vr_bridge vr_bridge_node

# 终端2: 启动teleop（包含manager+mapper）
ros2 launch qyh_dual_arm_teleop teleop.launch.py
```

#### 验证步骤

**1.2.1 检查节点启动**
```bash
# 终端3
ros2 node list

# 预期输出:
# /vr_bridge_node
# /teleop_manager_node
# /coordinate_mapper_node
```

**1.2.2 检查静态TF发布**
```bash
# 验证teleop_base发布
ros2 run tf2_ros tf2_echo base_link teleop_base

# 预期输出: 静态变换 (x: 0.5, y: 0, z: 0 或配置值)
# ✅ 通过: 有输出
```

**1.2.3 测试Clutch机制（增量模式）**
```bash
# 监听左手目标位姿
ros2 topic echo /teleop/left_hand/target

# 测试步骤（⭐ 关键：验证增量跟踪，非绝对位置）:
# 1. 不按grip按钮 → 应该无输出 ☐
# 2. 按下grip按钮（记录原点）→ 开始输出位姿 ☐
# 3. 移动VR手柄5cm → 目标位姿相应变化 ☐
# 4. 松开grip按钮 → 停止输出（目标位姿保持最后值）☐
# 5. 移动VR手柄到其他位置（不按grip）→ 仍无输出 ☐
# 6. 再次按下grip → 从步骤4的位置继续增量（不跳变）☐

# ✅ 通过标准:
# - 松开grip后目标位姿"冻结"在最后值
# - 再次按grip不会跳变到VR手柄的绝对位置
# - 增量累积正确（多次clutch后总位移 = 各段VR位移之和）
```

**1.2.4 验证坐标映射**
```bash
# 同时监听输入和输出
ros2 topic echo /vr/left_controller/pose &
ros2 topic echo /teleop/left_hand/target

# 验证:
# 1. 输出位姿经过滤波（变化更平滑）☐
# 2. 输出位姿有缩放（比输入大2倍）☐
# 3. frame_id正确 (应为 'vr_origin') ☐
```

**1.2.5 检查TF链完整性**
```bash
# 查看完整TF树
ros2 run tf2_tools view_frames

# 生成frames.pdf，检查:
# base_link → teleop_base ✅
# teleop_base → vr_origin ✅
# vr_origin → vr_left_controller ✅
# vr_left_controller → human_left_hand ✅
```

**1.2.6 检查发布频率**
```bash
ros2 topic hz /teleop/left_hand/target

# 预期输出: 100 Hz (按下grip时)
# ✅ 通过: 频率稳定
```

**✅ 节点2+3验证完成**: 全部测试通过 ☐  
**停止节点**: Ctrl+C (两个终端)

---

### 1.3 验证节点4: dual_arm_ik_solver（仅IK计算）

**⚠️ 重要**: 此阶段**不启动**机械臂控制节点，只验证IK计算

#### 启动节点
```bash
# 终端1: vr_bridge
ros2 run qyh_vr_bridge vr_bridge_node

# 终端2: teleop
ros2 launch qyh_dual_arm_teleop teleop.launch.py

# 终端3: IK solver
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
```

#### 验证步骤

**1.3.1 检查JAKA连接**
```bash
# 观察终端3的输出
# 预期看到: "连接到JAKA控制器 192.168.2.200 (第二个客户端)..."

# 情况A: 如果jaka_control已运行且机器人在线 → ✅ "连接成功！"
# 情况B: 如果jaka_control未运行或机器人离线 → ⚠️ "连接失败" (这也是正常的，此阶段仅验证节点启动)
```

**1.3.2 检查节点启动（即使连接失败）**
```bash
ros2 node list
# 预期输出: /dual_arm_ik_solver
```

**1.3.3 检查订阅的Topic**
```bash
ros2 node info /dual_arm_ik_solver

# 检查Subscriptions:
# - /teleop/left_hand/target ✅
# - /teleop/right_hand/target ✅
# - /joint_states ✅ (用于IK参考)

# ⚠️ 注意: 此阶段不启动jaka_control，所以没有真实的joint_states数据
# IK solver 在没有 joint_states 时会使用默认参考位置（零位或配置的初始值）
```

**1.3.4 检查发布的Topic**
```bash
ros2 topic list | grep arm

# 预期输出:
# /left_arm/joint_command
# /right_arm/joint_command
# /ik_solver/status
```

**1.3.5 模拟测试IK求解**
```bash
# 使用测试脚本发布目标位姿
cd ~/qyh_jushen_ws/src/qyh_dual_arm_ik_solver/scripts
python3 test_publish_targets.py

# 监听IK输出
ros2 topic echo /left_arm/joint_command

# ⚠️ 如果JAKA未连接，不会有输出（正常）
# 此步骤只是检查topic结构正确
```

**✅ 节点4验证完成**: Topic结构正确 ☐  
**停止节点**: Ctrl+C (所有终端)

---

## 🤖 阶段2: 机械臂上电验证（示教模式）

**⚠️ 危险等级: 低（机械臂处于示教模式）**

### 2.1 准备工作

**安全确认**:
```
☐ 工作区域清空
☐ 急停按钮可触及
☐ 机械臂处于安全位置（关节角度接近零位）
☐ 示教器已连接
☐ 操作人员了解示教器操作
```

### 2.2 启动机械臂控制节点（仅读取状态）

```bash
# 终端1: 启动jaka_control（现已集成关节名称适配器和robot_state_publisher）
# ⚠️ 注意: 只启动jaka_control.launch.py，不要启动display.launch.py
# display.launch.py是用于仿真可视化的，会启动joint_state_publisher_gui造成冲突
ros2 launch qyh_jaka_control jaka_control.launch.py
```

#### 验证步骤

**2.2.0 检查节点启动状态 ⭐ 关键**
```bash
# 检查节点列表
ros2 node list

# 预期输出（真机模式，已更新）:
# /jaka_control_node           ← 真机控制节点
# /qyh_jaka_joint_adapter      ← 关节名称适配器（新增）
# /robot_state_publisher       ← TF发布节点（已集成到launch）
# 
# ⚠️ 不应该看到: /joint_state_publisher_gui （这是仿真用的）

# 如果看到joint_state_publisher_gui，说明误启动了display.launch.py
# 需要停止所有节点，只启动jaka_control.launch.py

# 检查URDF是否加载到robot_state_publisher
ros2 param get /robot_state_publisher robot_description | head -20

# 预期输出: URDF XML内容（<robot name="qyh_dual_arms_description">）
# ✅ 通过: URDF已加载
# ❌ 失败: robot_state_publisher未启动，检查launch文件配置
```

**2.2.0.5 验证关节名称适配器工作 ⭐ 新增验证**
```bash
# 检查原始关节数据（来自JAKA控制器）
ros2 topic echo /joint_states_raw --once

# 预期输出: 关节名称可能是以下格式之一
# 格式1: r-j1, r-j2, ..., l-j1, l-j2, ...
# 格式2: left_joint1, left_joint2, ..., right_joint1, ...
# 格式3: l1, l2, ..., r1, r2, ...

# 检查适配后的关节数据（用于robot_state_publisher）
ros2 topic echo /joint_states --once

# 预期输出: 关节名称已转换为URDF标准格式
# name: ['l-j1', 'l-j2', 'l-j3', 'l-j4', 'l-j5', 'l-j6', 'l-j7',
#        'r-j1', 'r-j2', 'r-j3', 'r-j4', 'r-j5', 'r-j6', 'r-j7']

# ✅ 通过: 关节名称为 l-jN / r-jN 格式，匹配URDF
# ❌ 失败: 关节名称格式不正确，检查适配器日志

# 运行适配器测试脚本（可选，详细验证）
cd ~/qyh_jushen_ws/src/qyh_jaka_control/scripts
python3 test_joint_adapter.py

# 预期输出: 
# ✅ Joint names correctly converted to URDF format
# ✅ Joint count correct (14 joints)
# ✅ Position data preserved
# 🎉 ADAPTER WORKING CORRECTLY!
```

**2.2.1 检查连接状态**
```bash
# 观察终端1输出
# 预期: "✅ 连接成功！机器人已就绪"

# ✅ 通过: 连接成功
# ❌ 失败: 检查IP地址和网络
```

**2.2.2 检查joint_states发布**
```bash
# 终端2
ros2 topic echo /joint_states

# 预期输出: 14个关节的位置、速度、力矩
# frame_id: '' (空字符串) 或 'world'
# name: ['r-j1', 'r-j2', ..., 'r-j7', 'l-j1', 'l-j2', ..., 'l-j7']
# position: [rad × 14]
# velocity: [] (可能为空，取决于JAKA SDK返回)
# effort: [] (可能为空)

# ⚠️ 重要检查: 确认只有一个发布源
# 如果看到两种不同的关节名称格式（r-j1 和 right_joint1），
# 说明有多个节点在发布joint_states，需要停止display.launch.py

# ✅ 通过: 数据正常更新，关节名称一致
# ❌ 失败: 关节名称混乱或无数据
```

**2.2.3 检查发布频率**
```bash
# 检查适配后的关节状态频率（用于robot_state_publisher）
ros2 topic hz /joint_states

# 预期输出: 10-125 Hz (取决于JAKA控制器配置)
# 常见值: 
# - 20Hz: JAKA默认状态查询频率（非伺服模式）
# - 125Hz: 伺服模式下的频率
# 
# ✅ 通过: 频率稳定在10Hz以上
# ⚠️ 注意: 此时频率通常是20Hz，启动伺服控制后会提升到125Hz

# （可选）同时检查原始数据频率，验证适配器无延迟
ros2 topic hz /joint_states_raw
# 预期: 与 /joint_states 频率一致（适配器实时转换，无缓冲）

# （可选）验证适配器延迟
ros2 topic delay /joint_states --field header.stamp
# 预期: 延迟<5ms（适配器处理开销极小）
```

**2.2.4 验证TF发布 ⭐ 关键**
```bash
# 首先检查基础TF（静态变换，由URDF定义）
ros2 run tf2_ros tf2_echo base_link base_link_left

# 预期输出 (根据实际URDF): 
# Translation: [-0.000, 0.085, 0.003]  # X≈0, Y≈0.085m, Z≈0.003m
# Rotation: 绕Z轴-30° (RPY: [0.000, 0.000, -30.000])
# 
# ✅ 通过: 输出静态变换，Y轴偏移约0.085m，Z轴旋转约-30°
# ❌ 失败: 
#   - "Invalid frame ID" → robot_state_publisher未启动或URDF未加载
#   - 数值差异过大 → 检查URDF中的base_link_left定义

ros2 run tf2_ros tf2_echo base_link base_link_right
# 预期: 输出右臂安装点变换 (y=-0.08395, rz=30°)

# 然后检查完整机械臂TF链（动态变换，随关节运动）
ros2 run tf2_ros tf2_echo base_link lt

# 预期: 输出左臂末端位置（会随关节角度变化）
# ✅ 通过: 有输出

ros2 run tf2_ros tf2_echo base_link rt
# 预期: 输出右臂末端位置

# 检查完整TF树
ros2 run tf2_tools view_frames
# 生成frames.pdf，检查:
# base_link → base_link_left → l1 → l2 → ... → lt ✅
# base_link → base_link_right → r1 → r2 → ... → rt ✅
```

**2.2.5 手动移动机械臂测试**
```bash
# 使用示教器或手动拖动机械臂
# 同时监听joint_states

ros2 topic echo /joint_states

# 验证: 移动机械臂时，joint_states数值变化 ✅
```

**✅ 阶段2验证完成**: 机械臂状态读取正常 ☐

---

## 🎮 阶段3: IK求解器与机械臂联调（不发送指令）

**⚠️ 危险等级: 低（只计算IK，不控制机械臂）**

### 3.1 完整启动（除了实际控制）

```bash
# 终端1: jaka_control
ros2 launch qyh_jaka_control jaka_control.launch.py

# 终端2: vr_bridge
ros2 run qyh_vr_bridge vr_bridge_node

# 终端3: teleop
ros2 launch qyh_dual_arm_teleop teleop.launch.py

# 终端4: IK solver
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
```

### 3.2 验证IK求解

**3.2.1 检查双客户端连接**
```bash
# 观察终端1和终端4
# 终端1 (jaka_control): "✅ 连接成功！机器人已就绪"
# 终端4 (ik_solver): "✅ IK求解节点成功连接！"

# ✅ 通过: 两个节点都连接成功
# ❌ 失败: JAKA SDK只支持2个客户端，检查是否有其他程序连接
```

**3.2.2 测试IK求解**
```bash
# 按下VR手柄grip按钮，移动手柄
# 观察终端4的IK统计输出

# 预期输出（每秒一次）:
# "📊 IK统计: 左臂成功率=95.0%, 右臂成功率=96.0%, 总计=125次"

# ✅ 通过: IK成功率>90%
# ⚠️ 警告: 成功率<90%，检查工作空间设置
```

**3.2.3 检查安全检查功能**
```bash
# 监听IK输出
ros2 topic echo /left_arm/joint_command

# 尝试让VR手柄移动到极限位置
# 预期: 看到关节限位警告
# "⚠️ 左臂 关节X 超限: X.XXX rad"

# ✅ 通过: 安全检查生效
```

**3.2.4 验证使用实际joint作为参考**
```bash
# 手动移动机械臂到不同位置
# 然后移动VR手柄

# 检查: IK求解是否从当前位置出发（而非上次结果）
# 方法: 观察joint_command的值应该接近当前joint_states

# 验证步骤:
# 1. 监听当前关节位置（已通过适配器转换为URDF格式）
ros2 topic echo /joint_states --once

# 2. 按下VR手柄grip，移动手柄
# 3. 查看IK输出的关节指令
ros2 topic echo /left_arm/joint_command --once

# 4. 比较: joint_command 的起始值应该接近 joint_states 的当前值
#    （说明IK使用了真实关节位置作为参考，而非固定零位）

# ✅ 通过: IK参考使用实际关节位置（通过适配器获取的URDF格式数据）
# ⚠️ 注意: joint_states 已经是经过适配器转换的URDF格式（l-jN, r-jN）
```

**✅ 阶段3验证完成**: IK求解正常，安全检查有效 ☐

**停止所有节点**: Ctrl+C (保持机械臂在安全位置)

---

## ⚠️ 阶段4: 小范围伺服控制验证（真正运动）

**⚠️⚠️⚠️ 危险等级: 中高 - 机械臂将开始运动！**

### 4.1 准备工作（必须完成）

**安全确认**:
```
☐ 机械臂处于正零位或安全位置
☐ 工作区域完全清空
☐ 急停按钮随时可按
☐ 有第二人在场监督
☐ 已了解如何紧急停止
☐ 摄像头监控已开启（如有）
☐ 机械臂运动速度限制已设置
☐ VR手柄移动范围将被限制
```

### 4.2 首次运动测试

**4.2.1 限制参数配置**

编辑 `~/qyh_jushen_ws/src/qyh_dual_arm_teleop/config/teleop_params.yaml`:
```yaml
/**:
  ros__parameters:
    # Clutch阈值（滞回，防抖）
    grip_engage_threshold: 0.8    # 保持默认
    grip_release_threshold: 0.2   # 保持默认
    
    # 降低速度和缩放用于首次测试
    position_scale: 0.5      # 🔧 降低到0.5（原本2.0）
    max_position_delta: 0.02 # 🔧 降低到0.02 m（原本0.05）
    max_rotation_delta: 0.05 # 🔧 降低到0.05 rad（原本0.1）
    filter_alpha: 0.2        # 🔧 增加滤波（原本0.3）
```

**4.2.2 启动所有节点**
```bash
# 终端1: jaka_control
ros2 launch qyh_jaka_control jaka_control.launch.py

# 等待连接成功后...

# 终端2: vr_bridge
ros2 run qyh_vr_bridge vr_bridge_node

# 终端3: teleop
ros2 launch qyh_dual_arm_teleop teleop.launch.py

# 终端4: IK solver
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py

# 等待所有节点正常运行...
```

**4.2.3 首次运动测试（单臂）**

**准备**:
1. 确保只测试一只机械臂（例如左臂）
2. 右手不按grip按钮（右臂不动）
3. 操作人员站在急停按钮旁

**步骤（⭐ 重点验证增量跟踪）**:
```
1. 左手按下grip按钮（记录VR原点和机械臂原点）
2. 非常缓慢地向前移动手柄 5cm
3. 观察左臂运动（应同向移动约10cm，因position_scale=0.5时为5cm×2=10cm）
4. 松开grip按钮（机械臂应立即停止并保持当前位置）
5. 将VR手柄移回原位（机械臂不应动）
6. 再次按下grip → 机械臂保持不动（建立新原点）
7. 再次向前移动手柄5cm → 机械臂从当前位置继续前移

记录:
☐ 步骤3: 机械臂向前移动（与手柄方向一致，无跳变）
☐ 步骤4: 松开grip后机械臂立即"冻结"位置
☐ 步骤5: 移动VR手柄时机械臂保持不动
☐ 步骤6-7: 再次接合时从当前位置继续，不跳回手柄绝对位置
☐ 运动平滑无抖动
☐ 无异常声音
☐ 无碰撞或意外运动

✅ 通过标准:
- 增量跟踪正确（移动多少，机械臂就移动多少×scale）
- 松开保持最后位置（不漂移）
- 再次接合无跳变（从当前位置继续）

❌ 失败情况:
- 机械臂突然跳到VR手柄绝对位置 → 增量逻辑未生效
- 松开grip后机械臂继续漂移 → 下游节点未正确处理
- 立即按急停，检查coordinate_mapper_node日志
```

**4.2.4 方向验证测试**
```
测试每个方向:
1. 向前推 → 机械臂向前 ☐
2. 向后拉 → 机械臂向后 ☐
3. 向左移 → 机械臂向左 ☐
4. 向右移 → 机械臂向右 ☐
5. 向上移 → 机械臂向上 ☐
6. 向下移 → 机械臂向下 ☐

✅ 全部方向正确
⚠️ 有方向相反: 检查末端坐标系校正
```

**4.2.5 姿态控制测试**
```
测试旋转:
1. 手腕绕X轴旋转 → 末端绕对应轴旋转 ☐
2. 手腕绕Y轴旋转 → 末端绕对应轴旋转 ☐
3. 手腕绕Z轴旋转 → 末端绕对应轴旋转 ☐

✅ 姿态跟随正确
⚠️ 姿态不匹配: 检查四元数转换
```

**4.2.6 安全功能测试**
```
测试安全限制:
1. 快速移动手柄
   → 预期: 机械臂速度被限制 ☐

2. 移动到工作空间边缘
   → 预期: 出现IK失败或限位警告 ☐

3. 尝试奇异位置
   → 预期: IK失败，机械臂停止 ☐

✅ 安全功能正常工作
```

**✅ 阶段4验证完成**: 单臂运动正常且安全 ☐

**停止并复位**: 
1. 松开所有grip按钮
2. Ctrl+C停止所有节点
3. 使用示教器移动机械臂回零位

---

## 🚀 阶段5: 双臂协调验证（增加速度）

**⚠️⚠️ 危险等级: 高 - 双臂同时运动！**

### 5.1 准备工作

**安全确认**:
```
☐ 阶段4测试全部通过
☐ 双臂处于安全位置（互不干涉）
☐ 工作区域扩大（双臂可能碰撞范围）
☐ 急停按钮测试有效
☐ 碰撞检测已启用（如有）
```

### 5.2 逐步增加速度

**5.2.1 恢复部分速度**

编辑 `teleop_params.yaml`:
```yaml
/**:
  ros__parameters:
    position_scale: 1.0           # 🔧 恢复到1.0
    max_position_delta: 0.03      # 🔧 增加到0.03 m
    max_rotation_delta: 0.08      # 🔧 增加到0.08 rad
    filter_alpha: 0.3             # 🔧 恢复默认值
```

**5.2.2 双臂独立运动测试**
```
测试步骤:
1. 只移动左手 → 只有左臂动 ☐
2. 只移动右手 → 只有右臂动 ☐
3. 左右手交替移动 → 两臂交替运动 ☐

✅ 通过: 双臂独立控制正常
```

**5.2.3 双臂同时运动测试**
```
测试步骤:
1. 双手同时向前推 → 双臂同时向前 ☐
2. 双手同时向外 → 双臂同时向外 ☐
3. 双手做镜像动作 → 双臂镜像运动 ☐

⚠️ 注意: 监控双臂距离，防止碰撞
✅ 通过: 双臂协调运动正常
```

**5.2.4 Clutch增量机制测试（⭐ 核心功能）**
```
测试场景（验证基于原点的增量跟踪）:

场景1: 基本增量
1. 机械臂在位置A，按下grip（记录VR原点P1，目标原点A）
2. VR手柄前移10cm → 机械臂应移到 A + 10cm×scale
3. 松开grip → 机械臂停在位置B
4. 将VR手柄移回P1位置（不按grip）→ 机械臂保持在B ☐

场景2: 多次累积（验证无跳变）
5. 按下grip（记录新VR原点P2=当前手柄位置，目标原点B）
6. VR手柄再前移5cm → 机械臂应从B移到 B + 5cm×scale（位置C）
7. 松开grip → 机械臂停在C ☐
8. 总位移验证: C = A + (10+5)cm×scale ☐

场景3: 任意位置重新接合
9. 将VR手柄移到房间另一侧位置P3（距离P1很远）
10. 按下grip → 机械臂应保持在C，不跳变 ☐
11. VR手柄左移8cm → 机械臂从C左移 8cm×scale
12. 验证: 机械臂最终位置 ≠ P3的绝对映射位置 ☐

场景4: 姿态增量（同样逻辑）
13. 按下grip，记录VR姿态Q1和目标姿态R1
14. 旋转VR手柄30° → 机械臂末端应旋转 30°×rotation_scale
15. 松开grip，移动手柄到其他姿态Q2
16. 再按grip → 机械臂姿态保持，不跳到Q2对应的绝对姿态 ☐

场景5: 快速连续clutch（压力测试）
17. 快速多次 "按grip-移动5cm-松grip-移动手柄-再按grip" ☐
18. 验证每次增量都正确累加，无跳变或丢失 ☐

✅ 通过标准:
- 所有场景中机械臂位置/姿态 = 初始 + Σ(各段VR增量×scale)
- 松开grip时目标"冻结"（下游IK节点继续用最后目标解算）
- 再次按grip从"冻结位置"继续，不跳到VR绝对位置
- 与旧版Python vr_clutch_controller.py行为一致

❌ 失败: 若出现跳变或增量丢失，检查coordinate_mapper_node的HandState逻辑
```

**5.2.5 长时间运行测试**
```
测试步骤:
1. 连续操作5分钟
2. 监控性能和稳定性

记录:
- CPU使用率: _____ %
- 延迟: _____ ms
- IK成功率: _____ %
- 异常断连: _____ 次

✅ 通过: 系统稳定运行
```

**✅ 阶段5验证完成**: 双臂协调运动正常 ☐

---

## 🎯 阶段6: 完整功能验证（全速运行）

**⚠️⚠️⚠️ 危险等级: 最高 - 全速运行！**

### 6.1 恢复完整速度

编辑 `teleop_params.yaml`:
```yaml
/**:
  ros__parameters:
    position_scale: 2.0           # 🔧 恢复到2.0（完整缩放）
    max_position_delta: 0.05      # 🔧 恢复到0.05 m（单帧最大位移）
    max_rotation_delta: 0.1       # 🔧 恢复到0.1 rad（单帧最大旋转）
    filter_alpha: 0.3             # 🔧 默认值
```

### 6.2 完整功能测试

**6.2.1 复杂任务测试**
```
任务1: 双手协调拾取
1. 双臂移动到物体两侧
2. 同时抓取
3. 协调移动到目标位置
4. 同时放下

☐ 任务完成
☐ 运动流畅
☐ 无碰撞
☐ 精度满足要求
```

**6.2.2 极限工作空间测试**
```
测试每个边界:
☐ 最远到达距离
☐ 最高位置
☐ 最低位置
☐ 左右极限
☐ 奇异点处理

记录安全限制生效情况
```

**6.2.3 性能压力测试**
```
测试内容:
1. 快速往复运动（10次）☐
2. 复杂轨迹跟踪 ☐
3. 高频方向变化 ☐
4. 持续运行30分钟 ☐

记录:
- 温度: _____ °C
- 错误次数: _____
- 性能下降: 有☐ 无☐
```

**6.2.4 故障恢复测试**
```
测试场景:
1. IK失败时恢复 ☐
2. 超限后恢复 ☐
3. 断连后重连 ☐
4. 急停后恢复 ☐

✅ 通过: 故障恢复正常
```

**✅ 阶段6验证完成**: 系统完整功能正常 ☐

---

## 📊 最终验证清单

### 功能验证

```
✅ VR数据接收正常
✅ 坐标转换正确
✅ Clutch机制有效
✅ 滤波和缩放正常
✅ IK求解稳定
✅ 安全检查生效
✅ 双臂独立控制
✅ 双臂协调运动
✅ 运动方向正确
✅ 姿态跟随准确
✅ 速度限制有效
✅ 工作空间限制有效
✅ 长时间稳定运行
✅ 故障恢复正常
```

### 性能指标

```
指标                  目标        实际        通过
-----------------------------------------------------
VR数据频率           60-100Hz    _____Hz     ☐
坐标映射频率         100Hz       _____Hz     ☐
IK求解频率           125Hz       _____Hz     ☐
伺服控制频率         125Hz       _____Hz     ☐
端到端延迟           <100ms      _____ms     ☐
IK成功率             >90%        _____%      ☐
位置精度             <5mm        _____mm     ☐
姿态精度             <5°         _____°      ☐
```

### 安全验证

```
✅ 急停功能有效
✅ 速度限制工作
✅ 关节限位检查生效
✅ 工作空间限制有效
✅ 碰撞避让正常（如有）
✅ 异常停止及时
✅ 故障报警清晰
✅ 操作流程安全
```

---

## 🚨 故障排查指南

### 问题1: VR手柄数据不更新

**症状**: `/vr/left_controller/pose` 无输出或不变化

**排查步骤**:
1. 检查PICO4是否开机 ☐
2. 检查UDP端口10000是否被占用 ☐
3. 检查防火墙设置 ☐
4. 检查WiFi连接 ☐
5. 重启vr_bridge_node ☐

### 问题2: IK求解失败率高

**症状**: "IK统计: 左臂成功率=50%, ..."

**排查步骤**:
1. 检查目标位姿是否在工作空间内 ☐
2. 检查关节限位设置是否过严 ☐
3. 检查初始参考关节位置 ☐
4. 尝试移动到不同起始位置 ☐
5. 检查JAKA SDK版本 ☐

### 问题3: 机械臂运动方向错误

**症状**: VR向前推，机械臂向后移

**排查步骤**:
1. 检查vr_bridge的坐标转换 ☐
2. 检查末端坐标系校正（±90°）☐
3. 查看TF树完整性 ☐
4. 验证URDF中末端坐标系定义 ☐

### 问题4: 运动不流畅、抖动

**症状**: 机械臂运动有明显抖动

**排查步骤**:
1. 增加滤波系数（filter_alpha 0.3→0.2）☐
2. 降低速度限制 ☐
3. 检查网络延迟 ☐
4. 检查IK求解频率 ☐
5. 检查关节速度检查是否过严 ☐

### 问题5: 双客户端连接失败

**症状**: IK solver连接失败

**排查步骤**:
1. 确认jaka_control先启动 ☐
2. 检查是否有第三个程序连接 ☐
3. 重启JAKA控制器 ☐
4. 检查IP地址配置 ☐

### 问题6: 关节超限警告频繁

**症状**: 频繁看到"关节X超限"

**排查步骤**:
1. 检查安全裕度设置（SAFETY_MARGIN_POS）☐
2. 限制VR工作空间范围 ☐
3. 调整初始机械臂位置 ☐
4. 检查关节限位是否配置错误 ☐

### 问题7: 再次按grip时机械臂跳变

**症状**: 松开grip后，再按grip时机械臂突然跳到VR手柄的绝对位置

**原因**: 增量跟踪逻辑未生效，退化为绝对位置映射

**排查步骤**:
1. 检查coordinate_mapper_node是否使用更新后的代码 ☐
2. 查看节点日志，确认"Clutch thresholds: engage=0.8 release=0.2" ☐
3. 检查HandState结构体是否正确维护 have_last_target 标志 ☐
4. 调试输出: 在engage时打印 vr_origin_pos 和 target_origin_pos ☐
5. 验证公式: target = target_origin + (vr_pos - vr_origin) * scale ☐

### 问题8: 松开grip后机械臂继续漂移

**症状**: 松开grip后目标位姿停止发布，但机械臂仍在运动

**原因**: 下游节点（IK solver/controller）未正确处理"无新目标"状态

**排查步骤**:
1. 监听 /teleop/left_hand/target，确认松开grip后无新消息 ☐
2. 检查IK solver是否在无新目标时保持最后IK结果 ☐
3. 检查伺服控制器是否有超时停止机制 ☐
4. 临时方案: coordinate_mapper在松开时继续发布最后目标 ☐

### 问题9: joint_states出现两种不同的关节名称

**症状**: 
- `/joint_states`中看到`r-j1, l-j1`和`right_joint1, left_joint1`两种名称
- 或者频率异常低/高
- 或者frame_id在空字符串和'world'之间跳变

**原因**: 同时运行了多个发布joint_states的节点

**排查步骤**:
1. 检查节点列表: `ros2 node list` ☐
2. 确认是否同时运行了:
   - jaka_control.launch.py (真机控制) ☐
   - display.launch.py (仿真可视化) ☐
3. 检查是否有joint_state_publisher_gui节点 ☐
4. 解决方案:
   - **真机测试**: 只运行jaka_control.launch.py ☐
   - **仿真测试**: 只运行display.launch.py ☐
   - 停止多余节点: Ctrl+C 停止display.launch.py ☐
5. 重新验证: `ros2 topic echo /joint_states` 应只看到一种名称格式 ☐

### 问题10: 关节名称适配器未正确转换

**症状**: 
- robot_state_publisher报错: "Could not find joint 'left_joint1' in robot model"
- 或者: "Could not find joint 'l1' in robot model"
- TF树不完整，缺少机械臂链条

**原因**: 适配器未正确检测或转换关节名称格式

**排查步骤**:
1. 确认适配器节点正在运行 ☐
   ```bash
   ros2 node list | grep adapter
   # 应该看到: /qyh_jaka_joint_adapter
   ```

2. 检查适配器日志，确认格式检测 ☐
   ```bash
   ros2 node info /qyh_jaka_joint_adapter
   # 查看 "Detected format" 消息
   ```

3. 手动验证输入输出 ☐
   ```bash
   # 输入 (来自jaka_control)
   ros2 topic echo /joint_states_raw --once | grep -A 14 "name:"
   
   # 输出 (发往robot_state_publisher)
   ros2 topic echo /joint_states --once | grep -A 14 "name:"
   
   # 对比: 输出应该是 l-jN / r-jN 格式
   ```

4. 检查URDF中的关节名称定义 ☐
   ```bash
   grep "joint name=" ~/qyh_jushen_ws/src/qyh_dual_arms_description/urdf/dual_arms.urdf | grep -E "(l-j|r-j)"
   # 应该看到: l-j1 到 l-j7, r-j1 到 r-j7
   ```

5. 重启适配器节点 ☐
   ```bash
   # 先停止所有节点
   # 重新编译并启动
   cd ~/qyh_jushen_ws
   colcon build --packages-select qyh_jaka_control --symlink-install
   source install/setup.bash
   ros2 launch qyh_jaka_control jaka_control.launch.py
   ```

6. 运行适配器测试脚本 ☐
   ```bash
   cd ~/qyh_jushen_ws/src/qyh_jaka_control/scripts
   python3 test_joint_adapter.py
   # 应该显示: 🎉 ADAPTER WORKING CORRECTLY!
   ```

7. 如果适配器仍然失败，检查源代码 ☐
   - 文件: `qyh_jaka_control/scripts/qyh_jaka_joint_adapter_node.py`
   - 确认映射字典包含所有可能的输入格式
   - 添加调试输出: 打印检测到的格式和转换结果

---

## 📝 验证记录表

**验证日期**: __________  
**操作人员**: __________  
**设备信息**: JAKA Zu7 双臂 SN: __________

### 阶段完成情况

| 阶段 | 开始时间 | 结束时间 | 结果 | 备注 |
|------|---------|---------|------|------|
| 阶段0: 预检查 | ___:___ | ___:___ | ☐ | ____ |
| 阶段1: 单节点验证 | ___:___ | ___:___ | ☐ | ____ |
| 阶段2: 机械臂上电 | ___:___ | ___:___ | ☐ | ____ |
| 阶段3: IK联调 | ___:___ | ___:___ | ☐ | ____ |
| 阶段4: 小范围运动 | ___:___ | ___:___ | ☐ | ____ |
| 阶段5: 双臂协调 | ___:___ | ___:___ | ☐ | ____ |
| 阶段6: 完整功能 | ___:___ | ___:___ | ☐ | ____ |

### 异常记录

| 时间 | 阶段 | 问题描述 | 解决方案 | 结果 |
|------|------|---------|---------|------|
| ___:___ | ____ | ________ | ________ | ____ |
| ___:___ | ____ | ________ | ________ | ____ |

### 最终签字

**验证结论**: ☐ 通过  ☐ 部分通过  ☐ 未通过

**操作人员签字**: __________  **日期**: __________  
**监督人员签字**: __________  **日期**: __________  
**技术负责人**: __________  **日期**: __________

---

## 🎓 安全操作要点

**务必牢记**:

1. **急停优先**: 任何异常立即按急停
2. **逐步推进**: 不可跳过验证阶段
3. **保持距离**: 运动时保持安全距离
4. **专注观察**: 操作时全神贯注
5. **充分沟通**: 多人操作时保持沟通
6. **记录详细**: 所有异常都要记录
7. **量力而行**: 不确定时停止测试
8. **备份计划**: 准备好恢复方案

**紧急联系方式**:
- 技术支持: __________
- 设备厂商: JAKA __________
- 急救电话: 120

---

**文档版本**: v1.0  
**最后更新**: 2025-12-16  
**维护人员**: AI Assistant
