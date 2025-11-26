# VR遥操作完整系统集成指南

## 系统架构

完整的VR遥操作数据流：

```
VR手柄 → VR接口 → 虚拟臂(MoveIt) → 遥操作控制器 → JAKA桥接 → 真实机器人
   ↓         ↓          ↓              ↓              ↓
 90Hz    坐标变换   可视化      差分IK+平滑      125Hz伺服
         滤波                   安全检查        轨迹缓冲
```

### 各模块功能

1. **VR接口** (`vr_interface_node`)
   - 接收VR手柄姿态 (90Hz)
   - TF2坐标变换
   - 移动平均滤波
   - 死区过滤
   - 发布: `/vr/left_target_pose`, `/vr/right_target_pose`

2. **虚拟臂** (MoveIt2 `move_group`)
   - 提供IK求解器
   - 可视化虚拟臂
   - 碰撞检测
   - 发布: `/virtual/joint_states`

3. **遥操作控制器** (`teleoperation_node`)
   - 差分IK (Damped Least Squares)
   - 三阶段轨迹平滑 (速度/加速度/jerk限制)
   - 安全检查 (关节限位/碰撞/奇异)
   - 发布: `/left_arm/joint_command`, `/right_arm/joint_command`

4. **JAKA桥接** (`jaka_bridge_node`)
   - 订阅关节命令
   - 轨迹缓冲器 (10个点)
   - 插值平滑
   - 125Hz EtherCAT同步伺服
   - 性能监控

5. **真实机器人** (JAKA Zu7双臂)
   - 接收125Hz伺服命令
   - 执行平滑运动
   - 反馈关节状态

## 编译

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws

# 编译所有包
colcon build

# 或分别编译
colcon build --packages-select qyh_vr_calibration
colcon build --packages-select qyh_teleoperation_controller
colcon build --packages-select qyh_jaka_control

source install/setup.bash
```

## 运行

### 方式1: 使用完整系统launch

```bash
# 使用真实VR设备
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200

# 使用VR模拟器测试
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    use_simulator:=true

# 不显示RViz
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    rviz:=false
```

### 方式2: 分步启动（调试用）

**终端1: VR接口**
```bash
# 使用真实VR
ros2 launch qyh_vr_calibration vr_interface.launch.py

# 或使用模拟器
ros2 launch qyh_vr_calibration test_vr_interface.launch.py \
    motion_type:=circle \
    amplitude:=0.05
```

**终端2: MoveIt虚拟臂**
```bash
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py
```

**终端3: 遥操作控制器**
```bash
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py
```

**终端4: JAKA桥接**
```bash
ros2 launch qyh_jaka_control jaka_bridge.launch.py robot_ip:=192.168.2.200
```

**终端5: 启动伺服**
```bash
ros2 service call /jaka/bridge/start_servo std_srvs/srv/Trigger
```

## 监控和调试

### 查看话题

```bash
# 列出所有话题
ros2 topic list

# 查看VR目标姿态
ros2 topic echo /vr/left_target_pose
ros2 topic hz /vr/left_target_pose  # 应该是~90Hz

# 查看关节命令
ros2 topic echo /left_arm/joint_command
ros2 topic hz /left_arm/joint_command  # 应该是~125Hz

# 查看关节状态
ros2 topic echo /joint_states
ros2 topic hz /joint_states  # 应该是~125Hz
```

### 查看性能

```bash
# 停止伺服时会自动打印统计信息
ros2 service call /jaka/bridge/stop_servo std_srvs/srv/Trigger

# 输出示例:
# Left arm stats:
#   Frequency: 124.8 Hz
#   Latency: 2.31 ms
#   Errors: 0
#   Buffer overflows: 3
```

### RViz可视化

```bash
# 单独启动RViz
rviz2 -d $(ros2 pkg prefix qyh_dual_arms_moveit_config)/share/qyh_dual_arms_moveit_config/config/moveit.rviz

# 查看:
# - Virtual arms (虚拟臂)
# - Planning scene (碰撞场景)
# - Motion planning (规划轨迹)
```

### 检查TF树

```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf 显示坐标系关系
```

## 参数调整

### VR接口参数 (`vr_interface_params.yaml`)

```yaml
position_smoothing_window: 5      # 增大=更平滑，延迟增加
orientation_smoothing_window: 3
position_deadzone: 0.002          # 增大=减少抖动
orientation_deadzone: 0.01
position_scale: 1.0               # <1.0=减慢运动
orientation_scale: 1.0
```

### 遥操作控制器参数 (`teleoperation_params.yaml`)

```yaml
# 差分IK
damping_factor: 0.01              # 增大=更稳定但精度降低

# 平滑器
max_joint_velocity: 1.0           # 减小=更慢更安全
max_joint_acceleration: 0.5
max_joint_jerk: 5.0
smoothing_cutoff_freq: 10.0       # 减小=更平滑但响应慢

# 安全
joint_limit_margin: 0.05          # 增大=更保守
collision_distance_threshold: 0.02
singularity_threshold: 0.05
```

### JAKA桥接参数 (`jaka_bridge_params.yaml`)

```yaml
buffer_size: 10                   # 增大=更平滑但延迟增加
interpolation_weight: 0.5         # 增大=响应快，减小=更平滑
enable_interpolation: true        # false=直接使用最新命令
```

## 常见问题

### 1. 机器人频繁报错"位置跳变过大"

**原因**: 平滑参数设置不够保守

**解决**:
```yaml
# teleoperation_params.yaml
max_joint_velocity: 0.5    # 从1.0降低到0.5
max_joint_acceleration: 0.3  # 从0.5降低到0.3

# jaka_bridge_params.yaml
interpolation_weight: 0.3  # 从0.5降低到0.3
buffer_size: 15            # 从10增加到15
```

### 2. 延迟太大，响应慢

**原因**: 滤波器窗口太大，缓冲器太大

**解决**:
```yaml
# vr_interface_params.yaml
position_smoothing_window: 3  # 从5降低到3

# jaka_bridge_params.yaml
buffer_size: 5                # 从10降低到5
interpolation_weight: 0.7     # 从0.5增加到0.7
```

### 3. 伺服模式启动失败

**检查**:
- 机器人是否已上电并使能
- IP地址是否正确
- 网络连接是否正常

```bash
# 测试连接
ping 192.168.2.200

# 检查日志
ros2 topic echo /rosout
```

### 4. 缓冲器频繁溢出

**原因**: 输入频率 > 输出频率

**解决**:
```yaml
# 增大缓冲器
buffer_size: 20

# 或检查是否有其他节点占用CPU
top
```

### 5. 轨迹不平滑，有抖动

**检查顺序**:
1. VR输入是否平滑? → 增加VR接口滤波窗口
2. 关节命令是否平滑? → 调整遥操作控制器参数
3. 机器人执行是否平滑? → 调整JAKA桥接插值参数

## 性能指标

理想性能:

| 模块 | 频率 | 延迟 |
|------|------|------|
| VR手柄 | 90Hz | - |
| VR接口 | 90Hz | ~50ms (滤波) |
| 遥操作控制器 | 125Hz | ~8ms (计算) |
| JAKA桥接 | 125Hz | ~2-5ms (插值) |
| 真实机器人 | 125Hz | ~8ms (EtherCAT) |
| **端到端** | **90Hz** | **~70-80ms** |

## 安全注意事项

⚠️ **重要**: 
1. 首次运行时，使用**低速参数**测试
2. 在机器人工作空间内放置软垫保护
3. 保持急停按钮在手边
4. 监控机器人状态和错误日志
5. 逐步增加运动速度和幅度

## 测试流程

### 阶段1: 无机器人测试
```bash
# 只测试VR接口
ros2 launch qyh_vr_calibration test_vr_interface.launch.py

# 检查话题
ros2 topic hz /vr/left_target_pose  # 应该~90Hz
```

### 阶段2: 虚拟臂测试
```bash
# VR + MoveIt + 遥操作控制器
ros2 launch qyh_vr_calibration full_teleoperation.launch.py

# 在RViz中观察虚拟臂是否跟随VR运动
# 检查是否有碰撞警告
```

### 阶段3: 真实机器人测试
```bash
# 完整系统
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    use_simulator:=true  # 先用模拟VR

# 观察机器人运动是否平滑
# 确认无报错后，切换到真实VR
```

## 开发团队

- **VR接口**: `qyh_vr_calibration`
- **遥操作控制器**: `qyh_teleoperation_controller`
- **机器人桥接**: `qyh_jaka_control`

## 许可证

Apache-2.0
