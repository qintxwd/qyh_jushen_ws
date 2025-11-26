# VR Interface for Teleoperation

VR接口增强版，提供VR姿态处理、坐标变换、滤波和按键处理功能。

## 功能特性

### 1. 坐标变换
- 自动将VR坐标系转换到机器人基座坐标系
- 使用TF2实现动态坐标变换
- 支持自定义frame_id

### 2. 姿态滤波
- **位置滤波**: 移动平均滤波器（可配置窗口大小）
- **姿态滤波**: 四元数移动平均 + 归一化
- **死区处理**: 过滤微小抖动（位置2mm，姿态0.57度）

### 3. 运动缩放
- 位置缩放: 可将VR运动按比例映射到机器人运动
- 姿态缩放: 使用SLERP进行平滑姿态插值

### 4. 控制状态
- 按键启停控制
- 外部enable/disable接口
- 状态反馈发布

## 节点说明

### vr_interface_node
**订阅话题:**
- `/vr/left_hand/pose` (geometry_msgs/PoseStamped) - 左手VR姿态
- `/vr/right_hand/pose` (geometry_msgs/PoseStamped) - 右手VR姿态
- `/vr/buttons` (sensor_msgs/Joy) - VR控制器按键
- `/vr/control_enable` (std_msgs/Bool) - 外部控制启停

**发布话题:**
- `/vr/left_target_pose` (geometry_msgs/PoseStamped) - 左臂目标姿态
- `/vr/right_target_pose` (geometry_msgs/PoseStamped) - 右臂目标姿态
- `/vr/control_status` (std_msgs/Bool) - 控制状态

**参数:**
```yaml
vr_frame_id: "vr_world"              # VR坐标系
robot_frame_id: "base_link"          # 机器人坐标系
position_smoothing_window: 5          # 位置滤波窗口
orientation_smoothing_window: 3       # 姿态滤波窗口
position_deadzone: 0.002              # 位置死区 (2mm)
orientation_deadzone: 0.01            # 姿态死区 (~0.57度)
position_scale: 1.0                   # 位置缩放
orientation_scale: 1.0                # 姿态缩放
control_enabled: true                 # 初始控制状态
```

### vr_simulator_node (测试用)
模拟VR控制器输入，用于系统测试。

**发布话题:**
- `/vr/left_hand/pose` - 模拟左手姿态
- `/vr/right_hand/pose` - 模拟右手姿态
- `/vr/buttons` - 模拟按键

**参数:**
```yaml
publish_rate: 90.0                    # 发布频率 (Hz)
motion_type: "circle"                 # 运动类型: circle, figure8, vertical, static
motion_amplitude: 0.1                 # 运动幅度 (m)
motion_frequency: 0.2                 # 运动频率 (Hz)
```

## 使用方法

### 1. 编译
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_vr_calibration
source install/setup.bash
```

### 2. 启动VR接口（使用真实VR）
```bash
# 确保VR硬件节点已启动并发布 /vr/left_hand/pose 和 /vr/right_hand/pose

# 启动VR接口
ros2 launch qyh_vr_calibration vr_interface.launch.py
```

### 3. 测试VR接口（使用模拟器）
```bash
# 启动VR模拟器 + VR接口
ros2 launch qyh_vr_calibration test_vr_interface.launch.py

# 自定义运动类型
ros2 launch qyh_vr_calibration test_vr_interface.launch.py \
    motion_type:=figure8 \
    amplitude:=0.1 \
    frequency:=0.2

# 检查输出话题
ros2 topic echo /vr/left_target_pose
ros2 topic hz /vr/left_target_pose
```

### 4. 完整系统测试
```bash
# 启动完整的VR遥操作系统
# VR接口 → 虚拟臂(MoveIt) → 遥操作控制器
ros2 launch qyh_vr_calibration full_teleoperation.launch.py

# 或使用模拟VR输入
ros2 launch qyh_vr_calibration full_teleoperation.launch.py use_simulator:=true
```

### 5. 运行时控制
```bash
# 启用/禁用控制
ros2 topic pub /vr/control_enable std_msgs/Bool "data: true"
ros2 topic pub /vr/control_enable std_msgs/Bool "data: false"

# 查看控制状态
ros2 topic echo /vr/control_status
```

## 按键映射（待根据实际VR设备调整）

当前实现：
- **Button 0 (Trigger)**: 切换控制启停

可根据实际VR控制器修改`vr_interface_node.py`中的`button_callback`函数。

## 与真实VR设备集成

当使用Pico4或其他VR设备时，需要：

1. 确认VR设备发布的话题名称
2. 在launch文件中添加topic remapping：
```python
remappings=[
    ('/vr/left_hand/pose', '/pico4/left_controller/pose'),
    ('/vr/right_hand/pose', '/pico4/right_controller/pose'),
    ('/vr/buttons', '/pico4/buttons'),
]
```

3. 配置VR坐标系到机器人坐标系的TF变换

## 性能指标

- 输入频率: 90Hz (VR典型刷新率)
- 输出频率: 与输入相同
- 滤波延迟: 约50ms (5个样本 @ 90Hz)
- 坐标变换延迟: <1ms

## 调试

### 检查话题
```bash
# 列出所有VR相关话题
ros2 topic list | grep vr

# 查看消息内容
ros2 topic echo /vr/left_hand/pose
ros2 topic echo /vr/left_target_pose

# 检查频率
ros2 topic hz /vr/left_hand/pose
ros2 topic hz /vr/left_target_pose
```

### 查看TF树
```bash
ros2 run tf2_tools view_frames
# 生成 frames.pdf 显示坐标系关系
```

### 调整滤波参数
如果发现运动过于平滑（延迟大）或抖动严重，可以调整：
```bash
ros2 param set /vr_interface_node position_smoothing_window 3  # 减小延迟
ros2 param set /vr_interface_node position_deadzone 0.005      # 增大死区
```

## 下一步

Task 6已完成，下一步是Task 7: 增强机器人桥接接口（qyh_jaka_control）
- 添加轨迹缓冲器
- 实现125Hz平滑伺服接口
- 集成JAKA SDK
