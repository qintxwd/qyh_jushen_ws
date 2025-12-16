# 机械臂姿态标定GUI使用指南

## 功能说明

图形界面工具，用于：
1. **调整机械臂姿态**：通过滑块控制左右臂各7个关节
2. **实时查看位姿**：显示末端执行器相对base_link的XYZ和RPY
3. **一键保存**：保存当前姿态到标定配置文件

---

## 快速开始

### 1. 启动仿真环境

```bash
# 终端1: 启动Gazebo仿真
cd ~/qyh_jushen_ws
source install/setup.bash
ros2 launch dual_arms gazebo.launch.py
```

### 2. 启动标定GUI

```bash
# 终端2: 启动标定工具
ros2 run dual_arms robot_pose_calibration_gui.py
```

### 3. 使用GUI调整机械臂

![GUI界面布局]

```
┌─────────────────────────────────────────────────────────┐
│          机械臂姿态标定工具                              │
├──────────────────┬──────────────────────────────────────┤
│  左臂控制        │  右臂控制                            │
│                  │                                      │
│  关节1: [滑块]   │  关节1: [滑块]                       │
│  关节2: [滑块]   │  关节2: [滑块]                       │
│  ...             │  ...                                 │
│  关节7: [滑块]   │  关节7: [滑块]                       │
│                  │                                      │
│  末端位姿:       │  末端位姿:                           │
│  X: 0.5000 m     │  X: 0.5000 m                        │
│  Y: 0.3000 m     │  Y: -0.3000 m                       │
│  Z: 0.8000 m     │  Z: 0.8000 m                        │
│  Roll: 0.00°     │  Roll: 0.00°                        │
│  Pitch: 90.00°   │  Pitch: 90.00°                      │
│  Yaw: 0.00°      │  Yaw: 0.00°                         │
└──────────────────┴──────────────────────────────────────┘
│ 姿态保存:                                               │
│ 选择姿态: [arms_open ▼]                                 │
│ [保存左臂位姿] [保存右臂位姿] [保存双臂位姿]             │
└─────────────────────────────────────────────────────────┘
```

---

## 标定流程

### 姿态1: arms_open (双臂打开)

1. **调整机械臂**：
   - 移动滑块，让双臂水平展开，与肩平齐
   - 观察仿真环境，确保姿态正确

2. **记录位姿**：
   - 查看"末端位姿"区域的XYZ值
   - 选择姿态：`arms_open`
   - 点击"保存双臂位姿"

### 姿态2: arms_down (双臂垂直放下)

1. 调整机械臂至自然下垂状态
2. 选择姿态：`arms_down`
3. 点击"保存双臂位姿"

### 姿态3: arms_forward (双臂水平前伸)

1. 调整机械臂向前伸展，高度适中
2. 选择姿态：`arms_forward`
3. 点击"保存双臂位姿"

### 姿态4: arms_forward_low (双臂低位前伸)

1. 调整机械臂向前伸展，但高度较低（类似端托盘）
2. 选择姿态：`arms_forward_low`
3. 点击"保存双臂位姿"

---

## 保存的数据格式

数据保存到：`config/robot_calibration_poses.yaml`

```yaml
calibration_poses:
  arms_open:
    left_arm:
      position:
        x: 0.5000
        y: 0.3500
        z: 0.8000
    right_arm:
      position:
        x: 0.5000
        y: -0.3500
        z: 0.8000
  # ... 其他姿态
```

---

## 技术细节

### 关节控制

- **话题**: `/joint_commands` (sensor_msgs/JointState)
- **范围**: 每个关节 ±180° (-3.14 ~ 3.14 rad)
- **分辨率**: 0.01 rad

### 位姿获取

- **方法**: TF2监听
- **Frame链**: `base_link` → `left_Link7` / `right_Link7`
- **更新频率**: 10 Hz

### GUI更新

- **位姿刷新**: 100ms (10 Hz)
- **关节控制**: 实时响应滑块变化

---

## 常见问题

### Q: GUI显示"--"，无法获取位姿？

**A**: 检查：
1. Gazebo是否启动
2. 机器人模型是否加载成功
3. TF树是否发布：
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Q: 滑块移动，仿真机械臂不动？

**A**: 
1. 确认是否使用了joint_state_publisher或controller
2. 检查`/joint_commands`话题：
   ```bash
   ros2 topic echo /joint_commands
   ```
3. 可能需要修改launch文件启用joint_trajectory_controller

### Q: 保存后找不到配置文件？

**A**: 文件位置：
```bash
ls ~/qyh_jushen_ws/src/dual_arms/config/robot_calibration_poses.yaml
```

### Q: 需要调整末端frame名称？

**A**: 编辑脚本第30-32行：
```python
self.left_ee_frame = 'left_Link7'  # 改为实际末端frame
self.right_ee_frame = 'right_Link7'
```

查看可用frames:
```bash
ros2 run tf2_ros tf2_echo base_link [TAB自动补全]
```

---

## 替代方案

### 方案1: 使用MoveIt示教

```bash
ros2 launch dual_arms moveit_demo.launch.py
```

通过MoveIt的运动规划界面拖拽末端执行器。

### 方案2: 使用joint_state_publisher_gui

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

简单的关节滑块界面（不显示末端位姿）。

---

## 后续步骤

完成4个姿态标定后：

```bash
# 运行VR映射标定
ros2 run dual_arms calibrate_vr_mapping.py
```

标定工具会自动读取`robot_calibration_poses.yaml`中的数据。

---

## 依赖项

- Python 3
- tkinter (通常已安装)
- scipy
- yaml
- ROS2 packages: sensor_msgs, tf2_ros

安装缺失依赖：
```bash
pip install scipy pyyaml
```

---

## 调试

### 查看关节名称

```bash
ros2 topic echo /joint_states --once
```

### 查看TF树

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 查看末端位姿

```bash
ros2 run tf2_ros tf2_echo base_link left_Link7
```

---

## 联系支持

遇到问题请查看终端输出日志或联系技术支持。
