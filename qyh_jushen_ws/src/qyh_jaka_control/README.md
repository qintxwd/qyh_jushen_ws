# JAKA双臂机器人VR控制系统

## 系统架构

### 1. 消息和服务定义 (qyh_jaka_control_msgs)
定义了所有通信接口：
- 消息：VRPose, VRFollowStatus, JakaDualJointServo, JakaDualCartesianServo, JakaServoStatus
- 服务：EnableVRFollow, StartRecording, StopRecording, CalibrateVR, StartServo, StopServo

### 2. 核心控制节点 (qyh_jaka_control)
- **jaka_interface**: JAKA SDK C++包装层，提供线程安全的机器人控制接口
- **jaka_servo_node_vr**: 主控制节点，实现125Hz实时伺服控制、VR跟随、数据录制

### 3. 图形界面 (qyh_jaka_control_gui)
- **vr_control_gui**: PyQt5图形界面，提供伺服控制、VR跟随、数据录制的可视化操作

## 功能特点

### VR跟随控制
- 实时订阅VR控制器位姿 (`/vr/left_controller_pose`, `/vr/right_controller_pose`)
- 通过TF2进行坐标变换（VR空间→机器人空间）
- 支持VR坐标系校准
- 可配置跟随参数（缩放比例、偏移、旋转等）

### 数据录制
- 125Hz高频数据采集
- CSV格式存储：时间戳、VR位姿、机器人关节角度/速度、笛卡尔位姿
- 可配置录制路径和频率
- 支持实时录制开始/停止

### 实时伺服控制
- 125Hz (8ms周期) EtherCAT同步控制
- 支持关节空间和笛卡尔空间伺服
- 可配置滤波参数（位置、速度、加速度）
- 实时状态监控（周期时间、延迟、发布频率）

## 使用流程

### 1. 编译安装
```bash
cd ~/qyh_jushen_ws
colcon build --packages-select qyh_jaka_control_msgs qyh_jaka_control qyh_jaka_control_gui
source install/setup.bash
```

### 2. 配置机器人参数
编辑 `src/qyh_jaka_control/config/robot_config.yaml`：
```yaml
robot:
  left_arm_ip: "192.168.1.10"
  right_arm_ip: "192.168.1.11"
  # ... 其他参数
```

### 3. 启动VR跟随系统

#### 方式1：使用GUI（推荐）
```bash
ros2 launch qyh_jaka_control_gui vr_control_gui.launch.py
```
操作步骤：
1. 点击"启动伺服模式"
2. 点击"校准VR坐标系"（首次使用或坐标系变化时）
3. 点击"启用VR跟随"
4. 设置录制路径，点击"开始录制"
5. 完成后点击"停止录制"
6. 点击"停止VR跟随"和"停止伺服模式"

#### 方式2：使用launch文件
```bash
ros2 launch qyh_jaka_control jaka_servo.launch.py
```

### 4. 单独启动控制节点
```bash
ros2 run qyh_jaka_control jaka_servo_node_vr
```

### 5. 命令行操作
```bash
# 启动伺服模式
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo

# 启用VR跟随
ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: true}"

# 开始录制
ros2 service call /jaka/vr/start_recording qyh_jaka_control_msgs/srv/StartRecording \
  "{output_path: '/tmp/demo_recording', recording_frequency: 125.0}"

# 停止录制
ros2 service call /jaka/vr/stop_recording qyh_jaka_control_msgs/srv/StopRecording
```

## 数据格式

录制的CSV文件包含以下列：
- `timestamp`: 时间戳（秒）
- `left_vr_x/y/z`, `left_vr_qx/qy/qz/qw`: 左VR控制器位姿
- `right_vr_x/y/z`, `right_vr_qx/qy/qz/qw`: 右VR控制器位姿
- `left_joint_0~5`, `right_joint_0~5`: 关节角度（弧度）
- `left_joint_vel_0~5`, `right_joint_vel_0~5`: 关节速度（rad/s）
- `left_cart_x/y/z`, `left_cart_rx/ry/rz`: 左臂笛卡尔位姿
- `right_cart_x/y/z`, `right_cart_rx/ry/rz`: 右臂笛卡尔位姿

## 安全注意事项

1. **首次运行前**：确认机器人周围无障碍物，设置合适的工作空间限制
2. **紧急停止**：GUI中有"停止伺服模式"按钮，命令行可使用 `ros2 service call /jaka/servo/stop`
3. **VR跟随参数**：初始使用较小的缩放比例（0.5），确认安全后再调整
4. **速度限制**：在配置文件中设置合理的最大速度和加速度

## 故障排查

### 无法连接机器人
- 检查IP地址是否正确
- 确认网络连接正常
- 查看日志：`ros2 topic echo /rosout`

### VR跟随不准确
- 重新校准VR坐标系
- 调整配置文件中的VR变换参数
- 检查TF树：`ros2 run tf2_tools view_frames`

### 录制数据丢失
- 确认输出路径存在且有写权限
- 检查磁盘空间
- 降低录制频率（如改为60Hz）

## 开发扩展

### 添加新的VR输入源
实现VR位姿发布节点，发布到 `/vr/left_controller_pose` 和 `/vr/right_controller_pose` 话题。

### 自定义滤波器
修改 `jaka_servo_node_vr.cpp` 中的滤波参数，或实现自定义滤波算法。

### 集成具身智能训练
录制的CSV文件可直接用于训练，或转换为HDF5/TFRecord等格式。

## 相关资源

- JAKA SDK文档
- ROS2 Humble文档: https://docs.ros.org/en/humble/
- TF2教程: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
