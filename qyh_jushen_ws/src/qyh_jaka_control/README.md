# JAKA双臂机器人VR控制系统

# JAKA双臂机器人控制系统

## 系统架构

### 统一控制节点 (jaka_control_node)
**完整集成所有功能的单一节点**，包括：

#### 1. 基础控制功能
- **连接管理**: 自动连接到双臂机器人
- **电源控制**: Power On / Power Off
- **使能控制**: Enable / Disable  
- **错误处理**: Clear Error
- **急停**: Motion Abort

#### 2. 伺服模式（125Hz实时控制）
- **关节空间伺服**: 14自由度（双臂各7个关节）
- **笛卡尔空间伺服**: 位置+姿态控制
- **滤波器配置**: LPF、NLF多种滤波模式
- **EtherCAT同步**: 8ms周期精确控制

#### 3. VR跟随功能
- **实时位姿跟随**: 订阅VR控制器位姿
- **坐标系变换**: VR空间→机器人空间
- **校准功能**: 可配置坐标映射关系
- **误差监控**: 实时跟随误差反馈

#### 4. 数据录制（具身智能训练）
- **CSV格式**: 时间戳、VR位姿、关节状态、笛卡尔位姿
- **可配置频率**: 1-125Hz
- **自动文件管理**: 时间戳命名
- **实时统计**: 帧数、时长监控

## ROS服务接口

### 基础控制服务
```bash
# 上电
ros2 service call /jaka/robot/power_on std_srvs/srv/Trigger

# 下电
ros2 service call /jaka/robot/power_off std_srvs/srv/Trigger

# 使能
ros2 service call /jaka/robot/enable std_srvs/srv/Trigger

# 去使能
ros2 service call /jaka/robot/disable std_srvs/srv/Trigger

# 清除错误
ros2 service call /jaka/robot/clear_error std_srvs/srv/Trigger

# 急停
ros2 service call /jaka/robot/motion_abort std_srvs/srv/Trigger
```

### 伺服控制服务
```bash
# 启动伺服模式
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo

# 停止伺服模式
ros2 service call /jaka/servo/stop qyh_jaka_control_msgs/srv/StopServo
```

### VR跟随服务
```bash
# 启用VR跟随
ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: true}"

# 校准VR坐标系
ros2 service call /jaka/vr/calibrate qyh_jaka_control_msgs/srv/CalibrateVR

# 开始录制
ros2 service call /jaka/vr/start_recording qyh_jaka_control_msgs/srv/StartRecording \
  "{output_path: '/tmp/demo', recording_frequency: 125.0}"

# 停止录制
ros2 service call /jaka/vr/stop_recording qyh_jaka_control_msgs/srv/StopRecording
```

## 话题接口

### 发布话题
- `/jaka/servo/status` (JakaServoStatus) - 伺服状态
- `/jaka/vr/status` (VRFollowStatus) - VR跟随状态

### 订阅话题
- `/jaka/servo/joint_cmd` (JakaDualJointServo) - 关节空间指令
- `/jaka/servo/cartesian_cmd` (JakaDualCartesianServo) - 笛卡尔空间指令
- `/vr/left_controller` (VRPose) - 左VR控制器位姿
- `/vr/right_controller` (VRPose) - 右VR控制器位姿

## 使用方法

### 方式1：使用GUI（推荐）
```bash
ros2 launch qyh_jaka_control_gui jaka_gui.launch.py
```
GUI提供可视化控制，包含所有基础和高级功能。

### 方式2：使用Launch文件
```bash
# 默认配置
ros2 launch qyh_jaka_control jaka_control.launch.py

# 自定义配置
ros2 launch qyh_jaka_control jaka_control.launch.py \
  robot_ip:=192.168.1.100 \
  auto_connect:=true \
  auto_power_on:=true \
  auto_enable:=true
```

### 方式3：直接运行节点
```bash
ros2 run qyh_jaka_control jaka_control_node \
  --ros-args \
  -p robot_ip:=192.168.2.200 \
  -p auto_connect:=true
```

## 完整工作流程

### 场景1：基础运动控制
```bash
# 1. 启动节点
ros2 launch qyh_jaka_control jaka_control.launch.py

# 2. 上电
ros2 service call /jaka/robot/power_on std_srvs/srv/Trigger

# 3. 使能
ros2 service call /jaka/robot/enable std_srvs/srv/Trigger

# 4. 启动伺服模式
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo

# 5. 发送控制指令（通过ROS话题）
ros2 topic pub /jaka/servo/joint_cmd qyh_jaka_control_msgs/msg/JakaDualJointServo ...

# 6. 完成后停止
ros2 service call /jaka/servo/stop qyh_jaka_control_msgs/srv/StopServo
```

### 场景2：VR跟随+数据录制
```bash
# 1-4步同上，启动伺服模式

# 5. 校准VR坐标系（首次使用）
ros2 service call /jaka/vr/calibrate qyh_jaka_control_msgs/srv/CalibrateVR

# 6. 启用VR跟随
ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: true}"

# 7. 开始录制
ros2 service call /jaka/vr/start_recording qyh_jaka_control_msgs/srv/StartRecording \
  "{output_path: '/home/user/datasets/demo01', recording_frequency: 125.0}"

# 8. 进行VR操作演示...

# 9. 停止录制
ros2 service call /jaka/vr/stop_recording qyh_jaka_control_msgs/srv/StopRecording

# 10. 停止VR跟随
ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: false}"
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
