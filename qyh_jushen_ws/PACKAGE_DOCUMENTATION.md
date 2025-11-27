# QYH 巨神机器人工作空间完整说明文档

## 项目概述

本项目是一个完整的双臂人形机器人VR遥操作控制系统，支持JAKA Zu7双机械臂、JODELL EPG夹爪、HTD-85H头部舵机和Standard移动底盘的集成控制。系统通过VR设备实现实时遥操作，支持具身智能数据采集和仿真训练。

### 技术栈
- **ROS2 Humble**: 核心框架
- **MoveIt2**: 运动规划和IK求解
- **EtherCAT**: 实时伺服通信
- **Modbus RTU/TCP**: 夹爪和底盘通信
- **PyQt5**: 图形界面
- **C++17/Python3**: 开发语言

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                          VR遥操作系统                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  VR设备 → VR接口 → 校准模块 → 遥操作控制器 → JAKA桥接 → 双臂     │
│   90Hz    滤波     坐标变换    差分IK+平滑    125Hz伺服  机器人   │
│                                                                 │
│         ↓                       ↓                               │
│      MoveIt2虚拟臂          安全检查器                           │
│      可视化+IK              碰撞+限位                            │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                        末端执行器                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  左夹爪 ←─┬─→ 夹爪控制节点 ←─→ 夹爪GUI                          │
│  右夹爪 ←─┘   Modbus RTU      实时监控                          │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                        头部控制                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Pan/Tilt舵机 ←─→ 头部控制节点 ←─→ VR头显姿态                   │
│                   HTD-85H串口                                   │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│                        移动底盘                                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Standard底盘 ←─→ 底盘控制节点 ←─→ 导航系统/底盘GUI             │
│                  Modbus TCP     路径规划                        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## ROS2 功能包详细说明

### 1. qyh_dual_arms_description

**功能**: JAKA Zu7双机械臂的URDF模型描述包

**类型**: 描述包（ament_cmake）

**主要功能**:
- 双臂机器人URDF/Xacro模型定义
- 关节和连杆的物理参数
- 碰撞检测几何体
- 视觉模型（Mesh文件）
- TF树发布配置

**关键文件**:
```
urdf/
├── qyh_dual_arms.urdf.xacro    # 主URDF文件
├── jaka_zu7_macro.xacro        # 单臂宏定义
├── materials.xacro             # 材质定义
└── common.xacro                # 通用参数

meshes/
├── visual/                     # 可视化Mesh
└── collision/                  # 碰撞检测Mesh

launch/
└── display.launch.py           # RViz可视化
```

**对外接口**:
- **Launch文件**: `display.launch.py` - 启动RViz显示机器人模型
- **URDF参数**: 通过`robot_description`参数提供给其他节点

**使用示例**:
```bash
ros2 launch qyh_dual_arms_description display.launch.py
```

---

### 2. qyh_dual_arms_moveit_config

**功能**: MoveIt2运动规划配置包

**类型**: MoveIt配置包（ament_cmake）

**主要功能**:
- 运动规划配置（OMPL规划器）
- 运动学求解器配置（KDL/TracIK）
- 碰撞检测配置
- 虚拟臂关节控制
- Planning场景管理

**关键配置**:
```
config/
├── kinematics.yaml             # IK求解器配置
├── joint_limits.yaml           # 关节限位
├── moveit_controllers.yaml     # 控制器配置
├── ompl_planning.yaml          # 规划参数
├── pilz_cartesian_limits.yaml  # 笛卡尔限制
└── sensors_3d.yaml             # 传感器配置

launch/
├── move_group.launch.py        # 启动MoveIt核心
├── moveit_rviz.launch.py       # RViz可视化
└── demo.launch.py              # 演示配置
```

**对外接口**:

**话题**:
- `/virtual/joint_states` (sensor_msgs/JointState) - 虚拟臂关节状态
- `/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory) - 轨迹命令

**服务**:
- `/compute_ik` (moveit_msgs/GetPositionIK) - IK求解
- `/compute_fk` (moveit_msgs/GetPositionFK) - FK求解
- `/plan_kinematic_path` (moveit_msgs/GetMotionPlan) - 路径规划

**Action**:
- `/move_action` (moveit_msgs/MoveGroup) - 运动执行

**使用示例**:
```bash
# 启动MoveIt2虚拟臂
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py

# 启动带RViz的完整系统
ros2 launch qyh_dual_arms_moveit_config demo.launch.py
```

---

### 3. qyh_gripper_msgs

**功能**: 夹爪控制消息和服务定义

**类型**: 消息定义包（ament_cmake）

**定义的消息**:

#### GripperState.msg
```
uint8 gripper_id              # 夹爪ID
uint8 current_position        # 当前位置 (0-255)
uint8 target_position         # 目标位置 (0-255)
uint8 current_speed           # 当前速度 (0-255)
uint8 current_force           # 当前力/电流 (0-255)
uint8 object_status           # 物体检测状态
                              # 0=运动中, 1=内撑检测, 2=外夹检测(抓到), 3=到位(未抓到)
uint8 fault_code              # 故障码
                              # 0x00=正常, 0x01=未激活, 0x08=过流, 0x10=电压异常, 0x40=过温
bool is_activated             # 是否已激活
bool is_moving                # 是否在运动
float32 temperature           # 温度（如果支持）
```

**定义的服务**:

#### ActivateGripper.srv
```
# 请求
---
# 响应
bool success
string message
```

#### MoveGripper.srv
```
# 请求
uint8 position    # 目标位置 (0=全开, 255=全闭)
uint8 speed       # 速度 (0=最慢, 255=最快)
uint8 force       # 力度 (0=最小, 255=最大, 建议柔性抓取30-80)
---
# 响应
bool success
string message
```

#### GetGripperState.srv
```
# 请求
---
# 响应
GripperState state
```

**对外接口**:
- 消息和服务定义，由`qyh_gripper_control`节点实现

---

### 4. qyh_gripper_control

**功能**: JODELL EPG夹爪Modbus RTU控制节点

**类型**: C++节点包（ament_cmake）

**主要功能**:
- Modbus RTU通信（基于libmodbus）
- 双手夹爪独立控制（左/右命名空间）
- 实时状态发布（20Hz）
- 激活/去激活控制
- 位置/速度/力度精确控制
- 力反馈读取（具身智能关键数据）
- 物体检测状态
- 故障诊断和报警

**配置文件**:
```yaml
# config/gripper_left.yaml
gripper_control_node:
  ros__parameters:
    device_port: "/dev/ttyUSB0"     # 串口设备
    baudrate: 115200                # 波特率
    slave_id: 1                     # Modbus从站ID
    publish_rate: 20.0              # 状态发布频率
    timeout_ms: 100                 # 通信超时
    max_retries: 3                  # 最大重试次数
```

**对外接口**:

**话题（发布）**:
- `/left/gripper_state` (qyh_gripper_msgs/GripperState) - 左夹爪状态，20Hz
- `/right/gripper_state` (qyh_gripper_msgs/GripperState) - 右夹爪状态，20Hz

**服务**:
- `/left/activate_gripper` (qyh_gripper_msgs/ActivateGripper) - 激活左夹爪
- `/left/move_gripper` (qyh_gripper_msgs/MoveGripper) - 控制左夹爪
- `/left/get_gripper_state` (qyh_gripper_msgs/GetGripperState) - 查询左夹爪状态
- `/right/activate_gripper` (qyh_gripper_msgs/ActivateGripper) - 激活右夹爪
- `/right/move_gripper` (qyh_gripper_msgs/MoveGripper) - 控制右夹爪
- `/right/get_gripper_state` (qyh_gripper_msgs/GetGripperState) - 查询右夹爪状态

**Launch文件**:
- `gripper_control.launch.py` - 启动单个夹爪（参数：side=left/right）
- `dual_gripper.launch.py` - 同时启动左右夹爪

**使用示例**:
```bash
# 启动双手夹爪
ros2 launch qyh_gripper_control dual_gripper.launch.py

# 激活左手夹爪
ros2 service call /left/activate_gripper qyh_gripper_msgs/srv/ActivateGripper

# 控制左手夹爪闭合（位置255，速度200，力度150）
ros2 service call /left/move_gripper qyh_gripper_msgs/srv/MoveGripper \
  "{position: 255, speed: 200, force: 150}"

# 查看实时状态
ros2 topic echo /left/gripper_state
```

---

### 5. qyh_gripper_gui

**功能**: PyQt5双手夹爪监控和控制GUI

**类型**: Python节点包（ament_python）

**主要功能**:
- 双手夹爪实时状态监控
- 位置和力反馈可视化（进度条）
- 精确控制滑块（位置、速度、力度）
- 快捷操作按钮
  - 全开（位置0）
  - 全闭（位置255）
  - 半开（位置128）
  - 轻抓（柔性抓取，力度50）
- 物体检测状态显示
- 故障信息实时提示
- 同步控制（双手同步移动）

**对外接口**:

**订阅话题**:
- `/left/gripper_state` (qyh_gripper_msgs/GripperState) - 左夹爪状态
- `/right/gripper_state` (qyh_gripper_msgs/GripperState) - 右夹爪状态

**调用服务**:
- `/left/activate_gripper`, `/right/activate_gripper` - 激活
- `/left/move_gripper`, `/right/move_gripper` - 移动控制

**使用示例**:
```bash
ros2 run qyh_gripper_gui gripper_gui
```

---

### 6. qyh_head_control

**功能**: HTD-85H舵机头部控制节点

**类型**: Python节点包（ament_cmake + Python）

**主要功能**:
- 串口通信控制HTD-85H舵机
- Pan（水平旋转）和Tilt（垂直旋转）控制
- VR头显姿态跟随
- 角度位置控制
- 速度控制
- 位置反馈

**关键参数**:
```yaml
head_control_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB2"     # 串口
    baudrate: 115200                # 波特率
    pan_servo_id: 1                 # Pan舵机ID
    tilt_servo_id: 2                # Tilt舵机ID
    update_rate: 50.0               # 控制频率
    pan_range: [-90.0, 90.0]        # Pan角度范围
    tilt_range: [-45.0, 45.0]       # Tilt角度范围
```

**对外接口**:

**话题（订阅）**:
- `/vr/head_pose` (geometry_msgs/PoseStamped) - VR头显姿态
- `/head/pan_tilt_command` (std_msgs/Float64MultiArray) - [pan, tilt]角度命令

**话题（发布）**:
- `/head/joint_states` (sensor_msgs/JointState) - 头部关节状态
- `/head/status` (std_msgs/String) - 头部控制状态

**使用示例**:
```bash
# 启动头部控制节点
ros2 run qyh_head_control head_control_node

# 发送角度命令（Pan=30度, Tilt=15度）
ros2 topic pub /head/pan_tilt_command std_msgs/Float64MultiArray \
  "data: [30.0, 15.0]"
```

---

### 7. qyh_jaka_control_msgs

**功能**: JAKA双臂控制消息和服务定义

**类型**: 消息定义包（ament_cmake）

**定义的消息**:

#### JakaDualJointServo.msg
```
float64[14] positions         # 双臂关节位置（左7+右7）
bool is_abs                   # True=绝对位置, False=相对增量
float64 velocity              # 运动速度（弧度/秒或度/秒）
float64 acceleration          # 运动加速度
```

#### JakaDualCartesianServo.msg
```
geometry_msgs/Pose left_pose  # 左臂目标位姿
geometry_msgs/Pose right_pose # 右臂目标位姿
bool is_abs                   # True=绝对位姿, False=相对增量
```

#### JakaServoStatus.msg
```
string mode                   # 控制模式："joint", "cartesian", "idle"
bool is_abs                   # 是否绝对模式
int32 cycle_time_ns           # 控制周期（纳秒）
float64 publish_rate_hz       # 实际发布频率
float64 latency_ms            # 控制延迟（毫秒）
float64 packet_loss_rate      # 丢包率估计
int32 error_code              # 错误码（0=正常）
string error_message          # 错误描述
```

#### RobotState.msg
```
sensor_msgs/JointState joint_state      # 关节状态
geometry_msgs/Pose left_tcp_pose        # 左臂TCP位姿
geometry_msgs/Pose right_tcp_pose       # 右臂TCP位姿
bool is_servo_enabled                   # 伺服是否启用
bool is_emergency_stopped               # 是否急停
int32[] error_codes                     # 错误码列表
```

#### VRPose.msg
```
geometry_msgs/Pose pose       # VR控制器位姿
string controller_id          # "left" 或 "right"
float64 battery_level         # 电池电量 (0.0-1.0)
bool button_trigger           # 扳机键状态
bool button_grip              # 握把键状态
```

#### VRFollowStatus.msg
```
string left_arm_status        # 左臂跟随状态
string right_arm_status       # 右臂跟随状态
float64 left_pose_error       # 左臂跟踪误差（米）
float64 right_pose_error      # 右臂跟踪误差（米）
int32 recorded_frames         # 已录制帧数
float64 recording_duration    # 录制时长（秒）
```

**定义的服务**:

#### 伺服控制类
- **StartServo.srv**: 启动实时伺服模式
- **StopServo.srv**: 停止伺服模式
- **SetFilter.srv**: 配置运动滤波器（LPF/NLF）

#### 运动控制类
- **MoveJ.srv**: 关节空间运动
  ```
  float64[7] left_joints
  float64[7] right_joints
  float64 speed
  float64 acceleration
  ---
  bool success
  string message
  ```

- **MoveL.srv**: 笛卡尔空间直线运动
  ```
  geometry_msgs/Pose left_target
  geometry_msgs/Pose right_target
  float64 speed
  ---
  bool success
  string message
  ```

#### VR遥操作类
- **EnableVRFollow.srv**: 开启/关闭VR跟随
  ```
  bool enable
  ---
  bool success
  string message
  ```

- **CalibrateVR.srv**: VR坐标系校准
  ```
  ---
  bool success
  geometry_msgs/Transform calibration_transform
  string message
  ```

#### 传感器配置类
- **SetFTConfig.srv**: 力矩传感器配置
- **ZeroFT.srv**: 力矩传感器清零
- **SetToolOffset.srv**: 设置工具偏移
- **SetCollisionLevel.srv**: 设置碰撞等级

#### 状态查询类
- **GetRobotState.srv**: 获取机器人完整状态
  ```
  ---
  RobotState state
  ```

**对外接口**:
- 消息和服务定义，由`qyh_jaka_control`节点实现

---

### 8. qyh_jaka_control

**功能**: JAKA双臂核心控制节点

**类型**: C++节点包（ament_cmake）

**主要节点**:

#### 1. jaka_control_node
- 机器人状态管理
- 关节控制（MoveJ）
- 笛卡尔控制（MoveL）
- 传感器数据读取
- 安全监控

#### 2. jaka_bridge_node
- 接收高层运动命令
- 轨迹缓冲和插值
- 125Hz EtherCAT伺服同步
- 性能监控和统计

#### 3. smooth_servo_bridge
- 订阅关节命令
- 三阶段轨迹平滑
- 速度/加速度/jerk限制
- 实时伺服发送

**对外接口**:

**话题（订阅）**:
- `/left_arm/joint_command` (sensor_msgs/JointState) - 左臂关节命令
- `/right_arm/joint_command` (sensor_msgs/JointState) - 右臂关节命令
- `/jaka/dual_joint_servo` (qyh_jaka_control_msgs/JakaDualJointServo) - 双臂伺服
- `/jaka/dual_cartesian_servo` (qyh_jaka_control_msgs/JakaDualCartesianServo) - 笛卡尔伺服

**话题（发布）**:
- `/robot_state` (qyh_jaka_control_msgs/RobotState) - 机器人状态，125Hz
- `/joint_states` (sensor_msgs/JointState) - 关节状态
- `/jaka/servo_status` (qyh_jaka_control_msgs/JakaServoStatus) - 伺服状态

**服务**:
- `/jaka/start_servo` (qyh_jaka_control_msgs/StartServo) - 启动伺服
- `/jaka/stop_servo` (qyh_jaka_control_msgs/StopServo) - 停止伺服
- `/jaka/move_j` (qyh_jaka_control_msgs/MoveJ) - 关节运动
- `/jaka/move_l` (qyh_jaka_control_msgs/MoveL) - 直线运动
- `/jaka/get_robot_state` (qyh_jaka_control_msgs/GetRobotState) - 查询状态
- `/jaka/set_filter` (qyh_jaka_control_msgs/SetFilter) - 设置滤波器
- `/jaka/zero_ft` (qyh_jaka_control_msgs/ZeroFT) - 力矩传感器清零

**配置文件**:
```yaml
jaka_control_node:
  ros__parameters:
    robot_ip: "192.168.2.200"       # 机器人IP
    port: 10000                      # 控制端口
    servo_rate: 125.0                # 伺服频率（Hz）
    buffer_size: 10                  # 轨迹缓冲大小
    max_joint_velocity: 2.0          # 最大关节速度（rad/s）
    max_joint_acceleration: 5.0      # 最大加速度（rad/s²）
    max_cartesian_velocity: 0.5      # 最大笛卡尔速度（m/s）
```

**使用示例**:
```bash
# 启动JAKA控制节点
ros2 run qyh_jaka_control jaka_control_node \
  --ros-args -p robot_ip:=192.168.2.200

# 启动伺服模式
ros2 service call /jaka/start_servo qyh_jaka_control_msgs/srv/StartServo

# 查看机器人状态
ros2 topic echo /robot_state
```

---

### 9. qyh_jaka_control_gui

**功能**: JAKA控制图形界面

**类型**: Python节点包（ament_python）

**主要功能**:
- 发布ABS/REL伺服命令
- 关节位置实时显示
- 手动Jog控制
- 预设位置快速跳转
- 伺服状态监控

**对外接口**:

**订阅话题**:
- `/robot_state` (qyh_jaka_control_msgs/RobotState) - 机器人状态
- `/jaka/servo_status` (qyh_jaka_control_msgs/JakaServoStatus) - 伺服状态

**发布话题**:
- `/jaka/dual_joint_servo` (qyh_jaka_control_msgs/JakaDualJointServo) - 关节伺服命令

**使用示例**:
```bash
ros2 run qyh_jaka_control_gui jaka_gui
```

---

### 10. qyh_standard_robot_msgs

**功能**: Standard移动底盘消息和服务定义

**类型**: 消息定义包（ament_cmake）

**定义的消息**:

#### StandardRobotStatus.msg
```
float64 battery_level         # 电池电量 (0.0-1.0)
float64 speed                 # 当前速度 (m/s)
string robot_mode             # 机器人模式
bool is_emergency_stopped     # 是否急停
bool is_charging              # 是否充电中
int32 error_code              # 错误码
string error_message          # 错误描述
```

#### NavigationStatus.msg
```
string navigation_status      # 导航状态
geometry_msgs/Pose current_pose    # 当前位姿
geometry_msgs/Pose target_pose     # 目标位姿
float64 distance_to_target    # 距离目标距离（米）
float64 estimated_time        # 预估到达时间（秒）
```

#### NavigationPose.msg
```
string map_id                 # 地图ID
geometry_msgs/Pose pose       # 位姿
string site_name              # 站点名称（可选）
```

#### ManualMotionCommand.msg
```
float64 linear_velocity       # 线速度 (m/s)
float64 angular_velocity      # 角速度 (rad/s)
```

**定义的服务**:

#### 控制类
- **ControlEmergencyStop.srv**: 急停
- **ControlReleaseEmergencyStop.srv**: 解除急停
- **ControlStartCharging.srv**: 开始充电
- **ControlStopCharging.srv**: 停止充电
- **ControlEnterLowPowerMode.srv**: 进入低功耗模式
- **ControlExitLowPowerMode.srv**: 退出低功耗模式
- **ControlStartManualControl.srv**: 开始手动控制
- **ControlStopManualControl.srv**: 停止手动控制
- **ControlPauseMove.srv**: 暂停运动
- **ControlResumeMove.srv**: 恢复运动
- **ControlStopMove.srv**: 停止运动

#### 导航类
- **GoNavigateToCoordinate.srv**: 导航到坐标
  ```
  NavigationPose target
  ---
  bool success
  string message
  ```

- **GoNavigateToSiteWithTask.srv**: 导航到站点并执行任务
  ```
  string site_name
  string task_id
  ---
  bool success
  string message
  ```

- **GoSetCurrentMap.srv**: 设置当前地图
- **GoForceLocalize.srv**: 强制定位

#### 设置类
- **GoSetSpeedType.srv**: 设置速度类型
- **GoSetSpeakerVolume.srv**: 设置扬声器音量
- **GoSetObstacleStrategy.srv**: 设置避障策略
- **GoSetManualControl.srv**: 设置手动控制参数

**对外接口**:
- 消息和服务定义，由`qyh_standard_robot`节点实现

---

### 11. qyh_standard_robot

**功能**: Standard移动底盘Modbus TCP控制节点

**类型**: C++节点包（ament_cmake）

**主要功能**:
- Modbus TCP通信
- 底盘速度控制
- 导航指令发送
- 状态实时监控
- 急停和安全控制
- 电池电量监控
- 充电控制

**对外接口**:

**话题（订阅）**:
- `/cmd_vel` (geometry_msgs/Twist) - 速度控制命令
- `/chassis/manual_command` (qyh_standard_robot_msgs/ManualMotionCommand) - 手动控制

**话题（发布）**:
- `/chassis/status` (qyh_standard_robot_msgs/StandardRobotStatus) - 底盘状态，10Hz
- `/chassis/navigation_status` (qyh_standard_robot_msgs/NavigationStatus) - 导航状态
- `/odom` (nav_msgs/Odometry) - 里程计数据

**服务**: 实现所有`qyh_standard_robot_msgs`中定义的服务

**配置文件**:
```yaml
standard_robot_node:
  ros__parameters:
    robot_ip: "192.168.1.100"       # 底盘IP
    port: 502                        # Modbus TCP端口
    update_rate: 10.0                # 状态更新频率
    max_linear_velocity: 1.5         # 最大线速度 (m/s)
    max_angular_velocity: 1.0        # 最大角速度 (rad/s)
```

**使用示例**:
```bash
# 启动底盘控制节点
ros2 run qyh_standard_robot standard_robot_node \
  --ros-args -p robot_ip:=192.168.1.100

# 手动控制（前进0.5m/s）
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 查看底盘状态
ros2 topic echo /chassis/status
```

---

### 12. qyh_standard_robot_gui

**功能**: Standard底盘控制图形界面

**类型**: Python节点包（ament_python）

**主要功能**:
- 底盘状态实时监控
- 手动速度控制
- 导航目标设置
- 电池电量显示
- 急停按钮
- 充电控制

**对外接口**:

**订阅话题**:
- `/chassis/status` (qyh_standard_robot_msgs/StandardRobotStatus) - 底盘状态

**发布话题**:
- `/cmd_vel` (geometry_msgs/Twist) - 速度命令

**调用服务**: 调用底盘控制相关服务

**使用示例**:
```bash
ros2 run qyh_standard_robot_gui chassis_gui
```

---

### 13. qyh_vr_calibration_msgs

**功能**: VR校准消息和服务定义

**类型**: 消息定义包（ament_cmake）

**定义的消息**:

#### CalibrationPose.msg
```
geometry_msgs/Pose vr_pose    # VR控制器位姿
geometry_msgs/Pose robot_pose # 机器人TCP位姿
string arm_side               # "left" 或 "right"
time stamp                    # 时间戳
```

#### CalibrationProfile.msg
```
string user_id                # 用户ID
string profile_name           # 配置名称
geometry_msgs/Transform left_transform   # 左臂校准变换
geometry_msgs/Transform right_transform  # 右臂校准变换
float64 scale_factor          # 缩放因子
time created_at               # 创建时间
time updated_at               # 更新时间
```

**定义的服务**:

#### CalibrateUser.srv
```
string user_id                # 用户ID
CalibrationPose[] poses       # 校准位姿点（至少3个）
---
bool success
CalibrationProfile profile
string message
```

#### LoadUserCalibration.srv
```
string user_id
---
bool success
CalibrationProfile profile
string message
```

#### ListUsers.srv
```
---
string[] user_ids
```

#### DeleteUser.srv
```
string user_id
---
bool success
string message
```

#### SetRobotCalibration.srv
```
geometry_msgs/Transform left_transform
geometry_msgs/Transform right_transform
float64 scale_factor
---
bool success
string message
```

#### GetRobotCalibration.srv
```
---
geometry_msgs/Transform left_transform
geometry_msgs/Transform right_transform
float64 scale_factor
```

**对外接口**:
- 消息和服务定义，由`qyh_vr_calibration`节点实现

---

### 14. qyh_vr_calibration

**功能**: VR坐标系校准和用户配置管理

**类型**: Python节点包（ament_python）

**主要功能**:
- VR坐标系到机器人坐标系的转换
- 多用户校准配置管理
- 持久化存储（JSON/YAML）
- 实时坐标变换发布
- 自动加载上次使用的配置
- 缩放因子管理

**对外接口**:

**话题（订阅）**:
- `/vr/raw/left_pose` (geometry_msgs/PoseStamped) - 原始左VR位姿
- `/vr/raw/right_pose` (geometry_msgs/PoseStamped) - 原始右VR位姿

**话题（发布）**:
- `/vr/calibrated/left_pose` (geometry_msgs/PoseStamped) - 校准后左VR位姿
- `/vr/calibrated/right_pose` (geometry_msgs/PoseStamped) - 校准后右VR位姿

**TF发布**:
- `world` → `vr_origin` - VR原点变换
- `vr_origin` → `left_controller` - 左控制器
- `vr_origin` → `right_controller` - 右控制器

**服务**: 实现所有`qyh_vr_calibration_msgs`中定义的服务

**配置文件**:
```yaml
vr_calibration_node:
  ros__parameters:
    data_dir: "~/.ros/vr_calibration"  # 数据存储目录
    auto_load_last_user: true           # 自动加载上次用户
    default_scale_factor: 1.0           # 默认缩放因子
    publish_rate: 90.0                  # 发布频率（Hz）
```

**使用示例**:
```bash
# 启动VR校准节点
ros2 run qyh_vr_calibration vr_calibration_node

# 列出所有用户
ros2 service call /vr/list_users qyh_vr_calibration_msgs/srv/ListUsers

# 加载用户校准
ros2 service call /vr/load_user_calibration \
  qyh_vr_calibration_msgs/srv/LoadUserCalibration "{user_id: 'user001'}"

# 查看校准后的位姿
ros2 topic echo /vr/calibrated/left_pose
```

---

### 15. qyh_teleoperation_msgs

**功能**: 遥操作消息和服务定义

**类型**: 消息定义包（ament_cmake）

**定义的消息**:

#### TeleopStatus.msg
```
bool is_active                # 遥操作是否激活
string control_mode           # 控制模式："position", "velocity", "disabled"
float64 left_tracking_error   # 左臂跟踪误差（米）
float64 right_tracking_error  # 右臂跟踪误差（米）
int32 safety_violations       # 安全违规计数
string[] active_constraints   # 激活的约束列表
```

#### VirtualArmState.msg
```
sensor_msgs/JointState joint_state     # 虚拟臂关节状态
geometry_msgs/Pose left_tcp_pose       # 左臂TCP位姿
geometry_msgs/Pose right_tcp_pose      # 右臂TCP位姿
bool ik_solution_valid                 # IK解是否有效
```

#### SafetyStatus.msg
```
bool collision_detected       # 是否检测到碰撞
bool joint_limit_reached      # 是否达到关节限位
bool singularity_detected     # 是否接近奇异点
bool workspace_limit_reached  # 是否超出工作空间
float64 min_distance_to_obstacle  # 到障碍物最小距离
string[] warnings             # 警告信息列表
```

#### PerformanceMetrics.msg
```
float64 control_loop_hz       # 控制回路频率
float64 ik_solve_time_ms      # IK求解时间（毫秒）
float64 trajectory_smooth_time_ms  # 轨迹平滑时间
float64 total_latency_ms      # 总延迟
int32 dropped_frames          # 丢帧数
```

**定义的服务**:

#### StartTeleoperation.srv
```
string control_mode           # "position" 或 "velocity"
---
bool success
string message
```

#### StopTeleoperation.srv
```
---
bool success
string message
```

#### SetControlMode.srv
```
string mode                   # "position", "velocity", "disabled"
---
bool success
string message
```

**对外接口**:
- 消息和服务定义，由`qyh_teleoperation_controller`节点实现

---

### 16. qyh_teleoperation_controller

**功能**: 核心遥操作控制器

**类型**: C++节点包（ament_cmake）

**主要功能**:
- 差分逆运动学（Damped Least Squares）
- 三阶段轨迹平滑（速度/加速度/jerk限制）
- 实时安全检查
  - 关节限位检查
  - 碰撞检测
  - 奇异点检测
  - 工作空间限制
- 虚拟臂跟随控制
- 性能监控和优化
- 自适应控制增益

**核心模块**:

#### 1. differential_ik_controller.cpp
- 实现差分IK算法
- Jacobian矩阵计算
- 阻尼最小二乘法求解
- 奇异值分解（SVD）

#### 2. trajectory_smoother.cpp
- 速度平滑（一阶导数）
- 加速度平滑（二阶导数）
- Jerk平滑（三阶导数）
- 实时滤波

#### 3. safety_checker.cpp
- 关节限位检查
- 自碰撞检测
- 环境碰撞检测
- 奇异性分析

#### 4. virtual_arm_follower.cpp
- 订阅VR位姿
- 调用MoveIt IK
- 生成关节命令
- 状态可视化

**对外接口**:

**话题（订阅）**:
- `/vr/calibrated/left_pose` (geometry_msgs/PoseStamped) - 左VR位姿
- `/vr/calibrated/right_pose` (geometry_msgs/PoseStamped) - 右VR位姿
- `/virtual/joint_states` (sensor_msgs/JointState) - 虚拟臂状态

**话题（发布）**:
- `/left_arm/joint_command` (sensor_msgs/JointState) - 左臂关节命令
- `/right_arm/joint_command` (sensor_msgs/JointState) - 右臂关节命令
- `/teleop/status` (qyh_teleoperation_msgs/TeleopStatus) - 遥操作状态
- `/teleop/safety_status` (qyh_teleoperation_msgs/SafetyStatus) - 安全状态
- `/teleop/performance` (qyh_teleoperation_msgs/PerformanceMetrics) - 性能指标

**服务**:
- `/teleop/start` (qyh_teleoperation_msgs/StartTeleoperation) - 启动遥操作
- `/teleop/stop` (qyh_teleoperation_msgs/StopTeleoperation) - 停止遥操作
- `/teleop/set_control_mode` (qyh_teleoperation_msgs/SetControlMode) - 设置控制模式

**配置文件**:
```yaml
teleoperation_node:
  ros__parameters:
    control_rate: 125.0                  # 控制频率（Hz）
    ik_solver: "damped_least_squares"    # IK求解器类型
    damping_factor: 0.01                 # 阻尼因子
    
    # 轨迹平滑参数
    max_joint_velocity: 2.0              # rad/s
    max_joint_acceleration: 5.0          # rad/s²
    max_joint_jerk: 50.0                 # rad/s³
    
    # 安全参数
    joint_limit_margin: 0.1              # 关节限位裕度（rad）
    min_obstacle_distance: 0.05          # 最小障碍物距离（m）
    singularity_threshold: 0.01          # 奇异性阈值
    
    # 性能参数
    enable_performance_logging: true
    log_interval: 10.0                   # 日志间隔（秒）
```

**使用示例**:
```bash
# 启动遥操作控制器
ros2 run qyh_teleoperation_controller teleoperation_node

# 启动遥操作
ros2 service call /teleop/start qyh_teleoperation_msgs/srv/StartTeleoperation \
  "{control_mode: 'position'}"

# 查看性能指标
ros2 topic echo /teleop/performance

# 查看安全状态
ros2 topic echo /teleop/safety_status
```

---

### 17. qyh_teleoperation_bringup

**功能**: 完整系统启动配置

**类型**: Launch包（ament_cmake）

**主要功能**:
- 一键启动完整遥操作系统
- 参数配置管理
- 节点依赖管理
- 可选组件控制

**Launch文件**:

#### full_system.launch.py
启动完整的VR遥操作系统，包括：
1. JAKA双臂控制节点
2. VR接口和校准
3. MoveIt2虚拟臂
4. 遥操作控制器
5. 夹爪控制（可选）
6. 头部控制（可选）
7. 底盘控制（可选）
8. RViz可视化（可选）

**参数**:
```python
# 必需参数
robot_ip              # JAKA机器人IP地址

# 可选参数
use_simulator=false   # 使用VR模拟器（无需真实VR设备）
rviz=true            # 启动RViz可视化
enable_gripper=true  # 启动夹爪控制
enable_head=true     # 启动头部控制
enable_chassis=false # 启动底盘控制
log_level=info       # 日志级别
```

**使用示例**:
```bash
# 启动完整系统（真实VR）
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200

# 使用VR模拟器测试
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    use_simulator:=true

# 不启用夹爪和头部
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    enable_gripper:=false \
    enable_head:=false

# 启用底盘控制
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    enable_chassis:=true \
    chassis_ip:=192.168.1.100
```

**启动顺序**:
1. JAKA控制节点（等待机器人连接）
2. VR校准和接口（等待VR设备）
3. MoveIt2（等待URDF加载）
4. 遥操作控制器（等待所有依赖）
5. 外设控制节点（并行启动）
6. RViz（可选，最后启动）

---

## 快速开始

### 系统要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS2 Humble
- **Python版本**: Python 3.10+
- **依赖库**:
  - MoveIt2
  - libmodbus
  - PyQt5
  - NumPy, SciPy

### 编译项目

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译所有包
colcon build

# 或选择性编译
colcon build --packages-select qyh_gripper_control qyh_gripper_gui
colcon build --packages-select qyh_jaka_control qyh_teleoperation_controller

# 加载环境
source install/setup.bash
```

### 启动完整系统

```bash
# 方式1: 使用launch文件（推荐）
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200

# 方式2: 手动启动各模块（调试用）
# 终端1: JAKA控制
ros2 run qyh_jaka_control jaka_control_node \
    --ros-args -p robot_ip:=192.168.2.200

# 终端2: VR接口
ros2 run qyh_vr_calibration vr_interface_node

# 终端3: MoveIt2
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py

# 终端4: 遥操作控制器
ros2 run qyh_teleoperation_controller teleoperation_node

# 终端5: 夹爪控制
ros2 launch qyh_gripper_control dual_gripper.launch.py

# 终端6: GUI
ros2 run qyh_gripper_gui gripper_gui
```

### 测试单个模块

```bash
# 测试夹爪
ros2 launch qyh_gripper_control dual_gripper.launch.py
ros2 run qyh_gripper_gui gripper_gui

# 测试JAKA控制
ros2 run qyh_jaka_control jaka_control_node \
    --ros-args -p robot_ip:=192.168.2.200
ros2 service call /jaka/get_robot_state qyh_jaka_control_msgs/srv/GetRobotState

# 测试MoveIt2
ros2 launch qyh_dual_arms_moveit_config demo.launch.py

# 测试VR接口（模拟器）
ros2 launch qyh_vr_calibration test_vr_interface.launch.py \
    motion_type:=circle amplitude:=0.05
```

---

## 数据流和话题关系图

```
┌─────────────┐
│  VR Device  │
└──────┬──────┘
       │ 90Hz
       ↓
┌─────────────────┐
│  VR Interface   │ /vr/raw/{left|right}_pose
└────────┬────────┘
         │
         ↓
┌──────────────────┐
│ VR Calibration   │ /vr/calibrated/{left|right}_pose
└────────┬─────────┘
         │
         ↓
┌─────────────────────────┐
│ Teleoperation Controller │
│  - Differential IK      │ /left_arm/joint_command
│  - Trajectory Smooth    │ /right_arm/joint_command
│  - Safety Check         │
└──────────┬──────────────┘
           │
           ↓
┌─────────────────┐
│  JAKA Bridge    │
│  - Buffer       │ EtherCAT 125Hz
│  - Interpolate  │
└────────┬────────┘
         │
         ↓
┌─────────────────┐
│  JAKA Zu7 Arms  │ /robot_state
│  - Left Arm     │ /joint_states
│  - Right Arm    │
└─────────────────┘

         ┌─────────────────┐
         │  MoveIt2        │
         │  Virtual Arms   │ /virtual/joint_states
         │  - IK Solver    │ (Visualization)
         │  - Collision    │
         └─────────────────┘
```

---

## 性能指标

| 模块 | 频率 | 延迟 | 说明 |
|------|------|------|------|
| VR接口 | 90Hz | ~10ms | VR设备原生频率 |
| VR校准 | 90Hz | ~2ms | 坐标变换 |
| 遥操作控制器 | 125Hz | ~5ms | IK求解+平滑 |
| JAKA桥接 | 125Hz | ~3ms | EtherCAT同步 |
| 机器人伺服 | 125Hz | ~5ms | 内部控制回路 |
| **端到端延迟** | - | **~25ms** | VR到机器人 |
| 夹爪控制 | 20Hz | ~50ms | Modbus RTU |
| 底盘控制 | 10Hz | ~100ms | Modbus TCP |

---

## 常见问题

### 1. 编译错误

**问题**: 找不到MoveIt2相关头文件
```bash
# 解决方案
sudo apt install ros-humble-moveit
rosdep install --from-paths src --ignore-src -r -y
```

**问题**: libmodbus链接错误
```bash
# 解决方案
cd src/qyh_gripper_control/thirdparty
# 从qyh_standard_robot复制或安装libmodbus
sudo apt install libmodbus-dev
```

### 2. 运行时错误

**问题**: 无法连接JAKA机器人
```bash
# 检查网络连接
ping 192.168.2.200

# 检查防火墙
sudo ufw allow 10000/tcp

# 查看日志
ros2 run qyh_jaka_control jaka_control_node --ros-args --log-level debug
```

**问题**: 夹爪无响应
```bash
# 检查串口权限
sudo chmod 666 /dev/ttyUSB0

# 检查串口设备
ls -l /dev/ttyUSB*

# 测试Modbus通信
ros2 service call /left/activate_gripper qyh_gripper_msgs/srv/ActivateGripper
```

**问题**: VR设备未检测到
```bash
# 使用模拟器测试
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    use_simulator:=true
```

### 3. 性能问题

**问题**: 遥操作延迟高
```bash
# 检查控制回路频率
ros2 topic hz /teleop/performance

# 优化建议
# 1. 降低RViz可视化质量
# 2. 关闭不必要的日志
# 3. 使用RT内核
```

---

## 开发指南

### 添加新的末端执行器

1. 在`qyh_dual_arms_description`中添加URDF
2. 在`qyh_dual_arms_moveit_config`中更新MoveIt配置
3. 创建新的控制包和消息包
4. 在`full_system.launch.py`中添加启动选项

### 扩展VR控制功能

1. 在`qyh_vr_calibration`中添加新的坐标变换
2. 在`qyh_teleoperation_controller`中实现新的控制模式
3. 更新安全检查器

### 添加新的传感器

1. 定义传感器消息（创建新的msgs包或使用标准消息）
2. 实现传感器驱动节点
3. 在遥操作控制器中集成传感器数据

---

## 许可证

本项目采用以下许可证：
- 核心控制系统：Apache-2.0
- GUI组件：MIT
- MoveIt配置：BSD

---

## 维护者

- **项目负责人**: qyh <jsqinyinghao@live.com>
- **贡献者**: 查看Git提交历史

---

## 参考资料

### 硬件文档
- JAKA Zu7机械臂：`资料/双机械臂/`
- JODELL EPG夹爪：`资料/夹爪/`
- HTD-85H舵机：`资料/头部电机/`
- Standard底盘：`资料/底盘/`

### 软件文档
- ROS2 Humble: https://docs.ros.org/en/humble/
- MoveIt2: https://moveit.ros.org/
- 项目Wiki: 待添加

### 开发记录
- `DEVELOPMENT_SUMMARY.md` - 开发总结
- `PHASE1_COMPLETION_REPORT.md` - 阶段一报告
- `TELEOPERATION_INTEGRATION_GUIDE.md` - 集成指南
- `VR_TELEOPERATION_ARCHITECTURE.md` - 架构文档

---

## 更新日志

### v1.0.0 (2025-11-27)
- ✅ 完整的VR遥操作系统
- ✅ 双臂JAKA控制（125Hz伺服）
- ✅ 夹爪控制和GUI
- ✅ 头部控制
- ✅ 底盘集成
- ✅ MoveIt2运动规划
- ✅ 安全检查和轨迹平滑
- ✅ 完整文档

---

**文档生成时间**: 2025年11月27日  
**版本**: 1.0.0  
**状态**: 生产就绪
