# qyh_jaka_control

本功能包提供了JAKA双臂机器人系统的核心控制节点。它负责与机器人控制器通信，提供ROS 2控制接口，并集成了VR遥操作和数据录制功能。

## 核心节点: jaka_control_node

### 功能描述
`jaka_control_node` 通过以太网连接到JAKA机器人控制器，主要功能包括：
- **基础控制**：上下电、使能/去使能、错误清除、急停。
- **实时伺服**：支持关节空间和笛卡尔空间的实时控制（默认125Hz）。
- **VR遥操作**：订阅VR手柄位姿，控制机械臂跟随。
- **数据录制**：以CSV格式记录机器人状态和VR数据，用于具身智能训练。

### 参数列表 (Parameters)

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `robot_ip` | string | "192.168.2.200" | JAKA机器人控制器的IP地址。 |
| `cycle_time_ms` | double | 8.0 | 控制周期（毫秒），8ms对应125Hz。 |
| `use_cartesian` | bool | false | 是否使用笛卡尔空间控制（False为关节空间）。 |
| `auto_connect` | bool | true | 启动时是否自动连接机器人。 |
| `auto_power_on` | bool | false | 连接成功后是否自动上电。 |
| `auto_enable` | bool | false | 上电成功后是否自动使能。 |
| `recording_output_dir` | string | "/tmp/jaka_recordings" | 录制数据的保存目录。 |

### 订阅话题 (Subscribed Topics)

- `/jaka/servo/joint_cmd` (`qyh_jaka_control_msgs/msg/JakaDualJointServo`)
    - 双臂关节位置控制指令。
- `/jaka/servo/cartesian_cmd` (`qyh_jaka_control_msgs/msg/JakaDualCartesianServo`)
    - 双臂笛卡尔位姿控制指令。
- `/vr/left_controller` (`qyh_jaka_control_msgs/msg/VRPose`)
    - 左手VR控制器的位姿数据。
- `/vr/right_controller` (`qyh_jaka_control_msgs/msg/VRPose`)
    - 右手VR控制器的位姿数据。

### 发布话题 (Published Topics)

- `/jaka/servo/status` (`qyh_jaka_control_msgs/msg/JakaServoStatus`)
    - 伺服控制回路的实时状态（10Hz）。
- `/jaka/vr/status` (`qyh_jaka_control_msgs/msg/VRFollowStatus`)
    - VR跟随模式的状态（仅在跟随模式下发布）。
- `/jaka/robot_state` (`qyh_jaka_control_msgs/msg/RobotState`)
    - 机器人完整状态（包括关节位置、笛卡尔位姿、错误信息等，10Hz）。
- `/joint_states` (`sensor_msgs/msg/JointState`)
    - 标准ROS关节状态消息（用于RViz可视化，10Hz）。

### 服务接口 (Services)

#### 基础控制
- `/jaka/robot/power_on` (`std_srvs/srv/Trigger`)：机器人上电。
- `/jaka/robot/power_off` (`std_srvs/srv/Trigger`)：机器人下电。
- `/jaka/robot/enable` (`std_srvs/srv/Trigger`)：机器人使能。
- `/jaka/robot/disable` (`std_srvs/srv/Trigger`)：机器人去使能。
- `/jaka/robot/clear_error` (`std_srvs/srv/Trigger`)：清除机器人错误。
- `/jaka/robot/motion_abort` (`std_srvs/srv/Trigger`)：紧急停止运动。

#### 伺服控制
- `/jaka/servo/start` (`qyh_jaka_control_msgs/srv/StartServo`)：进入实时伺服模式。
- `/jaka/servo/stop` (`qyh_jaka_control_msgs/srv/StopServo`)：退出实时伺服模式。
- `/jaka/servo/set_filter` (`qyh_jaka_control_msgs/srv/SetFilter`)：配置运动滤波器（LPF/NLF）。

#### VR与录制
- `/jaka/vr/enable` (`qyh_jaka_control_msgs/srv/EnableVRFollow`)：开启/关闭VR跟随。
- `/jaka/vr/calibrate` (`qyh_jaka_control_msgs/srv/CalibrateVR`)：校准VR坐标系。
- `/jaka/vr/start_recording` (`qyh_jaka_control_msgs/srv/StartRecording`)：开始录制数据。
- `/jaka/vr/stop_recording` (`qyh_jaka_control_msgs/srv/StopRecording`)：停止录制并保存。

#### 点到点运动
- `/jaka/move_j` (`qyh_jaka_control_msgs/srv/MoveJ`)：关节空间点到点运动。
- `/jaka/move_l` (`qyh_jaka_control_msgs/srv/MoveL`)：笛卡尔空间直线运动。

#### 配置与查询
- `/jaka/set_collision_level` (`qyh_jaka_control_msgs/srv/SetCollisionLevel`)：设置碰撞检测等级。
- `/jaka/set_tool_offset` (`qyh_jaka_control_msgs/srv/SetToolOffset`)：设置工具坐标系偏移。
- `/jaka/get_robot_state` (`qyh_jaka_control_msgs/srv/GetRobotState`)：主动查询机器人状态。

## 使用方法

### 启动节点
```bash
# 使用默认参数启动
ros2 run qyh_jaka_control jaka_control_node

# 指定IP启动
ros2 run qyh_jaka_control jaka_control_node --ros-args -p robot_ip:="192.168.1.100"
```

### 典型操作流程

#### 1. 基础运动控制（点到点）
```bash
# 启动节点
ros2 run qyh_jaka_control jaka_control_node

# 上电
ros2 service call /jaka/robot/power_on std_srvs/srv/Trigger

# 使能
ros2 service call /jaka/robot/enable std_srvs/srv/Trigger

# 执行关节运动（移动到指定关节角度）
ros2 service call /jaka/move_j qyh_jaka_control_msgs/srv/MoveJ \
  "{robot_id: 0, joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    move_mode: false, velocity: 1.0, acceleration: 0.5, is_block: true}"

# 执行笛卡尔直线运动
ros2 service call /jaka/move_l qyh_jaka_control_msgs/srv/MoveL \
  "{robot_id: 0, target_pose: {position: {x: 0.5, y: 0.0, z: 0.3}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
    move_mode: false, velocity: 100.0, acceleration: 50.0, is_block: true}"
```

#### 2. 实时伺服控制
```bash
# 启动与使能（同上）

# 开启伺服模式
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo

# 通过话题发送实时控制指令
ros2 topic pub /jaka/servo/joint_cmd qyh_jaka_control_msgs/msg/JakaDualJointServo ...

# 停止伺服模式
ros2 service call /jaka/servo/stop qyh_jaka_control_msgs/srv/StopServo
```

#### 3. VR遥操作与录制
```bash
# 开启伺服模式后

# 校准VR坐标系（首次使用）
ros2 service call /jaka/vr/calibrate qyh_jaka_control_msgs/srv/CalibrateVR

# 启用VR跟随
ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: true}"

# 开始录制
ros2 service call /jaka/vr/start_recording qyh_jaka_control_msgs/srv/StartRecording \
  "{output_path: '/home/user/data'}"

# ... 进行操作 ...

# 停止录制
ros2 service call /jaka/vr/stop_recording qyh_jaka_control_msgs/srv/StopRecording
```

#### 4. 配置机器人参数
```bash
# 设置碰撞检测等级（0-5，0最灵敏）
ros2 service call /jaka/set_collision_level qyh_jaka_control_msgs/srv/SetCollisionLevel \
  "{robot_id: 0, level: 3}"

# 设置工具坐标系偏移
ros2 service call /jaka/set_tool_offset qyh_jaka_control_msgs/srv/SetToolOffset \
  "{robot_id: 0, tool_offset: {position: {x: 0.0, y: 0.0, z: 0.15}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### 5. 查询机器人状态
```bash
# 主动查询状态
ros2 service call /jaka/get_robot_state qyh_jaka_control_msgs/srv/GetRobotState

# 订阅状态话题（持续接收）
ros2 topic echo /jaka/robot_state

# 在RViz中可视化（订阅/joint_states）
rviz2
```

## 数据格式
录制的CSV文件包含以下字段：
- `timestamp`: 相对录制开始的时间戳（秒）。
- `left_vr_*`: 左手VR位姿 (x, y, z, qx, qy, qz, qw)。
- `right_vr_*`: 右手VR位姿 (x, y, z, qx, qy, qz, qw)。
- *(注：根据具体实现，可能还包含关节角度和末端位姿)*


对于人体的某个姿势（左右对称）的情况下，我们的机械臂： 1 3 5 6 轴是反的 ， 2 4轴是相同的