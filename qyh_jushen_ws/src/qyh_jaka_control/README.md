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

## 使用方法

### 启动节点
```bash
# 使用默认参数启动
ros2 run qyh_jaka_control jaka_control_node

# 指定IP启动
ros2 run qyh_jaka_control jaka_control_node --ros-args -p robot_ip:="192.168.1.100"
```

### 典型操作流程

1. **启动与连接**
   ```bash
   ros2 run qyh_jaka_control jaka_control_node
   ros2 service call /jaka/robot/power_on std_srvs/srv/Trigger
   ros2 service call /jaka/robot/enable std_srvs/srv/Trigger
   ```

2. **开启伺服控制**
   ```bash
   ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo
   ```

3. **VR遥操作与录制**
   ```bash
   # 开启VR跟随
   ros2 service call /jaka/vr/enable qyh_jaka_control_msgs/srv/EnableVRFollow "{enable: true}"
   
   # 开始录制
   ros2 service call /jaka/vr/start_recording qyh_jaka_control_msgs/srv/StartRecording "{output_path: '/home/user/data'}"
   
   # ... 进行操作 ...
   
   # 停止录制
   ros2 service call /jaka/vr/stop_recording qyh_jaka_control_msgs/srv/StopRecording
   ```

## 数据格式
录制的CSV文件包含以下字段：
- `timestamp`: 相对录制开始的时间戳（秒）。
- `left_vr_*`: 左手VR位姿 (x, y, z, qx, qy, qz, qw)。
- `right_vr_*`: 右手VR位姿 (x, y, z, qx, qy, qz, qw)。
- *(注：根据具体实现，可能还包含关节角度和末端位姿)*
