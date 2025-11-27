# qyh_jaka_control

本功能包提供了JAKA双臂机器人系统的核心控制节点。它负责与机器人控制器通信，提供ROS 2控制接口，**接收来自遥操作控制器的关节/笛卡尔命令**，并执行平滑伺服控制。

## 架构说明

根据新的VR遥操作架构，本包在整个系统中的位置：

```
VR手柄 → qyh_vr_calibration → qyh_teleoperation_controller → qyh_jaka_control → JAKA机器人
          (坐标变换+校准)      (差分IK+轨迹平滑)          (伺服执行)
```

**关键职责**:
- ✅ 接收关节/笛卡尔命令（来自 `qyh_teleoperation_controller`）
- ✅ 轨迹缓冲和插值平滑（通过 `smooth_servo_bridge`）
- ✅ 125Hz EtherCAT实时伺服控制
- ✅ 基础机器人控制（上下电、使能、错误处理）
- ❌ **不再**直接订阅VR数据
- ❌ **不再**计算VR到机器人的坐标变换

详细架构请参考: [TELEOPERATION_INTEGRATION_GUIDE.md](/TELEOPERATION_INTEGRATION_GUIDE.md)

## SDK 版本

- **JAKA SDK**: 2.3.0.13 (NVIDIA Jetson 优化版)
- **更新日期**: 2025-07-18
- **库文件**: `thirdparty/lib/libjakaAPI_2_3_0_13.so`
- **主要特性**:
  - 支持末端传感器扭矩反馈 (edg_get_stat)
  - 支持读写双臂安装位置
  - 废弃 edg_recv 接口，edg_get_stat 已脱离依赖
  - 多线程安全优化
  - edg优先级提升至98，绑定到CPU10

## 核心节点: jaka_control_node

### 功能描述
`jaka_control_node` 通过以太网连接到JAKA机器人控制器，主要功能包括：
- **基础控制**：上下电、使能/去使能、错误清除、急停。
- **实时伺服**：支持关节空间和笛卡尔空间的实时控制（默认125Hz）。
- **平滑执行**：通过 `smooth_servo_bridge` 提供轨迹缓冲和插值。
- **状态发布**：发布机器人状态和关节状态供其他节点使用。

**注意**: VR数据处理和坐标变换由 `qyh_vr_calibration` 和 `qyh_teleoperation_controller` 负责。

### 参数列表 (Parameters)

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `robot_ip` | string | "192.168.2.200" | JAKA机器人控制器的IP地址。 |
| `cycle_time_ms` | double | 8.0 | 控制周期（毫秒），8ms对应125Hz。 |
| `use_cartesian` | bool | false | 是否使用笛卡尔空间控制（False为关节空间）。 |
| `auto_connect` | bool | true | 启动时是否自动连接机器人。 |
| `auto_initialize` | bool | false | 连接成功后是否自动执行初始化流程（上电+使能+归位）。 |

### 订阅话题 (Subscribed Topics)

- `/jaka/servo/joint_cmd` (`qyh_jaka_control_msgs/msg/JakaDualJointServo`)
    - 双臂关节位置控制指令（通常来自 `qyh_teleoperation_controller`）。
- `/jaka/servo/cartesian_cmd` (`qyh_jaka_control_msgs/msg/JakaDualCartesianServo`)
    - 双臂笛卡尔位姿控制指令（可选）。

### 发布话题 (Published Topics)

- `/jaka/servo/status` (`qyh_jaka_control_msgs/msg/JakaServoStatus`)
    - 伺服控制回路的实时状态（10Hz）。
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

**通常由 `qyh_teleoperation_controller` 发送命令，无需手动操作。**

```bash
# 启动与使能（同上）

# 开启伺服模式
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo

# 命令由 qyh_teleoperation_controller 自动发送到 /jaka/servo/joint_cmd

# 停止伺服模式
ros2 service call /jaka/servo/stop qyh_jaka_control_msgs/srv/StopServo
```

#### 3. 完整VR遥操作系统

参考 [TELEOPERATION_INTEGRATION_GUIDE.md](/TELEOPERATION_INTEGRATION_GUIDE.md) 中的完整启动流程：

```bash
# 方式1: 完整系统launch
ros2 launch qyh_teleoperation_bringup full_system.launch.py robot_ip:=192.168.2.200

# 方式2: 分步启动（调试）
# 终端1: VR接口
ros2 launch qyh_vr_calibration vr_interface.launch.py
# 终端2: MoveIt虚拟臂
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py
# 终端3: 遥操作控制器
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py
# 终端4: JAKA桥接（本包）
ros2 launch qyh_jaka_control jaka_bridge.launch.py robot_ip:=192.168.2.200
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

## 机械臂关节对称性


对于人体的某个姿势（左右对称）的情况下，我们的机械臂： 1 3 5 6 轴是反的 ， 2 4轴是相同的