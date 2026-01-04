# QYH Head Motor Control

ROS2 C++ 头部电机位置控制包，使用 Bus Servo 协议控制头部舵机。

## 概述

本包使用 ros_robot_controller 主板的 Bus Servo 接口实现头部电机的位置控制。与电机接口(Motor)不同，Bus Servo 接口支持位置控制（脉宽 0-1000），适合需要精确位置控制的头部云台。

## 功能

- **位置控制**: 支持 0-1000 脉宽位置控制
- **归一化控制**: 支持 [-1, 1] 归一化位置输入
- **关节状态发布**: 以弧度发布关节位置 (兼容 embodied AI 数据格式)
- **电机ID设置**: 支持设置/读取舵机ID
- **扭矩控制**: 支持使能/失能电机扭矩

## 硬件接口

- **串口**: `/dev/ttyACM0` (ros_robot_controller 主板)
- **波特率**: 1000000
- **协议**: Bus Servo (功能码 0x05)

## 安装

```bash
cd ~/qyh-robot-system/qyh_jushen_ws
colcon build --packages-select qyh_head_motor_control
source install/setup.bash
```

## 使用

### 启动节点

```bash
# 使用默认配置
ros2 launch qyh_head_motor_control head_motor.launch.py

# 指定串口
ros2 launch qyh_head_motor_control head_motor.launch.py serial_port:=/dev/ttyACM0

# 使用自定义配置文件
ros2 launch qyh_head_motor_control head_motor.launch.py \
    config_file:=/path/to/custom_config.yaml
```

### 话题

| 话题名 | 类型 | 描述 |
|--------|------|------|
| `~/joint_states` | `sensor_msgs/JointState` | 发布关节状态 (位置为弧度) |
| `~/cmd_position` | `std_msgs/Float64MultiArray` | 订阅归一化位置命令 [-1, 1] |
| `~/cmd_position_raw` | `std_msgs/Int32MultiArray` | 订阅原始位置命令 [0-1000] |

### 服务

| 服务名 | 类型 | 描述 |
|--------|------|------|
| `~/enable_torque` | `std_srvs/SetBool` | 使能/失能电机扭矩 |

### 示例命令

```bash
# 发送归一化位置命令 (两个电机都设为中间位置)
ros2 topic pub /head_motor_node/cmd_position std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0]"

# 发送原始位置命令 (两个电机都设为中间位置 500)
ros2 topic pub /head_motor_node/cmd_position_raw std_msgs/msg/Int32MultiArray \
    "data: [500, 500]"

# 查看关节状态
ros2 topic echo /head_motor_node/joint_states

# 使能扭矩
ros2 service call /head_motor_node/enable_torque std_srvs/srv/SetBool "data: true"

# 失能扭矩 (电机可自由转动)
ros2 service call /head_motor_node/enable_torque std_srvs/srv/SetBool "data: false"
```

## 配置参数

配置文件位于 `config/head_motor_params.yaml`:

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `serial_port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | int | `1000000` | 波特率 |
| `motor_ids` | int[] | `[1, 2]` | 电机ID列表 |
| `joint_names` | string[] | `["head_pan_joint", "head_tilt_joint"]` | 关节名称 |
| `position_min_rad` | double[] | `[-1.5708, -0.7854]` | 最小位置 (弧度) |
| `position_max_rad` | double[] | `[1.5708, 0.7854]` | 最大位置 (弧度) |
| `publish_rate` | int | `20` | 发布频率 (Hz) |
| `move_duration_ms` | int | `100` | 运动时间 (ms) |

## 位置映射

### 原始位置 ↔ 归一化位置
- 0 → -1.0
- 500 → 0.0
- 1000 → +1.0

### 原始位置 ↔ 弧度
- 0 → position_min_rad
- 1000 → position_max_rad

## 协议说明

### Bus Servo 数据帧格式

```
| 0xAA | 0x55 | Func | Length | Data... | CRC8 |
```

- **Func**: 功能码 (0x05 = Bus Servo)
- **Length**: 数据长度
- **CRC8**: CRC8-MAXIM 校验 (polynomial 0x31, init 0xFF, output inverted)

### 子命令

| 子命令 | 值 | 描述 |
|--------|-----|------|
| SET_POSITION | 0x01 | 位置控制 |
| STOP | 0x03 | 停止 |
| READ_POSITION | 0x05 | 读取位置 |
| ENABLE_TORQUE | 0x0B | 掉电(失能) |
| DISABLE_TORQUE | 0x0C | 上电(使能) |
| SET_ID | 0x10 | 设置ID |
| SET_OFFSET | 0x20 | 设置偏差 |
| SET_ANGLE_LIMIT | 0x30 | 设置角度限位 |

## 与 Embodied AI 集成

本节点发布的 `JointState` 消息使用弧度作为位置单位，与 ACT/ACT++ 等 embodied AI 框架的数据格式兼容：

```python
# 数据采集示例
head_position_rad = joint_states.position  # [pan_rad, tilt_rad]
```

## 故障排除

### 串口权限问题
```bash
sudo chmod 666 /dev/ttyACM0
# 或添加用户到 dialout 组
sudo usermod -a -G dialout $USER
```

### 电机无响应
1. 检查串口连接
2. 确认电机ID配置正确
3. 使用 `ros2 service call` 尝试使能扭矩
4. 检查电源供应

## 舵机ID设置工具

当需要设置舵机ID时（例如两个舵机出厂时都是ID=1），使用 `set_servo_id` 工具。

### 使用方法

⚠️ **重要**: 使用此工具时，必须只连接一个舵机！

```bash
# 将舵机ID设置为 2
ros2 run qyh_head_motor_control set_servo_id --ros-args -p new_id:=2

# 指定串口（默认 /dev/ttyUSB0）
ros2 run qyh_head_motor_control set_servo_id --ros-args \
    -p new_id:=2 \
    -p port:=/dev/ttyACM0

# 指定波特率（默认 1000000）
ros2 run qyh_head_motor_control set_servo_id --ros-args \
    -p new_id:=2 \
    -p port:=/dev/ttyACM0 \
    -p baudrate:=115200
```

### 参数

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `new_id` | int | 必填 | 新的舵机ID (0-253) |
| `port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | int | `1000000` | 波特率 |

### 操作流程

1. **断开所有舵机**
2. **只连接需要设置ID的那个舵机**
3. 运行工具设置ID
4. 工具会自动扫描并验证只有一个舵机连接
5. 设置成功后，可以连接下一个舵机重复操作

### 示例：设置两个舵机

```bash
# 第一步：只连接第一个舵机，设置为ID=1
ros2 run qyh_head_motor_control set_servo_id --ros-args -p new_id:=1 -p port:=/dev/ttyACM0

# 第二步：断开第一个舵机，只连接第二个舵机，设置为ID=2
ros2 run qyh_head_motor_control set_servo_id --ros-args -p new_id:=2 -p port:=/dev/ttyACM0

# 第三步：同时连接两个舵机，启动正常节点
ros2 launch qyh_head_motor_control head_motor.launch.py
```

## 文件结构

```
qyh_head_motor_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── head_motor_params.yaml
├── include/qyh_head_motor_control/
│   ├── bus_servo_protocol.hpp
│   └── head_motor_node.hpp
├── launch/
│   └── head_motor.launch.py
└── src/
    ├── bus_servo_protocol.cpp
    ├── head_motor_node.cpp
    └── set_servo_id.cpp      # 舵机ID设置工具
```

## License

Apache 2.0
