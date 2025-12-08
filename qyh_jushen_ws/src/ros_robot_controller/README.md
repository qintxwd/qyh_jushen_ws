# ROS Robot Controller

ROS2 驱动包，用于控制 STM32 扩展板，支持电机控制、舵机控制、IMU 数据读取等功能。

## 目录
- [硬件连接](#硬件连接)
- [安装与配置](#安装与配置)
- [启动节点](#启动节点)
- [电机控制](#电机控制)
- [其他功能](#其他功能)
- [通信协议](#通信协议)

---

## 硬件连接

- **串口设备**: `/dev/ttyACM0`
- **波特率**: 1000000 (1Mbps)
- **Vendor ID**: `1a86`
- **Product ID**: `55d4`

---

## 安装与配置

### 1. 编译包

```bash
cd ~/ros2_ws
colcon build --packages-select ros_robot_controller ros_robot_controller_msgs
source install/setup.bash
```

### 2. 设置 udev 规则（设置串口权限）

```bash
cd src/ros_robot_controller/scripts
sudo ./create_udev_rules.sh
```

这会将 `/dev/ttyACM0` 的权限设置为 `0777`，避免权限问题。

---

## 启动节点

```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.xml
```

或直接运行节点：

```bash
ros2 run ros_robot_controller ros_robot_controller
```

### 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `imu_frame` | `imu_link` | IMU 数据的 frame_id |

---

## 电机控制

### Topic

**订阅**: `~/set_motor` (`ros_robot_controller_msgs/MotorsState`)

### 消息格式

```
# MotorsState.msg
ros_robot_controller_msgs/MotorState[] data

# MotorState.msg
uint16 id      # 电机ID (1, 2, 3, 4...)
float64 rps    # 转速 (转/秒, r/s)
```

### 使用示例

#### 命令行控制电机

```bash
# 控制单个电机：电机1 以 0.5 r/s 转动
ros2 topic pub /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \
  "{data: [{id: 1, rps: 0.5}]}" --once

# 控制多个电机：电机1 以 -0.3 r/s，电机2 以 0.3 r/s 转动
ros2 topic pub /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \
  "{data: [{id: 1, rps: -0.3}, {id: 2, rps: 0.3}]}" --once

# 停止所有电机
ros2 topic pub /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState \
  "{data: [{id: 1, rps: 0}, {id: 2, rps: 0}, {id: 3, rps: 0}, {id: 4, rps: 0}]}" --once
```

#### Python 代码示例

```python
import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.publisher = self.create_publisher(
            MotorsState, 
            '/ros_robot_controller/set_motor', 
            10
        )
    
    def set_motor_speed(self, motor_id: int, rps: float):
        """设置单个电机转速"""
        msg = MotorsState()
        motor = MotorState()
        motor.id = motor_id
        motor.rps = rps
        msg.data = [motor]
        self.publisher.publish(msg)
    
    def set_motors_speed(self, speeds: list):
        """设置多个电机转速
        Args:
            speeds: [(id1, rps1), (id2, rps2), ...]
        """
        msg = MotorsState()
        for motor_id, rps in speeds:
            motor = MotorState()
            motor.id = motor_id
            motor.rps = rps
            msg.data.append(motor)
        self.publisher.publish(msg)
    
    def stop_all(self):
        """停止所有电机"""
        self.set_motors_speed([(1, 0), (2, 0), (3, 0), (4, 0)])

def main():
    rclpy.init()
    controller = MotorController()
    
    # 示例：电机1 以 0.5 r/s 转动
    controller.set_motor_speed(1, 0.5)
    
    # 示例：多电机控制
    controller.set_motors_speed([
        (1, -0.3),  # 电机1 反转 0.3 r/s
        (2, 0.3),   # 电机2 正转 0.3 r/s
    ])
    
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
```

### 电机速度单位

- **单位**: `rps` (转/秒, revolutions per second)
- **正值**: 正转
- **负值**: 反转
- **0**: 停止

### SDK 直接调用

如果不使用 ROS2，可以直接使用 SDK：

```python
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device="/dev/ttyACM0", baudrate=1000000)
board.enable_reception()

# 设置电机速度: [[电机ID, 转速(r/s)], ...]
board.set_motor_speed([[1, 0.5]])                    # 电机1: 0.5 r/s
board.set_motor_speed([[1, -0.3], [2, 0.3]])         # 多电机控制
board.set_motor_speed([[1, 0], [2, 0]])              # 停止
```

---

## 其他功能

### 发布的 Topics

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `~/imu_raw` | `sensor_msgs/Imu` | IMU 原始数据 (加速度、角速度) |
| `~/joy` | `sensor_msgs/Joy` | 手柄数据 |
| `~/sbus` | `ros_robot_controller_msgs/Sbus` | 航模遥控器数据 |
| `~/button` | `ros_robot_controller_msgs/ButtonState` | 板载按键状态 |
| `~/battery` | `std_msgs/UInt16` | 电池电压 (mV) |

### 订阅的 Topics

| Topic | 消息类型 | 说明 |
|-------|----------|------|
| `~/set_motor` | `ros_robot_controller_msgs/MotorsState` | 电机控制 |
| `~/set_led` | `ros_robot_controller_msgs/LedState` | LED 控制 |
| `~/set_buzzer` | `ros_robot_controller_msgs/BuzzerState` | 蜂鸣器控制 |
| `~/bus_servo/set_state` | `ros_robot_controller_msgs/SetBusServoState` | 总线舵机控制 |
| `~/pwm_servo/set_state` | `ros_robot_controller_msgs/SetPWMServoState` | PWM 舵机控制 |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `~/bus_servo/get_state` | `GetBusServoState` | 读取总线舵机状态 |
| `~/pwm_servo/get_state` | `GetPWMServoState` | 读取 PWM 舵机状态 |

### LED 控制示例

```bash
# LED 闪烁：亮0.1秒，灭0.9秒，重复3次
ros2 topic pub /ros_robot_controller/set_led ros_robot_controller_msgs/msg/LedState \
  "{id: 1, on_time: 0.1, off_time: 0.9, repeat: 3}" --once
```

### 蜂鸣器控制示例

```bash
# 蜂鸣器：1000Hz，响0.1秒，停0.9秒，重复1次
ros2 topic pub /ros_robot_controller/set_buzzer ros_robot_controller_msgs/msg/BuzzerState \
  "{freq: 1000, on_time: 0.1, off_time: 0.9, repeat: 1}" --once
```

---

## 通信协议

### 帧格式

```
┌──────────┬──────────┬──────────┬──────────┬──────────┬──────────┐
│  0xAA    │  0x55    │ Function │  Length  │   Data   │   CRC8   │
│ (帧头1)  │ (帧头2)  │ (功能码) │ (数据长度)│  (数据)  │ (校验)   │
└──────────┴──────────┴──────────┴──────────┴──────────┴──────────┘
```

### 功能码

| 功能码 | 值 | 说明 |
|--------|-----|------|
| `PACKET_FUNC_SYS` | 0 | 系统 (电池等) |
| `PACKET_FUNC_LED` | 1 | LED 控制 |
| `PACKET_FUNC_BUZZER` | 2 | 蜂鸣器控制 |
| `PACKET_FUNC_MOTOR` | 3 | 电机控制 |
| `PACKET_FUNC_PWM_SERVO` | 4 | PWM 舵机控制 |
| `PACKET_FUNC_BUS_SERVO` | 5 | 总线舵机控制 |
| `PACKET_FUNC_KEY` | 6 | 按键 |
| `PACKET_FUNC_IMU` | 7 | IMU |
| `PACKET_FUNC_GAMEPAD` | 8 | 手柄 |
| `PACKET_FUNC_SBUS` | 9 | 航模遥控 |

### 电机控制协议

**功能码**: `0x03` (PACKET_FUNC_MOTOR)

**子命令 0x01**: 多电机速度控制

```
数据格式: [0x01, N, motor_id_1, speed_1(float), motor_id_2, speed_2(float), ...]
- 0x01: 子命令
- N: 电机数量
- motor_id: 电机ID (0-based, 发送时会自动减1)
- speed: 4字节 float, 单位 r/s
```

### CRC8 校验

- 多项式: 0x31 (CRC-8/Maxim)
- 初始值: 0x00
- 计算范围: Function + Length + Data

---

## 故障排除

### 1. 串口权限问题

```bash
# 检查设备
ls -la /dev/ttyACM0

# 临时解决
sudo chmod 777 /dev/ttyACM0

# 永久解决 (设置 udev 规则)
cd src/ros_robot_controller/scripts
sudo ./create_udev_rules.sh
```

### 2. 找不到设备

```bash
# 查看连接的 USB 设备
lsusb

# 查看串口设备
ls /dev/ttyACM* /dev/ttyUSB*

# 查看内核日志
dmesg | tail -20
```

### 3. 通信失败

- 检查波特率是否为 1000000
- 检查 CRC 校验是否正确
- 使用 `minicom` 或 `screen` 测试串口通信

---

## 依赖

- ROS2 Humble
- Python 3
- pyserial

```bash
pip install pyserial
```
