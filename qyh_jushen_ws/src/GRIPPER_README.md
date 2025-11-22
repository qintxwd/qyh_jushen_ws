# QYH JODELL EPG Gripper Control System

完整的ROS2夹爪控制系统，包含双手夹爪的控制、状态发布和GUI监控。

## 系统架构

```
qyh_gripper_msgs/          # 消息和服务定义
├── msg/
│   └── GripperState.msg   # 夹爪状态（位置、力反馈、物体检测等）
└── srv/
    ├── ActivateGripper.srv    # 激活服务
    ├── MoveGripper.srv         # 移动控制服务
    └── GetGripperState.srv     # 状态查询服务

qyh_gripper_control/       # C++ 控制节点
├── src/
│   ├── gripper_control_node.cpp      # 主控制节点
│   └── gripper_control_node_main.cpp
├── include/
├── config/
│   ├── gripper_left.yaml   # 左手配置
│   └── gripper_right.yaml  # 右手配置
├── launch/
│   ├── gripper_control.launch.py   # 单个夹爪启动
│   └── dual_gripper.launch.py       # 双手夹爪启动
└── thirdparty/
    └── lib/linux/
        └── libmodbus.so.*   # Modbus库

qyh_gripper_gui/           # PyQt5 GUI
└── qyh_gripper_gui/
    └── gripper_gui.py     # 双手夹爪监控GUI
```

## 硬件规格

- **夹爪型号**: JODELL EPG系列电动平行夹爪
- **通信方式**: Modbus RTU via RS-485
- **默认波特率**: 115200, 8N1
- **从站ID**: 1 (默认)
- **供电**: DC 24V
- **控制寄存器**: 0x03E8-0x03EA
- **状态寄存器**: 0x07D0-0x07D2

## 功能特性

### qyh_gripper_control (C++控制节点)

- ✅ Modbus RTU通信（基于libmodbus C++封装）
- ✅ 实时状态发布（20Hz）
- ✅ 激活/去激活控制
- ✅ 位置、速度、力控制
- ✅ 力反馈读取（具身智能关键数据）
- ✅ 物体检测状态
- ✅ 故障诊断
- ✅ 支持双手独立控制（/left, /right命名空间）

### qyh_gripper_gui (PyQt5 GUI)

- ✅ 双手夹爪实时状态监控
- ✅ 位置、力反馈可视化（进度条）
- ✅ 精确控制滑块（位置、速度、力）
- ✅ 快捷操作按钮
  - 全开
  - 全闭
  - 半开
  - 轻抓（柔性抓取）
- ✅ 物体检测显示
- ✅ 故障信息提示

## 安装与编译

### 1. 前置条件

```bash
# 确保已安装ROS2 Humble
# 确保已安装PyQt5
pip install PyQt5

# 复制libmodbus库到thirdparty目录
cd ~/qyh_jushen_ws/src/qyh_gripper_control
mkdir -p thirdparty/lib/linux thirdparty/include/modbus
# 从qyh_standard_robot复制或自行安装libmodbus
```

### 2. 编译

```bash
cd ~/qyh_jushen_ws
colcon build --packages-select qyh_gripper_msgs qyh_gripper_control qyh_gripper_gui
source install/setup.bash
```

## 使用方法

### 1. 启动单个夹爪控制节点

```bash
# 左手夹爪（串口 /dev/ttyUSB0）
ros2 launch qyh_gripper_control gripper_control.launch.py side:=left

# 右手夹爪（串口 /dev/ttyUSB1）
ros2 launch qyh_gripper_control gripper_control.launch.py side:=right
```

### 2. 启动双手夹爪

```bash
ros2 launch qyh_gripper_control dual_gripper.launch.py
```

### 3. 启动GUI

```bash
ros2 run qyh_gripper_gui gripper_gui
```

### 4. 命令行测试

```bash
# 激活左手夹爪
ros2 service call /left/activate_gripper qyh_gripper_msgs/srv/ActivateGripper

# 移动左手夹爪（闭合到255，速度255，力150）
ros2 service call /left/move_gripper qyh_gripper_msgs/srv/MoveGripper "{position: 255, speed: 255, force: 150}"

# 查看左手夹爪状态
ros2 topic echo /left/gripper_state
```

## 配置说明

### 修改串口和波特率

编辑配置文件：

```yaml
# config/gripper_left.yaml
gripper_control_node:
  ros__parameters:
    device_port: "/dev/ttyUSB0"  # 修改为实际串口
    baudrate: 115200
    slave_id: 1
    publish_rate: 20.0
```

## ROS2 接口

### 话题 (Topics)

- `/left/gripper_state` (qyh_gripper_msgs/msg/GripperState)
  - 左手夹爪状态（20Hz发布）
  
- `/right/gripper_state` (qyh_gripper_msgs/msg/GripperState)
  - 右手夹爪状态（20Hz发布）

### 服务 (Services)

- `/left/activate_gripper` (qyh_gripper_msgs/srv/ActivateGripper)
  - 激活左手夹爪
  
- `/left/move_gripper` (qyh_gripper_msgs/srv/MoveGripper)
  - 控制左手夹爪移动
  
- `/right/activate_gripper` (qyh_gripper_msgs/srv/ActivateGripper)
  - 激活右手夹爪
  
- `/right/move_gripper` (qyh_gripper_msgs/srv/MoveGripper)
  - 控制右手夹爪移动

## 参数说明

### MoveGripper 服务参数

```
position: uint8 (0-255)
  0   = 完全张开
  255 = 完全闭合
  
speed: uint8 (0-255)
  0   = 最慢
  255 = 最快
  
force: uint8 (0-255)
  0   = 最小力
  255 = 最大力
  建议柔性抓取使用 30-80
```

### GripperState 消息字段

```
object_status: uint8
  0 = 运动中
  1 = 内撑模式检测到物体
  2 = 外夹模式检测到物体（抓到了）
  3 = 到达目标位置（未抓到）

current_force: uint8 (0-255)
  实时力反馈/电流值
  用于具身智能感知接触力度

fault_code: uint8
  0x00 = 无故障
  0x01 = 电机未激活
  0x08 = 过流/卡死
  0x10 = 电压异常
  0x40 = 过温
```

## 故障排查

### 1. 无法连接夹爪

- 检查串口权限：`sudo chmod 666 /dev/ttyUSB0`
- 检查串口号：`ls /dev/ttyUSB*`
- 检查Modbus从站ID是否正确（默认1）

### 2. 夹爪不响应

- 确保已调用激活服务
- 检查24V电源供电
- 查看故障码：`ros2 topic echo /left/gripper_state`

### 3. GUI无数据

- 确保控制节点已启动
- 检查话题：`ros2 topic list | grep gripper`
- 检查话题频率：`ros2 topic hz /left/gripper_state`

## 开发说明

### 使用C++封装的Modbus库

本项目使用C++封装的libmodbus库（来自qyh_standard_robot项目）：

```cpp
#include <modbus/modbus.hpp>

// 创建RTU连接
modbus::ModbusTCP ctx(device, baudrate);
ctx.set_slave(slave_id);
ctx.connect();

// 读写操作
ctx.read_registers(addr, count, buffer);
ctx.write_registers(addr, count, values);
```

### 扩展功能

如需添加新功能：
1. 在`qyh_gripper_msgs`中定义新的服务
2. 在`gripper_control_node.cpp`中实现服务处理
3. 在GUI中添加对应控制按钮

## 参考资料

- 夹爪说明书：`资料/夹爪/JODELL EPG 系列夹爪开发简明手册.txt`
- 力反馈说明：`资料/夹爪/JODELL_EPG_Manual_ForceFeedback.txt`
- 测试代码：`资料/夹爪/test.cpp`, `test2.cpp`

## License

MIT

## 维护者

Your Name <your_email@example.com>
