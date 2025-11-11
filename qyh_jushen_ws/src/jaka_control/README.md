# JAKA Control - ROS 2 控制包

这个ROS 2包为节卡(JAKA)双臂机器人提供了完整的控制接口。

## 功能特性

### 服务接口

#### 基础控制
- `~/power_on` - 机器人上电
- `~/power_off` - 机器人下电
- `~/enable_robot` - 使能机器人
- `~/disable_robot` - 下使能机器人
- `~/clear_error` - 清除错误
- `~/motion_abort` - 终止当前运动

#### 运动控制
- `~/move_j` - 关节空间运动
  - 支持绝对/相对运动模式
  - 支持阻塞/非阻塞模式
  - 可控制单臂或双臂同步运动
  
- `~/move_l` - 笛卡尔空间直线运动
  - 末端执行器走直线轨迹
  - 支持绝对/相对运动模式
  - 支持阻塞/非阻塞模式

#### 配置服务
- `~/set_collision_level` - 设置碰撞检测等级(0-5)
- `~/set_tool_offset` - 设置工具坐标系偏置
- `~/get_robot_state` - 获取机器人状态

#### 伺服控制
- `~/servo_move_enable` - 使能/关闭伺服模式
  - 伺服模式用于高频率、低延迟的实时控制
  - 适合复杂轨迹跟踪和力控应用
  
- `~/servo_j` - 伺服关节空间运动
  - 在伺服模式下发送关节位置指令
  - 需要周期性调用(100-1000Hz)
  
- `~/servo_p` - 伺服笛卡尔空间运动
  - 在伺服模式下发送笛卡尔位置指令
  - 支持实时轨迹跟踪
  
- `~/set_servo_filter` - 设置伺服模式滤波器
  - 支持多种滤波器类型
  - 平衡轨迹精度和运动平滑性

### 话题发布

- `/jaka_robot/joint_states` (sensor_msgs/JointState) - 关节状态(兼容RViz)
- `~/robot_status` (jaka_control/msg/RobotStatus) - 完整机器人状态
- `~/left_arm/cartesian_pose` (geometry_msgs/PoseStamped) - 左臂末端位姿
- `~/right_arm/cartesian_pose` (geometry_msgs/PoseStamped) - 右臂末端位姿

## 安装和编译

### 前提条件

1. ROS 2 (Humble/Foxy或更高版本)
2. JAKA机器人SDK (位于`../../资料/双机械臂/SDK-2.3.0.5/SDK`)

### 编译

```bash
cd qyh_jushen_ws
colcon build --packages-select jaka_control
source install/setup.bash
```

## 使用方法

### 1. 基本启动

```bash
ros2 launch jaka_control jaka_robot.launch.py
```

### 2. 自定义IP地址启动

```bash
ros2 launch jaka_control jaka_robot.launch.py robot_ip:=192.168.2.200
```

### 3. 自动上电并使能

```bash
ros2 launch jaka_control jaka_robot.launch.py auto_power_on:=true auto_enable:=true
```

### 4. 使用自定义配置文件

```bash
ros2 launch jaka_control jaka_robot.launch.py config_file:=/path/to/your/config.yaml
```

## 服务调用示例

### 上电和使能

```bash
# 上电
ros2 service call /jaka_robot_node/power_on jaka_control/srv/PowerOn

# 使能
ros2 service call /jaka_robot_node/enable_robot jaka_control/srv/EnableRobot
```

### 关节运动

```bash
# 左臂关节运动到指定位置(弧度) - 需要14个关节值
ros2 service call /jaka_robot_node/move_j jaka_control/srv/MoveJ "{
  robot_id: 0,
  joint_positions: [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  move_mode: false,
  velocity: 0.5,
  acceleration: 1.0,
  is_block: true
}
}"
```

### 笛卡尔空间运动

```bash
# 左臂末端移动到指定位置(位置单位:米, 姿态单位:弧度)
ros2 service call /jaka_robot_node/move_l jaka_control/srv/MoveL "{
  robot_id: 0,
  target_pose: {
    position: {x: 0.5, y: 0.0, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  move_mode: false,
  velocity: 100.0,
  acceleration: 200.0,
  is_block: true
}"
```

### 设置碰撞等级

```bash
# 设置左臂碰撞等级为2(中等灵敏度)
ros2 service call /jaka_robot_node/set_collision_level jaka_control/srv/SetCollisionLevel "{
  robot_id: 0,
  level: 2
}"
```

### 终止运动

```bash
ros2 service call /jaka_robot_node/motion_abort jaka_control/srv/MotionAbort
```

### 查询机器人状态

```bash
ros2 service call /jaka_robot_node/get_robot_state jaka_control/srv/GetRobotState
```

## 伺服控制使用

### 伺服模式说明

伺服模式是一种高频率、低延迟的实时控制模式，适用于：
- 复杂轨迹跟踪
- 力控应用
- 实时路径规划
- 视觉伺服

**重要特性：**
- 控制频率：100-1000Hz
- 需要周期性发送指令
- 退出伺服模式前必须关闭
- 支持多种滤波器优化运动

### 伺服模式基本流程

```bash
# 1. 设置滤波器(可选，在使能前设置)
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{
  filter_type: 1,
  cutoff_freq: 125.0
}"

# 2. 使能伺服模式
ros2 service call /jaka_robot_node/servo_move_enable jaka_control/srv/ServoMoveEnable "{
  enable: true,
  robot_id: 0
}"

# 3. 周期性发送伺服指令(14个关节值)
ros2 service call /jaka_robot_node/servo_j jaka_control/srv/ServoJ "{
  robot_id: 0,
  joint_positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  move_mode: false
}"

# 4. 关闭伺服模式
ros2 service call /jaka_robot_node/servo_move_enable jaka_control/srv/ServoMoveEnable "{
  enable: false,
  robot_id: 0
}"
```

### 滤波器类型

```bash
# 0: 不使用滤波器(最快响应，但可能有抖动)
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{filter_type: 0}"

# 1: 关节空间低通滤波器(推荐用于一般轨迹跟踪)
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{
  filter_type: 1,
  cutoff_freq: 125.0
}"

# 2: 关节空间非线性滤波器(用于复杂姿态变化)
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{
  filter_type: 2,
  nlf_params: [0.0, 0.0, 0.0, 100.0, 500.0, 1000.0]
}"

# 3: 笛卡尔空间非线性滤波器(用于末端轨迹优化)
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{
  filter_type: 3,
  nlf_params: [500.0, 2000.0, 10000.0, 100.0, 500.0, 1000.0]
}"
```

## Python客户端示例

```python
import rclpy
from rclpy.node import Node
from jaka_control.srv import EnableRobot, MoveJ

class JakaClient(Node):
    def __init__(self):
        super().__init__('jaka_client')
        self.enable_client = self.create_client(EnableRobot, '/jaka_robot_node/enable_robot')
        self.move_j_client = self.create_client(MoveJ, '/jaka_robot_node/move_j')
        
    def enable_robot(self):
        request = EnableRobot.Request()
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def move_joint(self, positions):
        request = MoveJ.Request()
        request.robot_id = 0  # LEFT
        request.joint_positions = positions
        request.move_mode = False  # ABS
        request.velocity = 0.5
        request.acceleration = 1.0
        request.is_block = True
        
        future = self.move_j_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = JakaClient()
    
    # 使能机器人
    result = client.enable_robot()
    print(f"Enable result: {result.success}, {result.message}")
    
    # 移动到目标位置
    target = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    result = client.move_joint(target)
    print(f"Move result: {result.success}, {result.message}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 伺服控制Python示例

```python
import rclpy
from rclpy.node import Node
from jaka_control.srv import ServoMoveEnable, ServoJ
import math
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.servo_enable_client = self.create_client(
            ServoMoveEnable, '/jaka_robot_node/servo_move_enable')
        self.servo_j_client = self.create_client(
            ServoJ, '/jaka_robot_node/servo_j')
    
    def enable_servo(self, robot_id=0):
        request = ServoMoveEnable.Request()
        request.enable = True
        request.robot_id = robot_id
        future = self.servo_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def servo_move(self, robot_id, positions):
        request = ServoJ.Request()
        request.robot_id = robot_id
        request.joint_positions = positions
        request.move_mode = False
        future = self.servo_j_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    controller = ServoController()
    
    # 使能伺服模式
    controller.enable_servo(0)
    time.sleep(1.0)
    
    # 正弦波运动
    for i in range(1000):
        angle = 0.3 * math.sin(2 * math.pi * i / 200)
        positions = [angle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        controller.servo_move(0, positions)
        time.sleep(0.01)  # 100Hz
    
    # 关闭伺服模式
    request = ServoMoveEnable.Request()
    request.enable = False
    request.robot_id = 0
    controller.servo_enable_client.call_async(request)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 配置参数

### robot_config.yaml

```yaml
robot_ip: "192.168.2.200"           # 机器人IP地址
status_publish_rate: 10.0           # 状态发布频率(Hz)
auto_connect: true                  # 自动连接
auto_power_on: false                # 自动上电
auto_enable: false                  # 自动使能
default_joint_velocity: 0.5         # 默认关节速度(rad/s)
default_joint_acceleration: 1.0     # 默认关节加速度(rad/s^2)
default_cartesian_velocity: 100.0   # 默认笛卡尔速度(mm/s)
default_cartesian_acceleration: 200.0  # 默认笛卡尔加速度(mm/s^2)
default_collision_level: 2          # 默认碰撞等级(0-5)
```

## 机器人ID说明

- `LEFT = 0` - 左臂
- `RIGHT = 1` - 右臂
- `DUAL = -1` - 双臂同步运动

## 运动模式说明

- `ABS (false)` - 绝对运动：运动到指定的绝对位置
- `INCR (true)` - 相对运动：相对当前位置进行增量运动

## 阻塞模式说明

- `TRUE` - 阻塞模式：等待运动完成后才返回
- `FALSE` - 非阻塞模式：发送指令后立即返回

## 碰撞等级说明

- `0` - 关闭碰撞检测
- `1` - 最低灵敏度 (25N)
- `2` - 低灵敏度 (50N)
- `3` - 中等灵敏度 (75N)
- `4` - 高灵敏度 (100N)
- `5` - 最高灵敏度 (125N)

## 图形界面控制 (GUI)

### GUI控制面板

本包提供了一个可视化的GUI控制程序 `gui_joint_control.py`，特别适合调试和测试：

**功能特点**：
- 🎛️ 双臂各7个关节的可视化控制
- 🔘 每个关节的增加/减小按钮（步进5°）
- 📊 滑动条直观调整角度（-180° ~ +180°）
- 📈 实时显示角度值（度和弧度）
- ⚡ 快速复位和初始位置功能
- 🎯 支持单臂/双臂运动模式选择
- 🔄 实时服务状态监控

**启动方法**：

```bash
# 1. 安装PyQt5
pip install PyQt5

# 2. 启动机器人节点
ros2 launch jaka_control jaka_robot.launch.py

# 3. 启动GUI（新终端）
python3 ~/qyh_jushen_ws/src/jaka_control/scripts/gui_joint_control.py
```

**详细使用说明请查看**: [`GUI_USAGE.md`](GUI_USAGE.md)

### Python脚本示例

包含多个Python示例脚本：

1. **基本控制** (`example_control.py`)
   - 上电、使能流程
   - 关节运动示例
   - 笛卡尔运动示例
   - 双臂同步运动

2. **伺服控制** (`example_servo_control.py`)
   - 正弦波轨迹控制
   - 圆形轨迹控制
   - 增量运动示例

3. **GUI控制** (`gui_joint_control.py`)
   - 可视化关节控制
   - 实时角度显示
   - 交互式操作

运行示例：
```bash
cd ~/qyh_jushen_ws/src/jaka_control/scripts
chmod +x *.py
python3 example_control.py
```

## 注意事项

1. **安全警告**：在使用机器人前，请确保工作区域安全，人员远离机器人运动范围
2. **下使能等待**：下使能后至少等待3秒再上使能，避免硬件损坏
3. **碰撞等级**：根据应用场景合理设置碰撞等级，过高容易误触发，过低降低安全性
4. **错误处理**：某些严重错误需要重新上下电才能清除
5. **坐标系统**：
   - 关节角度单位：弧度(rad)
   - 笛卡尔位置单位：ROS使用米(m)，JAKA SDK使用毫米(mm)，本包自动转换
   - 姿态表示：ROS使用四元数，JAKA使用欧拉角，本包自动转换
6. **伺服模式**：
   - 使能伺服前必须先设置滤波器
   - 伺服模式下需要周期性发送指令(建议100-1000Hz)
   - 退出伺服前应逐渐减速，避免突然停止
   - 伺服模式下不能调用普通运动指令(movej/movel)
   - 伺服模式关闭后才能切换到其他模式

## 故障排除

### 无法连接到机器人
- 检查IP地址是否正确
- 检查网络连接
- 确认机器人控制器已启动

### 使能失败
- 检查是否有错误，调用`clear_error`清除
- 确认机器人已上电
- 检查急停按钮是否按下

### 运动指令失败
- 检查机器人是否使能
- 检查目标位置是否在工作范围内
- 检查是否超出关节限位
- 查看错误信息获取详细原因

## 开发者信息

- 维护者: [Your Name]
- 邮箱: [your_email@example.com]
- 版本: 1.0.0
- 许可证: BSD

## 参考资料

- [JAKA机器人官方文档](https://www.jaka.com)
- [ROS 2文档](https://docs.ros.org)
- SDK示例代码：`../../资料/双机械臂/SDK-2.3.0.5/SDK/`
