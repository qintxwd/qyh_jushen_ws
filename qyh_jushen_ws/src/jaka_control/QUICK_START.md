# JAKA双臂机器人控制包 - 快速入门指南

## 📦 包结构

```
jaka_control/
├── CMakeLists.txt              # CMake构建文件
├── package.xml                 # ROS 2包描述文件
├── README.md                   # 详细文档
├── QUICK_START.md             # 本文件
├── config/                     # 配置文件目录
│   └── robot_config.yaml      # 机器人配置
├── include/                    # C++头文件
│   └── jaka_control/
│       └── jaka_robot_interface.hpp
├── src/                        # C++源文件
│   ├── jaka_robot_interface.cpp
│   └── jaka_robot_node.cpp
├── srv/                        # 服务定义
│   ├── EnableRobot.srv
│   ├── DisableRobot.srv
│   ├── PowerOn.srv
│   ├── PowerOff.srv
│   ├── MoveJ.srv
│   ├── MoveL.srv
│   ├── SetCollisionLevel.srv
│   ├── SetToolOffset.srv
│   ├── GetRobotState.srv
│   ├── ClearError.srv
│   └── MotionAbort.srv
├── msg/                        # 消息定义
│   ├── JointPosition.msg
│   ├── CartesianPose.msg
│   └── RobotStatus.msg
├── launch/                     # 启动文件
│   └── jaka_robot.launch.py
└── scripts/                    # Python示例脚本
    └── example_control.py
```

## 🚀 快速开始

### 1. 编译包

```bash
cd ~/qyh_jushen_ws
colcon build --packages-select jaka_control
source install/setup.bash
```

### 2. 修改机器人IP

编辑配置文件：
```bash
nano src/jaka_control/config/robot_config.yaml
```

修改IP地址：
```yaml
robot_ip: "192.168.2.200"  # 改为你的机器人IP
```

### 3. 启动控制节点

```bash
ros2 launch jaka_control jaka_robot.launch.py
```

### 4. 基本操作流程

在另一个终端中执行：

```bash
# 1. 上电
ros2 service call /jaka_robot_node/power_on jaka_control/srv/PowerOn

# 2. 等待2秒后使能
ros2 service call /jaka_robot_node/enable_robot jaka_control/srv/EnableRobot

# 3. 设置碰撞等级
ros2 service call /jaka_robot_node/set_collision_level jaka_control/srv/SetCollisionLevel "{robot_id: 0, level: 2}"

# 4. 执行关节运动(左臂移动到零位) - 需要14个关节值
ros2 service call /jaka_robot_node/move_j jaka_control/srv/MoveJ "{robot_id: 0, joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_mode: false, velocity: 0.5, acceleration: 1.0, is_block: true}"
```

## 📝 常用命令

### 基本运动控制

查看可用服务：
```bash
ros2 service list | grep jaka
```

查看发布的话题：
```bash
ros2 topic list | grep jaka
```

监听机器人状态：
```bash
ros2 topic echo /jaka_robot_node/robot_status
```

紧急停止：
```bash
ros2 service call /jaka_robot_node/motion_abort jaka_control/srv/MotionAbort
```

### 伺服控制命令

使能伺服模式：
```bash
ros2 service call /jaka_robot_node/servo_move_enable jaka_control/srv/ServoMoveEnable "{enable: true, robot_id: 0}"
```

伺服关节运动(14个关节值)：
```bash
ros2 service call /jaka_robot_node/servo_j jaka_control/srv/ServoJ "{robot_id: 0, joint_positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_mode: false}"
```

设置滤波器：
```bash
ros2 service call /jaka_robot_node/set_servo_filter jaka_control/srv/SetServoFilter "{filter_type: 1, cutoff_freq: 125.0}"
```

关闭伺服模式：
```bash
ros2 service call /jaka_robot_node/servo_move_enable jaka_control/srv/ServoMoveEnable "{enable: false, robot_id: 0}"
```

## 🐍 Python脚本示例

### 运行示例脚本

基本控制示例：
```bash
cd ~/qyh_jushen_ws/src/jaka_control/scripts
chmod +x example_control.py
python3 example_control.py
```

伺服控制示例：
```bash
cd ~/qyh_jushen_ws/src/jaka_control/scripts
chmod +x example_servo_control.py
python3 example_servo_control.py
```

### 最小示例代码

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jaka_control.srv import EnableRobot, MoveJ

rclpy.init()
node = Node('simple_control')

# 创建服务客户端
enable_client = node.create_client(EnableRobot, '/jaka_robot_node/enable_robot')
move_client = node.create_client(MoveJ, '/jaka_robot_node/move_j')

# 等待服务
enable_client.wait_for_service()
move_client.wait_for_service()

# 使能机器人
request = EnableRobot.Request()
future = enable_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(f"使能结果: {future.result().message}")

# 移动机器人 (需要14个关节值: 左臂7个 + 右臂7个)
request = MoveJ.Request()
request.robot_id = 0  # 左臂
request.joint_positions = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0,  # 左臂
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # 右臂
request.move_mode = False  # 绝对运动
request.velocity = 0.5
request.acceleration = 1.0
request.is_block = True

future = move_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(f"运动结果: {future.result().message}")

rclpy.shutdown()
```

## 🔧 配置说明

### 重要参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| robot_ip | 机器人IP地址 | 192.168.2.200 |
| auto_connect | 启动时自动连接 | true |
| auto_power_on | 启动时自动上电 | false |
| auto_enable | 启动时自动使能 | false |
| status_publish_rate | 状态发布频率(Hz) | 10.0 |
| default_collision_level | 默认碰撞等级 | 2 |

### 机器人ID

- **0 (LEFT)**: 左臂
- **1 (RIGHT)**: 右臂
- **-1 (DUAL)**: 双臂同步

### 运动模式

- **false (ABS)**: 绝对运动 - 运动到指定位置
- **true (INCR)**: 相对运动 - 基于当前位置增量运动

## ⚠️ 重要提示

1. **安全第一**: 首次使用时，确保周围无人员和障碍物
2. **下使能延迟**: 下使能后至少等待3秒再上使能
3. **错误处理**: 出现错误时先调用`clear_error`
4. **速度设置**: 初次使用建议使用较低速度(0.3 rad/s)
5. **工作范围**: 确保目标位置在机器人工作范围内

## 🐛 常见问题

### Q: 无法连接到机器人
**A**: 
- 检查IP地址是否正确
- 确认与机器人在同一网络
- ping测试: `ping 192.168.2.200`

### Q: 使能失败
**A**: 
- 确认已上电: `ros2 service call /jaka_robot_node/power_on ...`
- 清除错误: `ros2 service call /jaka_robot_node/clear_error ...`
- 检查急停按钮是否释放

### Q: 运动指令不执行
**A**: 
- 确认机器人已使能
- 检查目标位置是否超出限位
- 查看错误信息: `ros2 service call /jaka_robot_node/get_robot_state ...`

### Q: 编译错误
**A**: 
- 检查SDK库文件路径是否正确
- 确认所有依赖已安装: `rosdep install --from-paths src --ignore-src -r -y`

## 📚 下一步

- 阅读完整文档: [README.md](README.md)
- 查看SDK示例: `../../资料/双机械臂/SDK-2.3.0.5/SDK/`
- 学习ROS 2: https://docs.ros.org/en/humble/

## 💡 实用技巧

### 1. 自动启动并使能

```bash
ros2 launch jaka_control jaka_robot.launch.py auto_power_on:=true auto_enable:=true
```

### 2. 使用别名简化命令

在`~/.bashrc`中添加：
```bash
alias jaka_start='ros2 launch jaka_control jaka_robot.launch.py'
alias jaka_enable='ros2 service call /jaka_robot_node/enable_robot jaka_control/srv/EnableRobot'
alias jaka_stop='ros2 service call /jaka_robot_node/motion_abort jaka_control/srv/MotionAbort'
```

### 3. 监控机器人状态

```bash
watch -n 0.5 'ros2 topic echo /jaka_robot_node/robot_status --once'
```

## 📞 获取帮助

- 查看服务定义: `ros2 interface show jaka_control/srv/MoveJ`
- 查看消息定义: `ros2 interface show jaka_control/msg/RobotStatus`
- 查看节点信息: `ros2 node info /jaka_robot_node`

---

**版本**: 1.0.0  
**更新日期**: 2025-01-11
