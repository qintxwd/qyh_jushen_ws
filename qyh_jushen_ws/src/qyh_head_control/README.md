# qyh_head_control

HTD-85H 智能总线舵机 ROS2 控制包 - 用于机器人头部双自由度运动控制（Pan/Tilt）

## 功能特性

- ✅ 双舵机控制 (Pan左右 + Tilt上下)
- ✅ 多种控制模式
  - 位置控制 (0-1000)
  - 归一化控制 (-1.0 到 1.0)
  - 目标跟踪模式
- ✅ 实时关节状态发布
- ✅ 安全保护 (电压/温度监控)
- ✅ 参数化配置
- ✅ 完整的测试工具

## 硬件要求

- **舵机**: HTD-85H 智能总线舵机 x2
- **控制板**: 倒置安装在头部下方
- **接口**: Type-C 连接到 Jetson Nano
- **串口**: `/dev/ttyTHS1` (默认)
- **波特率**: 115200

## 依赖项

```bash
# ROS2 Humble
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs

# Python依赖
pip3 install pyserial
```

## 首次使用 - 舵机ID配置

**重要**: 使用前必须为两个舵机设置不同的ID！

```bash
# 1. 单独连接第一个舵机（左右转动）
cd ~/qyh_jushen_ws/资料/头部电机/06\ Jetson\ Nano版本/程序文件/案例2\ 总线舵机ID设置/
python3 set_serial_servo_status.py
# 设置为 ID=1

# 2. 断开第一个舵机，连接第二个舵机（上下转动）
python3 set_serial_servo_status.py
# 设置为 ID=2
```

## 编译

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_head_control
source install/setup.bash
```

## 使用方法

### 1. 启动控制节点

```bash
# 使用默认参数
ros2 launch qyh_head_control head_control.launch.py

# 自定义串口
ros2 launch qyh_head_control head_control.launch.py serial_port:=/dev/ttyUSB0

# 自定义波特率
ros2 launch qyh_head_control head_control.launch.py baudrate:=9600
```

### 2. 运行测试

```bash
ros2 run qyh_head_control test_head_servo
```

测试菜单：
- 1 - 位置控制测试 (0-1000)
- 2 - 归一化控制测试 (-1.0 到 1.0)
- 3 - 目标跟踪测试
- 4 - 扫描模式测试
- 5 - 运行所有测试

### 3. 手动控制

#### 位置控制 (0-1000，500为中位)

```bash
# 左右转动
ros2 topic pub /head/pan_position std_msgs/Float64 "{data: 700.0}"

# 上下转动
ros2 topic pub /head/tilt_position std_msgs/Float64 "{data: 300.0}"

# 归中
ros2 topic pub /head/pan_position std_msgs/Float64 "{data: 500.0}"
ros2 topic pub /head/tilt_position std_msgs/Float64 "{data: 500.0}"
```

#### 归一化控制 (-1.0 到 1.0)

```bash
# 左右转动
ros2 topic pub /head/pan_normalized std_msgs/Float64 "{data: 0.5}"  # 右
ros2 topic pub /head/pan_normalized std_msgs/Float64 "{data: -0.5}" # 左

# 上下转动
ros2 topic pub /head/tilt_normalized std_msgs/Float64 "{data: 0.5}"  # 上
ros2 topic pub /head/tilt_normalized std_msgs/Float64 "{data: -0.5}" # 下
```

#### 目标跟踪控制

```bash
# 发布目标点 (x: 水平, y: 垂直)
ros2 topic pub /head/target_point geometry_msgs/Point "{x: 0.5, y: 0.3, z: 0.0}"
```

### 4. 查看状态

```bash
# 查看关节状态
ros2 topic echo /head/joint_states

# 查看节点信息
ros2 node info /head_servo_node

# 查看所有话题
ros2 topic list
```

## 话题接口

### 订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/head/pan_position` | `std_msgs/Float64` | Pan位置控制 (0-1000) |
| `/head/tilt_position` | `std_msgs/Float64` | Tilt位置控制 (0-1000) |
| `/head/pan_normalized` | `std_msgs/Float64` | Pan归一化控制 (-1.0~1.0) |
| `/head/tilt_normalized` | `std_msgs/Float64` | Tilt归一化控制 (-1.0~1.0) |
| `/head/target_point` | `geometry_msgs/Point` | 目标点跟踪 |

### 发布的话题

| 话题 | 类型 | 频率 | 描述 |
|------|------|------|------|
| `/head/joint_states` | `sensor_msgs/JointState` | 20Hz | 关节状态 (弧度) |

## 参数配置

配置文件: `config/head_params.yaml`

```yaml
head_servo_node:
  ros__parameters:
    # 串口配置
    serial_port: "/dev/ttyTHS1"
    baudrate: 115200
    
    # 舵机ID
    pan_servo_id: 1
    tilt_servo_id: 2
    
    # 控制参数
    update_rate: 20.0          # 状态发布频率 (Hz)
    default_duration: 100      # 运动时间 (ms)
    
    # Pan舵机范围
    pan_min_position: 100      # 最左
    pan_max_position: 900      # 最右
    pan_center_position: 500
    
    # Tilt舵机范围
    tilt_min_position: 200     # 最下
    tilt_max_position: 800     # 最上
    tilt_center_position: 500
    
    # 安全参数
    voltage_min: 6000          # 最低电压 (mV)
    voltage_max: 12000         # 最高电压 (mV)
    temp_max: 65               # 最高温度 (℃)
    enable_safety_check: true
```

## Python API示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class HeadController(Node):
    def __init__(self):
        super().__init__('head_controller')
        
        # 创建发布器
        self.pan_pub = self.create_publisher(Float64, 'head/pan_position', 10)
        self.tilt_pub = self.create_publisher(Float64, 'head/tilt_position', 10)
        self.target_pub = self.create_publisher(Point, 'head/target_point', 10)
    
    def move_to(self, pan, tilt):
        """移动到指定位置"""
        self.pan_pub.publish(Float64(data=float(pan)))
        self.tilt_pub.publish(Float64(data=float(tilt)))
    
    def track_target(self, x, y):
        """跟踪目标点"""
        point = Point()
        point.x = x
        point.y = y
        self.target_pub.publish(point)

# 使用示例
rclpy.init()
controller = HeadController()
controller.move_to(600, 400)  # 右上方
controller.track_target(0.5, 0.3)  # 跟踪目标
rclpy.shutdown()
```

## 常见问题

### Q1: 串口权限错误

```bash
sudo chmod 666 /dev/ttyTHS1
# 或永久解决
sudo usermod -aG dialout $USER
# 重新登录
```

### Q2: 找不到舵机

**原因**: ID未设置或冲突

**解决**: 参考"首次使用 - 舵机ID配置"章节

### Q3: 舵机抖动

**解决**:
- 检查供电电压 (6-12V)
- 增加 `default_duration` 参数
- 降低控制频率

### Q4: 导入SDK失败

```bash
# 检查SDK文件是否存在
ls qyh_head_control/sdk/

# 确保已编译安装
colcon build --packages-select qyh_head_control
source install/setup.bash
```

## 集成示例

### 与视觉系统集成

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionHeadController(Node):
    def __init__(self):
        super().__init__('vision_head')
        self.bridge = CvBridge()
        
        # 订阅相机
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        
        # 发布头部控制
        self.target_pub = self.create_publisher(Point, 'head/target_point', 10)
    
    def image_cb(self, msg):
        # 检测目标
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        target_x, target_y = self.detect_face(cv_image)
        
        if target_x is not None:
            # 转换为归一化坐标
            h, w = cv_image.shape[:2]
            norm_x = (target_x - w/2) / (w/2)
            norm_y = (target_y - h/2) / (h/2)
            
            # 发布目标
            point = Point()
            point.x = norm_x
            point.y = norm_y
            self.target_pub.publish(point)
```

## 维护者

qintxwd

## 许可证

MIT
