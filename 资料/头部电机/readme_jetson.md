# HTD-85H 智能总线舵机 Jetson ROS 开发指南

## 概述

本指南提供在 Jetson Nano 平台下使用 HTD-85H 智能总线舵机开发 ROS 包的完整说明，适用于机器人头部双自由度（左右/上下）运动控制。

### 硬件连接
- **舵机控制板**：倒置安装在头部下方
- **接口**：控制板右侧 Type-C 接口连接到 Jetson Nano
- **通信方式**：串口通信（默认 `/dev/ttyTHS1`，波特率 115200）
- **舵机配置**：两个舵机分别控制左右转动和上下转动

---

## 1. SDK 文件结构

### 核心库文件位置
```
1 教程资料/2.总线舵机二次开发教程/06 Jetson Nano版本/程序文件/所需库文件/sdk/
├── hiwonder_servo_controller.py  # 舵机控制器核心类
├── hiwonder_servo_cmd.py         # 命令定义常量
├── __init__.py
└── 其他外设库文件...
```

### 示例代码位置
```
1 教程资料/2.总线舵机二次开发教程/06 Jetson Nano版本/程序文件/
├── 案例1 获取总线舵机信息/       # 读取舵机状态
├── 案例2 总线舵机ID设置/         # 配置舵机ID
├── 案例3 控制总线舵机转动/       # 基本运动控制
├── 案例4 调节总线舵机速度/       # 速度控制
└── 案例5 示教记录实现/           # 高级功能
```

---

## 2. 快速开始

### 2.1 环境准备

#### 安装依赖
```bash
# Jetson GPIO 库
sudo pip3 install Jetson.GPIO

# pyserial 串口库
sudo pip3 install pyserial

# ROS 相关（假设已安装 ROS Melodic/Noetic）
sudo apt-get install ros-${ROS_DISTRO}-serial
```

#### 串口权限配置
```bash
# 将用户添加到 dialout 组
sudo usermod -aG dialout $USER

# 重新登录生效，或临时授权
sudo chmod 666 /dev/ttyTHS1
```

### 2.2 SDK 库文件拷贝

将 SDK 文件夹拷贝到你的 ROS 工作空间：
```bash
cd ~/catkin_ws/src/your_robot_head_pkg/
mkdir -p scripts/sdk
cp -r "1 教程资料/2.总线舵机二次开发教程/06 Jetson Nano版本/程序文件/所需库文件/sdk/"* scripts/sdk/
```

---

## 3. 核心 API 使用说明

### 3.1 初始化控制器

```python
#!/usr/bin/env python3
import sys
sys.path.append("path/to/sdk")
from sdk import hiwonder_servo_controller

# 创建控制器实例
servo_control = hiwonder_servo_controller.HiwonderServoController(
    port='/dev/ttyTHS1',  # Jetson Nano 串口
    baudrate=115200       # 波特率
)
```

### 3.2 基本控制函数

#### 位置控制（最常用）
```python
servo_control.set_servo_position(
    servo_id=1,      # 舵机ID (0-253)
    position=500,    # 目标位置 (0-1000, 500为中位)
    duration=500     # 运动时间 (毫秒, 范围20-30000)
)
```

**参数说明：**
- `servo_id`：舵机ID，出厂默认为1，需为不同舵机设置不同ID
- `position`：目标位置（0-1000），500为中位
- `duration`：运动时间（毫秒），控制速度，时间越短速度越快

#### 停止运动
```python
servo_control.stop(servo_id=1)
```

#### 读取当前位置
```python
position = servo_control.get_servo_position(servo_id=1)
print(f"当前位置: {position}")  # 返回值 0-1000
```

### 3.3 舵机配置函数

#### 设置舵机ID（重要！）
```python
# 首次使用前，分别设置两个舵机的ID
# 注意：每次只能连接一个舵机进行ID设置
servo_control.set_servo_id(oldid=1, newid=1)  # 左右转动舵机
servo_control.set_servo_id(oldid=1, newid=2)  # 上下转动舵机
```

#### 设置运动范围限制
```python
servo_control.set_servo_range(
    servo_id=1,
    low=100,    # 最小位置
    high=900    # 最大位置
)
```

#### 读取舵机信息
```python
# 读取舵机ID
servo_id = servo_control.get_servo_id()

# 读取当前位置
pos = servo_control.get_servo_position(servo_id)

# 读取电压 (单位: mV)
voltage = servo_control.get_servo_vin(servo_id)

# 读取温度 (单位: ℃)
temp = servo_control.get_servo_temp(servo_id)

# 读取运动范围
angle_range = servo_control.get_servo_range(servo_id)  # 返回元组 (low, high)
```

### 3.4 高级功能

#### 偏差调整
```python
# 设置偏差 (-125 到 125)
servo_control.set_servo_deviation(servo_id=1, dev=10)

# 保存偏差（掉电保护）
servo_control.save_servo_deviation(servo_id=1)
```

#### 卸载/锁定舵机
```python
# 卸载舵机（自由转动）
servo_control.unload_servo(servo_id=1, status=0)

# 锁定舵机
servo_control.unload_servo(servo_id=1, status=1)
```

---

## 4. ROS 包开发示例

### 4.1 包结构
```
your_robot_head_pkg/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── head_control.launch
├── scripts/
│   ├── sdk/                           # SDK 库文件夹
│   │   ├── hiwonder_servo_controller.py
│   │   ├── hiwonder_servo_cmd.py
│   │   └── __init__.py
│   └── head_servo_node.py            # ROS 节点
└── config/
    └── head_params.yaml
```

### 4.2 ROS 节点实现

创建 `scripts/head_servo_node.py`：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import sys
import os

# 添加 SDK 路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, 'sdk'))

from sdk import hiwonder_servo_controller

class HeadServoController:
    def __init__(self):
        rospy.init_node('head_servo_node', anonymous=False)
        
        # 参数配置
        self.port = rospy.get_param('~serial_port', '/dev/ttyTHS1')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.pan_servo_id = rospy.get_param('~pan_servo_id', 1)   # 左右转动舵机ID
        self.tilt_servo_id = rospy.get_param('~tilt_servo_id', 2) # 上下转动舵机ID
        self.update_rate = rospy.get_param('~update_rate', 20)    # Hz
        
        # 初始化舵机控制器
        try:
            self.servo_control = hiwonder_servo_controller.HiwonderServoController(
                self.port, self.baudrate
            )
            rospy.loginfo(f"舵机控制器初始化成功: {self.port}@{self.baudrate}")
        except Exception as e:
            rospy.logerr(f"舵机控制器初始化失败: {e}")
            sys.exit(1)
        
        # 订阅话题
        rospy.Subscriber('/head/pan_position', Float64, self.pan_callback)
        rospy.Subscriber('/head/tilt_position', Float64, self.tilt_callback)
        
        # 发布话题
        self.joint_state_pub = rospy.Publisher('/head/joint_states', JointState, queue_size=10)
        
        # 当前状态
        self.current_pan = 500
        self.current_tilt = 500
        
        rospy.loginfo("头部舵机控制节点已启动")
        
    def pan_callback(self, msg):
        """左右转动控制回调"""
        position = int(msg.data)
        position = max(0, min(1000, position))  # 限制范围
        try:
            self.servo_control.set_servo_position(
                self.pan_servo_id, 
                position, 
                duration=100  # 快速响应
            )
            self.current_pan = position
        except Exception as e:
            rospy.logerr(f"Pan舵机控制错误: {e}")
    
    def tilt_callback(self, msg):
        """上下转动控制回调"""
        position = int(msg.data)
        position = max(0, min(1000, position))
        try:
            self.servo_control.set_servo_position(
                self.tilt_servo_id, 
                position, 
                duration=100
            )
            self.current_tilt = position
        except Exception as e:
            rospy.logerr(f"Tilt舵机控制错误: {e}")
    
    def publish_joint_states(self):
        """发布关节状态"""
        try:
            # 读取实际位置
            pan_pos = self.servo_control.get_servo_position(self.pan_servo_id)
            tilt_pos = self.servo_control.get_servo_position(self.tilt_servo_id)
            
            # 转换为弧度（0-1000 映射到 -π/2 到 π/2）
            pan_angle = (pan_pos - 500) / 500.0 * 1.57  # π/2
            tilt_angle = (tilt_pos - 500) / 500.0 * 1.57
            
            # 构建消息
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['head_pan_joint', 'head_tilt_joint']
            joint_state.position = [pan_angle, tilt_angle]
            
            self.joint_state_pub.publish(joint_state)
        except Exception as e:
            rospy.logwarn(f"读取关节状态失败: {e}")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.publish_joint_states()
            rate.sleep()
        
        # 关闭时回到中位
        try:
            self.servo_control.set_servo_position(self.pan_servo_id, 500, 500)
            self.servo_control.set_servo_position(self.tilt_servo_id, 500, 500)
        except:
            pass

if __name__ == '__main__':
    try:
        controller = HeadServoController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```

### 4.3 Launch 文件

创建 `launch/head_control.launch`：

```xml
<?xml version="1.0"?>
<launch>
    <!-- 头部舵机控制节点 -->
    <node name="head_servo_node" pkg="your_robot_head_pkg" type="head_servo_node.py" output="screen">
        <!-- 串口配置 -->
        <param name="serial_port" value="/dev/ttyTHS1"/>
        <param name="baudrate" value="115200"/>
        
        <!-- 舵机ID配置 -->
        <param name="pan_servo_id" value="1"/>   <!-- 左右转动 -->
        <param name="tilt_servo_id" value="2"/>  <!-- 上下转动 -->
        
        <!-- 更新频率 -->
        <param name="update_rate" value="20"/>
    </node>
    
    <!-- 可选：启动rviz可视化 -->
    <!-- <node name="rviz" pkg="rviz" type="rviz" args="-d $(find your_robot_head_pkg)/config/head.rviz"/> -->
</launch>
```

### 4.4 参数配置文件

创建 `config/head_params.yaml`：

```yaml
# 舵机范围限制
pan_servo:
  id: 1
  min_position: 100   # 最小位置（左极限）
  max_position: 900   # 最大位置（右极限）
  center_position: 500
  
tilt_servo:
  id: 2
  min_position: 200   # 最小位置（下极限）
  max_position: 800   # 最大位置（上极限）
  center_position: 500

# 运动参数
motion:
  default_duration: 200  # 默认运动时间（ms）
  smooth_duration: 500   # 平滑运动时间（ms）
  
# 安全参数
safety:
  voltage_min: 6000   # 最低电压 (mV)
  voltage_max: 12000  # 最高电压 (mV)
  temp_max: 65        # 最高温度 (℃)
```

### 4.5 CMakeLists.txt 配置

在 `CMakeLists.txt` 中添加：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(your_robot_head_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/head_servo_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装 config 文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
```

### 4.6 package.xml 配置

```xml
<?xml version="1.0"?>
<package format="2">
  <name>your_robot_head_pkg</name>
  <version>0.1.0</version>
  <description>HTD-85H servo controller for robot head</description>
  
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  
  <exec_depend>python3-serial</exec_depend>
</package>
```

---

## 5. 使用流程

### 5.1 首次使用配置

1. **单独连接舵机设置ID**（重要！）
   ```bash
   # 先连接第一个舵机（左右转动）
   cd ~/catkin_ws/src/your_robot_head_pkg/scripts
   python3 -c "
   import sys
   sys.path.append('sdk')
   from sdk import hiwonder_servo_controller
   servo = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)
   servo.set_servo_id(1, 1)  # 设置为ID=1
   print('Pan舵机ID已设置为1')
   "
   
   # 断开第一个舵机，连接第二个舵机（上下转动）
   python3 -c "
   import sys
   sys.path.append('sdk')
   from sdk import hiwonder_servo_controller
   servo = hiwonder_servo_controller.HiwonderServoController('/dev/ttyTHS1', 115200)
   servo.set_servo_id(1, 2)  # 设置为ID=2
   print('Tilt舵机ID已设置为2')
   "
   ```

2. **测试舵机连接**
   ```bash
   # 使用SDK提供的测试脚本
   cd "1 教程资料/2.总线舵机二次开发教程/06 Jetson Nano版本/程序文件/案例1 获取总线舵机信息"
   python3 serial_servo_status.py
   ```

### 5.2 编译和运行

```bash
# 编译
cd ~/catkin_ws
catkin_make

# 添加到环境
source devel/setup.bash

# 启动节点
roslaunch your_robot_head_pkg head_control.launch
```

### 5.3 控制测试

**方式1：命令行发布**
```bash
# 控制左右转动（0-1000）
rostopic pub /head/pan_position std_msgs/Float64 "data: 700.0"

# 控制上下转动
rostopic pub /head/tilt_position std_msgs/Float64 "data: 300.0"

# 回到中位
rostopic pub /head/pan_position std_msgs/Float64 "data: 500.0"
rostopic pub /head/tilt_position std_msgs/Float64 "data: 500.0"
```

**方式2：Python 脚本控制**
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

rospy.init_node('head_test')
pan_pub = rospy.Publisher('/head/pan_position', Float64, queue_size=10)
tilt_pub = rospy.Publisher('/head/tilt_position', Float64, queue_size=10)

rospy.sleep(1)

# 左右扫描
for pos in range(200, 800, 100):
    pan_pub.publish(Float64(pos))
    rospy.sleep(0.5)

# 上下扫描
for pos in range(300, 700, 100):
    tilt_pub.publish(Float64(pos))
    rospy.sleep(0.5)
```

---

## 6. 实用技巧

### 6.1 坐标映射

将角度（-90° 到 90°）映射到位置（0-1000）：
```python
def angle_to_position(angle_deg):
    """角度转位置"""
    # angle_deg: -90 到 90
    return int(500 + (angle_deg / 90.0) * 500)

def position_to_angle(position):
    """位置转角度"""
    # position: 0 到 1000
    return (position - 500) / 500.0 * 90
```

### 6.2 速度控制

通过调整 `duration` 参数控制速度：
```python
# 慢速平滑运动
servo_control.set_servo_position(servo_id=1, position=800, duration=1000)

# 快速响应
servo_control.set_servo_position(servo_id=1, position=800, duration=100)

# 极速运动（最小值）
servo_control.set_servo_position(servo_id=1, position=800, duration=20)
```

### 6.3 安全保护

```python
def safe_move(servo_id, target_position, duration=200):
    """带安全检查的运动"""
    # 检查电压
    voltage = servo_control.get_servo_vin(servo_id)
    if voltage < 6000 or voltage > 12000:
        rospy.logwarn(f"电压异常: {voltage}mV")
        return False
    
    # 检查温度
    temp = servo_control.get_servo_temp(servo_id)
    if temp > 65:
        rospy.logerr(f"温度过高: {temp}℃")
        return False
    
    # 执行运动
    servo_control.set_servo_position(servo_id, target_position, duration)
    return True
```

### 6.4 平滑轨迹

```python
import numpy as np

def smooth_trajectory(start_pos, end_pos, steps=10):
    """生成平滑轨迹"""
    positions = np.linspace(start_pos, end_pos, steps)
    for pos in positions:
        servo_control.set_servo_position(servo_id=1, position=int(pos), duration=50)
        rospy.sleep(0.05)
```

---

## 7. 常见问题

### Q1: 串口权限问题
**错误**: `PermissionError: [Errno 13] Permission denied: '/dev/ttyTHS1'`

**解决**:
```bash
sudo chmod 666 /dev/ttyTHS1
# 或永久解决
sudo usermod -aG dialout $USER
# 重新登录
```

### Q2: 找不到舵机
**原因**: ID设置错误或多个舵机ID冲突

**解决**:
- 单独连接每个舵机设置不同的ID
- 使用 ID=254 广播命令查询舵机

### Q3: 舵机抖动
**原因**: 
- 电源不足
- duration 参数过小
- 发送指令频率过高

**解决**:
- 检查供电电压（6-12V）
- 增加 duration 参数至200ms以上
- 降低指令发送频率

### Q4: 通信失败
**错误**: `DroppedPacketError`

**解决**:
- 检查串口连接
- 确认波特率为 115200
- 检查 GPIO 引脚配置（TX_CON=GPIO27, RX_CON=GPIO17）

### Q5: 舵机卡死
**现象**: 发送指令无响应

**解决**:
```python
# 卸载舵机
servo_control.unload_servo(servo_id=1, status=0)
rospy.sleep(1)

# 重新锁定
servo_control.unload_servo(servo_id=1, status=1)
```

---

## 8. 高级应用

### 8.1 视觉跟踪联动

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisionTracker:
    def __init__(self):
        rospy.init_node('vision_tracker')
        self.bridge = CvBridge()
        
        # 订阅摄像头图像
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # 发布舵机控制
        self.pan_pub = rospy.Publisher('/head/pan_position', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('/head/tilt_position', Float64, queue_size=10)
        
        # PID 控制参数
        self.kp = 0.5
        self.pan_pos = 500
        self.tilt_pos = 500
    
    def image_callback(self, msg):
        # 转换图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = cv_image.shape[:2]
        
        # 检测目标（示例：使用颜色检测）
        # ... 目标检测代码 ...
        target_x, target_y = self.detect_target(cv_image)
        
        if target_x is not None:
            # 计算误差
            error_x = (target_x - w/2) / (w/2)  # -1 到 1
            error_y = (target_y - h/2) / (h/2)
            
            # 更新舵机位置
            self.pan_pos -= error_x * 100 * self.kp
            self.tilt_pos += error_y * 100 * self.kp
            
            # 限制范围
            self.pan_pos = max(100, min(900, self.pan_pos))
            self.tilt_pos = max(200, min(800, self.tilt_pos))
            
            # 发布控制指令
            self.pan_pub.publish(Float64(self.pan_pos))
            self.tilt_pub.publish(Float64(self.tilt_pos))
```

### 8.2 动作序列播放

```python
class ActionSequence:
    def __init__(self, servo_control):
        self.servo_control = servo_control
        self.sequences = {}
    
    def record_action(self, name):
        """记录动作序列"""
        positions = []
        print(f"记录动作: {name} (按Ctrl+C结束)")
        try:
            while True:
                pan_pos = self.servo_control.get_servo_position(1)
                tilt_pos = self.servo_control.get_servo_position(2)
                positions.append((pan_pos, tilt_pos))
                rospy.sleep(0.1)
        except KeyboardInterrupt:
            self.sequences[name] = positions
            print(f"动作已保存，共{len(positions)}帧")
    
    def play_action(self, name, speed=1.0):
        """播放动作序列"""
        if name not in self.sequences:
            rospy.logwarn(f"动作 {name} 不存在")
            return
        
        positions = self.sequences[name]
        duration = int(100 / speed)
        
        for pan_pos, tilt_pos in positions:
            self.servo_control.set_servo_position(1, pan_pos, duration)
            self.servo_control.set_servo_position(2, tilt_pos, duration)
            rospy.sleep(duration / 1000.0)
```

---

## 9. 参考资料

### SDK 文档位置
- **通信协议**: `3 相关手册及图纸/02 总线舵机通信协议/`
- **调试软件**: `2 软件工具/08 舵机调试板软件/BusLinker调试板软件/`
- **规格参数**: `3 相关手册及图纸/05 总线舵机规格参数/`

### 命令常量定义
参考 `sdk/hiwonder_servo_cmd.py`：
- `HIWONDER_SERVO_MOVE_TIME_WRITE = 1`: 位置控制
- `HIWONDER_SERVO_MOVE_STOP = 12`: 停止运动
- `HIWONDER_SERVO_ID_WRITE = 13`: 设置ID
- `HIWONDER_SERVO_POS_READ = 28`: 读取位置
- `HIWONDER_SERVO_VIN_READ = 27`: 读取电压
- `HIWONDER_SERVO_TEMP_READ = 26`: 读取温度

### 舵机参数范围
- **位置**: 0-1000 (500为中位)
- **时间**: 20-30000 ms
- **ID**: 0-253 (254为广播地址)
- **电压**: 6-12V (6000-12000 mV)
- **温度**: 建议 < 65℃

---

## 10. 总结

通过本指南，你可以：
1. ✅ 理解 HTD-85H 舵机的通信原理
2. ✅ 正确配置 Jetson Nano 串口和 GPIO
3. ✅ 使用 SDK 进行基本控制
4. ✅ 创建完整的 ROS 包实现头部运动控制
5. ✅ 集成视觉系统实现智能跟踪

**关键步骤**：
1. 首次使用必须分别设置两个舵机的ID（1和2）
2. 确保串口权限配置正确
3. 位置值范围 0-1000，500为中位
4. duration 参数控制速度（建议 100-500ms）

**下一步**：
- 集成到完整的机器人系统
- 添加自动标定功能
- 实现更复杂的运动规划
- 集成语音/手势控制

如有问题，请参考 `常见问题解答/` 文件夹或查看示例代码。
