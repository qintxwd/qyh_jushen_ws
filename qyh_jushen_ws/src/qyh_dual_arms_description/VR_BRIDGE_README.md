# VR数据桥接到base_vr坐标系

## 📁 文件说明

### scripts/vr_to_base_vr.py
Python脚本，接收PICO 4 VR的UDP数据并发布TF变换。

**功能：**
- 监听UDP端口（默认9999）接收VR数据
- 解析VR手柄位置和姿态（参考C++代码的数据包结构）
- 在`base_vr`坐标系下发布两个TF frame：
  - `base_vr -> vr_left` (左手柄)
  - `base_vr -> vr_right` (右手柄)

**关键设计：**
VR手柄的原始数据（X右 Y上 Z后）直接对应`base_vr`坐标系，无需坐标转换。

---

## 🚀 使用方法

### 1. 构建包
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select dual_arms
source install/setup.bash
```

### 2. 启动VR桥接
```bash
# 方法1: 使用launch文件（推荐）
ros2 launch dual_arms vr_bridge.launch.py

# 方法2: 直接运行节点
ros2 run dual_arms vr_to_base_vr.py

# 方法3: 自定义UDP端口
ros2 launch dual_arms vr_bridge.launch.py udp_port:=8888
```

### 3. 验证TF是否发布
```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 查看base_vr到vr_left的变换
ros2 run tf2_ros tf2_echo base_vr vr_left

# 查看base_vr到vr_right的变换
ros2 run tf2_ros tf2_echo base_vr vr_right

# 查看vr_left到机械臂末端的变换
ros2 run tf2_ros tf2_echo vr_left rt

# 查看vr_right到机械臂末端的变换
ros2 run tf2_ros tf2_echo vr_right lt
```

### 4. 可视化
```bash
# 启动RViz并显示机器人模型
ros2 launch dual_arms display.launch.py

# 在RViz中添加TF显示：
# Add -> TF -> 勾选 "Show Names" 和 "Show Axes"
# 可以看到 base_vr -> vr_left/vr_right 的变换
```

---

## 📊 TF树结构

```
base_link
    └── base_vr (rpy=1.5708 0 -1.5708)
            ├── vr_left  (VR左手柄，动态)
            └── vr_right (VR右手柄，动态)

rt (右臂末端)
lt (左臂末端)
```

**说明：**
- `base_link` 是机器人基座（标准ROS坐标系：X前 Y左 Z上）
- `base_vr` 是VR参考坐标系（X右 Y上 Z后，适合VR操作）
- `vr_left/vr_right` 是VR手柄的实时位置，在`base_vr`坐标系下
- 通过TF2可以自动计算任意frame之间的转换

---

## 🎮 VR数据流

```
PICO 4 VR设备
    ↓ (UDP数据包，端口9999)
vr_to_base_vr.py节点
    ↓ (解析数据)
发布TF: base_vr -> vr_left
发布TF: base_vr -> vr_right
    ↓ (TF2自动计算)
任意坐标系之间的转换
    例如: vr_left -> rt (左手柄到右臂末端)
```

---

## 🔧 参数配置

在launch文件或命令行中可以配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `udp_port` | 9999 | UDP监听端口 |
| `publish_rate` | 100.0 | TF发布频率（Hz） |

**示例：**
```bash
ros2 launch dual_arms vr_bridge.launch.py udp_port:=8888 publish_rate:=120.0
```

---

## 📝 数据包格式

Python脚本接收的UDP数据包与C++节点完全一致：

```python
# 数据包结构（150 bytes）
timestamp (int64)           # 8 bytes
head_position (3 float)     # 12 bytes
head_orientation (4 float)  # 16 bytes
left_active (uint8)         # 1 byte
left_position (3 float)     # 12 bytes
left_orientation (4 float)  # 16 bytes
left_joystick (2 float)     # 8 bytes
left_trigger (float)        # 4 bytes
left_grip (float)           # 4 bytes
right_active (uint8)        # 1 byte
right_position (3 float)    # 12 bytes
right_orientation (4 float) # 16 bytes
right_joystick (2 float)    # 8 bytes
right_trigger (float)       # 4 bytes
right_grip (float)          # 4 bytes
buttons_bitmask (uint32)    # 4 bytes
touches_bitmask (uint32)    # 4 bytes
```

---

## 🔍 调试

### 查看接收到的数据
```bash
# 查看发布的PoseStamped消息
ros2 topic echo /vr_base_vr/left_pose
ros2 topic echo /vr_base_vr/right_pose

# 查看话题列表
ros2 topic list | grep vr
```

### 检查TF是否正常
```bash
# 查看所有TF frame
ros2 run tf2_tools view_frames
evince frames.pdf

# 实时监控TF
ros2 run tf2_ros tf2_monitor
```

### 常见问题

**1. 节点启动但没有收到数据**
- 检查VR设备是否正在发送数据
- 确认UDP端口是否正确
- 检查防火墙设置

**2. TF变换不更新**
- 确认VR手柄是否激活（active字段）
- 检查节点是否正常运行：`ros2 node list`

**3. 坐标不对**
- `base_vr`坐标系是X右Y上Z后
- VR数据直接映射到这个坐标系，不需要转换
- 如果要转换到机械臂坐标系，使用TF2查询

---

## 💡 使用示例

### 示例1: 获取VR左手柄到右臂末端的变换

```python
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class VRControlExample(Node):
    def __init__(self):
        super().__init__('vr_control_example')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.control_callback)
    
    def control_callback(self):
        try:
            # 查询VR左手柄到右臂末端的变换
            transform = self.tf_buffer.lookup_transform(
                'rt',        # 目标：右臂末端
                'vr_left',   # 源：VR左手柄
                rclpy.time.Time()
            )
            
            # 现在可以用这个变换来控制机械臂
            # ...
            
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')

def main():
    rclpy.init()
    node = VRControlExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 示例2: 监听VR手柄位置

```bash
# 订阅左手柄位置
ros2 topic echo /vr_base_vr/left_pose

# 订阅右手柄位置  
ros2 topic echo /vr_base_vr/right_pose
```

---

## ✅ 优势

1. **简洁**：VR数据直接对应`base_vr`坐标系，无需复杂转换
2. **灵活**：通过TF2可以轻松查询到任意frame的转换
3. **标准**：符合ROS2的TF2最佳实践
4. **独立**：Python实现，易于修改和调试

---

## 🔗 相关文档

- [VR_COORDINATE_SYSTEM.md](VR_COORDINATE_SYSTEM.md) - VR坐标系详细说明
- [SETUP_COMPLETE.md](SETUP_COMPLETE.md) - base_vr设置总结
