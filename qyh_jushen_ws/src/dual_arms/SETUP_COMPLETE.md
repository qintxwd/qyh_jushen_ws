# base_vr 坐标系设置完成总结

## ✅ 已完成的修改

### 1. URDF更新
在 [dual_arms.urdf](urdf/dual_arms.urdf) 中添加了 `base_vr` 坐标系：

```xml
<link name="base_vr">
  <!-- 虚拟link，用于VR坐标系 -->
</link>

<joint name="base_to_vr" type="fixed">
  <origin xyz="0 0 0" rpy="1.5708 0 -1.5708" />
  <parent link="base_link" />
  <child link="base_vr" />
</joint>
```

**坐标系约定**：
- X轴：向右
- Y轴：向上
- Z轴：向后（相对于base_link的观察方向）

### 2. 坐标转换验证
创建了测试脚本 [test_vr_frame.py](test_vr_frame.py)，验证结果：
```
✅ VR X(右) → base_link -Y 方向
✅ VR Y(上) → base_link Z 方向  
✅ VR Z(后) → base_link -X 方向
```

### 3. 示例代码
创建了 [vr_teleoperation_example.py](vr_teleoperation_example.py) 展示：
- 如何订阅VR手柄数据
- 如何使用TF2进行坐标转换
- 如何应用速度限制和缩放

### 4. 文档
创建了 [VR_COORDINATE_SYSTEM.md](VR_COORDINATE_SYSTEM.md) 详细说明使用方法

---

## 🎯 使用思路

### 核心思想
**在base_link下挂载一个符合VR习惯的坐标系base_vr，让VR数据直接在这个坐标系下工作，然后利用ROS2的TF2系统自动转换到机械臂末端坐标系。**

### 工作流程
```
VR手柄原始数据
    ↓
转换为base_vr坐标系的Twist
    ↓
发布到topic (frame_id='base_vr')
    ↓
TF2自动转换到目标坐标系(rt/lt)
    ↓
机械臂控制器执行
```

---

## 🔧 在Linux/WSL环境下构建和测试

### 1. 构建包
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source /opt/ros/humble/setup.bash  # ROS2 Humble
colcon build --packages-select dual_arms
source install/setup.bash
```

### 2. 启动RViz查看坐标系
```bash
ros2 launch dual_arms display.launch.py
```

在RViz中：
1. 点击 `Add` → `TF`
2. 在TF设置中勾选 `Show Names` 和 `Show Axes`
3. 找到 `base_vr` 坐标系（应该在base_link处，但朝向不同）
4. 观察坐标轴颜色：
   - 红色(X)应该指向右
   - 绿色(Y)应该指向上
   - 蓝色(Z)应该指向后

### 3. 查看TF树
```bash
ros2 run tf2_tools view_frames
# 生成frames.pdf，可以看到base_link -> base_vr的连接
```

### 4. 测试坐标转换
```bash
# 查看base_link到base_vr的变换
ros2 run tf2_ros tf2_echo base_link base_vr

# 查看base_vr到右臂末端的变换
ros2 run tf2_ros tf2_echo base_vr rt

# 查看base_vr到左臂末端的变换
ros2 run tf2_ros tf2_echo base_vr lt
```

### 5. 测试VR控制（模拟）
```bash
# 终端1：启动显示
ros2 launch dual_arms display.launch.py

# 终端2：启动VR遥操作节点（如果实现了）
ros2 run dual_arms vr_teleoperation_example.py

# 终端3：发布测试数据（模拟VR手柄向右移动）
ros2 topic pub /vr/right_controller/twist geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_vr'}, \
    twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, \
           angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --once
```

---

## 💡 在你的VR系统中集成

### 方案A：VR数据直接使用base_vr
如果你的VR手柄本身就是 X右/Y上/Z后：

```python
# 在VR数据采集节点中
from geometry_msgs.msg import TwistStamped

def publish_vr_data(vr_controller):
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'base_vr'  # 关键！
    
    # 直接使用VR的值
    msg.twist.linear.x = vr_controller.velocity.x  # 右
    msg.twist.linear.y = vr_controller.velocity.y  # 上
    msg.twist.linear.z = vr_controller.velocity.z  # 后
    
    msg.twist.angular.x = vr_controller.angular.x
    msg.twist.angular.y = vr_controller.angular.y
    msg.twist.angular.z = vr_controller.angular.z
    
    self.vr_pub.publish(msg)
```

### 方案B：VR数据需要映射
如果你的VR坐标系不同，先做映射：

```python
def vr_to_base_vr(vr_data):
    """
    将VR原始坐标映射到base_vr
    
    假设VR是：X向前，Y向右，Z向上
    需要转换为：X向右，Y向上，Z向后
    """
    msg = TwistStamped()
    msg.header.frame_id = 'base_vr'
    
    # X右 = VR的Y右
    msg.twist.linear.x = vr_data.linear.y
    # Y上 = VR的Z上  
    msg.twist.linear.y = vr_data.linear.z
    # Z后 = VR的-X前
    msg.twist.linear.z = -vr_data.linear.x
    
    # 角速度同样处理
    msg.twist.angular.x = vr_data.angular.y
    msg.twist.angular.y = vr_data.angular.z
    msg.twist.angular.z = -vr_data.angular.x
    
    return msg
```

---

## 📊 坐标系关系图

```
        ┌──────────────────┐
        │   base_link      │
        │   (机器人基座)    │
        └────────┬─────────┘
                 │
        fixed joint (rpy=1.5708 0 -1.5708)
                 │
        ┌────────┴─────────┐
        │   base_vr        │
        │   (VR参考系)      │
        │   X右 Y上 Z后     │
        └──────────────────┘
                 │
          (TF2自动计算)
                 │
        ┌────────┴─────────┐
        │   rt / lt        │
        │   (机械臂末端)    │
        └──────────────────┘
```

---

## 🎮 实际使用示例

### 增量位置控制
```python
# VR手柄每次移动产生一个增量
delta_position = vr_controller.get_position_delta()  # [dx, dy, dz]

# 发布到base_vr坐标系
twist_msg = TwistStamped()
twist_msg.header.frame_id = 'base_vr'
twist_msg.twist.linear.x = delta_position[0] * 10.0  # 缩放
twist_msg.twist.linear.y = delta_position[1] * 10.0
twist_msg.twist.linear.z = delta_position[2] * 10.0

# 控制节点自动转换并控制机械臂
```

### 速度控制
```python
# VR手柄持续输出速度
velocity = vr_controller.get_velocity()  # [vx, vy, vz]

twist_msg = TwistStamped()
twist_msg.header.frame_id = 'base_vr'
twist_msg.twist.linear.x = velocity[0]
twist_msg.twist.linear.y = velocity[1]
twist_msg.twist.linear.z = velocity[2]

# 以固定频率发布（如100Hz）
```

---

## ✅ 优势总结

1. **直观性**：VR操作者的动作方向和机械臂运动方向一致
2. **简洁性**：VR端代码只需关注自己的坐标系，不需要了解机器人坐标系
3. **灵活性**：通过TF2可以轻松切换到不同的目标坐标系
4. **可维护性**：坐标关系清晰定义在URDF中，便于调试和修改
5. **标准化**：符合ROS2的最佳实践

---

## 📞 需要帮助？

- 查看详细文档：[VR_COORDINATE_SYSTEM.md](VR_COORDINATE_SYSTEM.md)
- 运行测试脚本：`python3 test_vr_frame.py`
- 参考示例代码：[vr_teleoperation_example.py](vr_teleoperation_example.py)
