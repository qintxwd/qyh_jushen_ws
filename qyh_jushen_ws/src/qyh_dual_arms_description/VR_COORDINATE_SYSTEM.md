# VR遥操作坐标系配置说明

## 📐 坐标系设计

### base_vr 坐标系

在 `dual_arms.urdf` 中添加了 `base_vr` 坐标系，作为VR设备和机器人之间的桥梁。

#### 坐标系定义
相对于 `base_link` 观察方向：
- **X轴**: 向右 (Right)
- **Y轴**: 向上 (Up)  
- **Z轴**: 向后 (Backward)

#### URDF配置
```xml
<link name="base_vr">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="0.001" />
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
  </inertial>
</link>

<joint name="base_to_vr" type="fixed">
  <origin xyz="0 0 0" rpy="1.5708 0 -1.5708" />
  <parent link="base_link" />
  <child link="base_vr" />
</joint>
```

#### 坐标转换关系
```
base_vr X(右) = base_link -Y
base_vr Y(上) = base_link Z
base_vr Z(后) = base_link -X
```

---

## 🎮 VR遥操作使用方法

### 方法1: 直接在base_vr坐标系下工作（推荐）

如果你的VR系统输出的数据符合 X右/Y上/Z后 的约定，可以直接将VR数据发布为 `base_vr` 坐标系下的消息。

```python
# VR数据采集端
vr_twist = TwistStamped()
vr_twist.header.frame_id = 'base_vr'  # 关键：指定坐标系
vr_twist.twist.linear.x = vr_controller.position.x  # 右
vr_twist.twist.linear.y = vr_controller.position.y  # 上
vr_twist.twist.linear.z = vr_controller.position.z  # 后
# ... 发布
```

然后使用TF2自动转换到机械臂末端：
```python
# 机械臂控制端
transform = tf_buffer.lookup_transform(
    'rt',        # 目标：右臂末端
    'base_vr',   # 源：VR坐标系
    rclpy.time.Time()
)
# 应用transform到twist...
```

### 方法2: 使用现有VR坐标后转换

如果VR系统有自己的坐标约定，先转换到 `base_vr` 约定：

```python
def vr_native_to_base_vr(vr_data):
    """
    将VR原生坐标转换到base_vr坐标系
    
    示例：如果VR原生是 X前/Y右/Z上
    需要转换为 X右/Y上/Z后
    """
    base_vr_data = TwistStamped()
    base_vr_data.header.frame_id = 'base_vr'
    
    # X右 = VR的Y右
    base_vr_data.twist.linear.x = vr_data.y
    # Y上 = VR的Z上
    base_vr_data.twist.linear.y = vr_data.z
    # Z后 = VR的-X前
    base_vr_data.twist.linear.z = -vr_data.x
    
    return base_vr_data
```

---

## 🧪 测试和验证

### 1. 可视化TF树
```bash
cd ~/qyh_jushen_ws
colcon build --packages-select dual_arms
source install/setup.bash

# 启动RViz显示
ros2 launch dual_arms display.launch.py

# 在另一个终端查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf
```

在RViz中：
1. 添加 `TF` 显示
2. 勾选 `Show Names`
3. 找到 `base_vr` 坐标系，验证其朝向

### 2. 运行坐标转换测试
```bash
cd ~/qyh_jushen_ws/src/dual_arms
python3 test_vr_frame.py
```

这会验证：
- ✅ VR向右移动 → 机械臂Y负方向
- ✅ VR向上移动 → 机械臂Z正方向  
- ✅ VR向后移动 → 机械臂X负方向

### 3. 运行VR遥操作示例
```bash
# 启动机器人
ros2 launch dual_arms display.launch.py

# 启动VR遥操作节点
ros2 run dual_arms vr_teleoperation_example.py

# 发布测试数据
ros2 topic pub /vr/right_controller/twist geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_vr'}, \
    twist: {linear: {x: 0.1, y: 0.0, z: 0.0}}}"
```

---

## 💡 优势

### ✅ 简化VR集成
- VR数据直接使用直观的坐标约定（X右/Y上/Z后）
- 无需在VR端做复杂的坐标转换

### ✅ 利用TF2自动转换
- 从 `base_vr` 到任意坐标系（`rt`/`lt`/`base_link`等）的转换由TF2自动处理
- 支持动态的机械臂姿态

### ✅ 便于调试
- 可以在RViz中直观看到 `base_vr` 坐标系
- 可以发布测试数据到 `base_vr` 坐标系验证行为

### ✅ 易于维护
- 坐标系关系明确定义在URDF中
- 修改只需要调整URDF中的RPY参数

---

## 📊 坐标系图示

```
        base_link               base_vr
        ┌─────┐                ┌─────┐
        │     │                │     │
        │  Z↑ │                │  Y↑ │
        │  |  │                │  |  │
        │  +─→X               │  +─→X
        │ /   │                │ /   │
        │Y    │                │Z    │
        └─────┘                └─────┘
     (X前Y左Z上)            (X右Y上Z后)
```

---

## 🔧 调整参数

### 修改坐标系位置
如果需要调整 `base_vr` 的位置（例如偏移到操作者位置）：

```xml
<joint name="base_to_vr" type="fixed">
  <origin xyz="0.5 0 1.2" rpy="1.5708 0 -1.5708" />
  <!--     ↑偏移位置  ↑旋转角度 -->
  <parent link="base_link" />
  <child link="base_vr" />
</joint>
```

### 修改速度缩放
在VR遥操作节点中调整：
```python
self.scale_factor = 0.5  # 降低灵敏度
self.max_linear_vel = 0.3  # 限制最大速度
```

---

## 📝 注意事项

1. **增量控制**: 这个设计是为增量控制（速度控制）优化的，如果需要绝对位置控制，需要额外处理
2. **安全限制**: 在实际使用时务必添加工作空间限制、速度限制、奇异点检测等安全机制
3. **手柄朝向**: 如果VR手柄的方向也需要控制机械臂姿态，需要同样处理角速度的转换

---

## 🚀 下一步

1. 集成到现有VR系统
2. 添加碰撞检测
3. 实现双臂协调控制
4. 添加力反馈（如果硬件支持）
