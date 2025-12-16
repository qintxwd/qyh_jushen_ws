# qyh_dual_arm_teleop

双臂VR遥操作系统 - 状态管理与坐标变换

## 📦 包概述

本包包含VR双臂遥操作系统的核心中间层节点，负责：
1. **状态管理**：Clutch机制、零位校准
2. **坐标变换**：VR → 人手语义坐标系
3. **数据处理**：握持补偿、缩放、滤波、速度限制

## 🏗️ 架构位置

```
VR设备 → [vr_bridge_node] → [teleop_manager_node] → [coordinate_mapper_node] → [ik_solver_node] → [arm_controller_node] → 机械臂
         (节点1)              (节点2 - 本包)         (节点3 - 本包)
```

## 📋 节点说明

### 1. `teleop_manager_node` (节点2/5)

**职责：**
- 监听左右手grip按钮
- 管理Clutch状态机：IDLE → ENGAGING → TRACKING → RELEASING
- 发布动态TF：`teleop_base → vr_origin`
- 提供服务：`/teleop/{start, stop, recenter}`

**订阅话题：**
- `/vr/left_controller/joy` (sensor_msgs/Joy)
- `/vr/right_controller/joy` (sensor_msgs/Joy)

**发布TF：**
- `teleop_base → vr_origin` (动态，100Hz)

**服务：**
- `/teleop/start` (std_srvs/Trigger) - 启动遥操作
- `/teleop/stop` (std_srvs/Trigger) - 停止遥操作
- `/teleop/recenter` (std_srvs/Trigger) - 重置零位

**参数：**
```yaml
grip_engage_threshold: 0.8    # Grip接合阈值
grip_release_threshold: 0.2   # Grip释放阈值
update_rate: 100.0            # TF更新频率(Hz)
```

---

### 2. `coordinate_mapper_node` (节点3/5)

**职责：**
- 读取VR controller位姿
- 应用握持补偿、位置缩放、旋转缩放
- 低通滤波、速度限制
- 发布处理后的人手位姿

**订阅TF：**
- `vr_origin → vr_left_controller`
- `vr_origin → vr_right_controller`

**发布TF：**
- `vr_left_controller → human_left_hand`
- `vr_right_controller → human_right_hand`

**发布话题：**
- `/teleop/left_hand/target` (geometry_msgs/PoseStamped)
- `/teleop/right_hand/target` (geometry_msgs/PoseStamped)

**参数：**
```yaml
grip_offset_deg: 35.0         # 握持补偿角度(度)
position_scale: 2.0           # 位置缩放因子
rotation_scale: 1.0           # 旋转缩放因子
filter_alpha: 0.3             # 低通滤波系数
max_position_delta: 0.05      # 单帧最大位移(m)
max_rotation_delta: 0.1       # 单帧最大旋转(rad)
update_rate: 100.0            # 更新频率(Hz)
```

## 🚀 使用方法

### 编译

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_dual_arm_teleop
source install/setup.bash
```

### 启动

**完整系统：**
```bash
# 1. 启动VR数据接收（终端1）
ros2 run qyh_vr_bridge vr_bridge_node

# 2. 启动遥操作系统（终端2）
ros2 launch qyh_dual_arm_teleop teleop.launch.py

# 3. 启动机器人URDF显示（终端3，可选）
ros2 launch qyh_dual_arms_description display.launch.py
```

**单独启动节点：**
```bash
# 节点2: Teleop Manager
ros2 run qyh_dual_arm_teleop teleop_manager_node \
  --ros-args --params-file config/teleop_params.yaml

# 节点3: Coordinate Mapper
ros2 run qyh_dual_arm_teleop coordinate_mapper_node \
  --ros-args --params-file config/teleop_params.yaml
```

### 调用服务

```bash
# 启动遥操作
ros2 service call /teleop/start std_srvs/srv/Trigger

# 停止遥操作
ros2 service call /teleop/stop std_srvs/srv/Trigger

# 重置零位
ros2 service call /teleop/recenter std_srvs/srv/Trigger
```

### 查看TF树

```bash
# 实时查看TF树
ros2 run rqt_tf_tree rqt_tf_tree

# 生成PDF图
ros2 run tf2_tools view_frames

# 查看特定变换
ros2 run tf2_ros tf2_echo teleop_base vr_origin
ros2 run tf2_ros tf2_echo vr_left_controller human_left_hand
```

## 🔧 配置调优

### 位置缩放 (position_scale)

**物理意义：** 机器人工作半径 / VR手臂活动范围

```yaml
position_scale: 2.0  # 默认：手移动0.5米 → 机器人移动1米
```

**调整建议：**
- 太小：机器人移动范围受限
- 太大：机器人移动过于敏感

### 滤波系数 (filter_alpha)

**物理意义：** 新数据权重，越大响应越快但越抖

```yaml
filter_alpha: 0.3    # 推荐范围：0.2-0.5
```

**调整建议：**
- `0.1-0.2`：强滤波，平滑但有延迟
- `0.3-0.4`：平衡（推荐）
- `0.5-0.8`：弱滤波，响应快但可能抖动

### 握持补偿 (grip_offset_deg)

**物理意义：** 手柄自然握持时手指方向与设备坐标的偏差

```yaml
grip_offset_deg: 35.0  # 绕Y轴pitch旋转
```

**标定方法：**
1. 握住手柄，手指自然向前
2. 在RViz中观察 `vr_*_controller` 坐标轴
3. 调整角度使红色X轴（向前）对齐手指方向

## 📊 话题监控

```bash
# 查看VR手柄joy数据
ros2 topic echo /vr/left_controller/joy

# 查看处理后的目标位姿
ros2 topic echo /teleop/left_hand/target

# 查看话题列表
ros2 topic list | grep teleop
```

## 🐛 调试

### 查看日志
```bash
# 实时日志
ros2 run qyh_dual_arm_teleop teleop_manager_node --ros-args --log-level debug

# 使用rqt_console
ros2 run rqt_console rqt_console
```

### 常见问题

**问题1：TF查询失败**
```
Solution: 确保vr_bridge_node正常运行，检查TF树
ros2 run tf2_ros tf2_echo vr_origin vr_left_controller
```

**问题2：Clutch不响应**
```
Solution: 检查joy数据格式和grip阈值
ros2 topic echo /vr/left_controller/joy
调整 grip_engage_threshold 参数
```

**问题3：机器人移动太快/太慢**
```
Solution: 调整 position_scale 参数
太快 → 减小 (如1.5)
太慢 → 增大 (如3.0)
```

## 📐 坐标系关系

```
base_link (机器人中心)
  └── teleop_base (人体参考系，前方0.5m)
      └── vr_origin (VR零位锁定点)
          ├── vr_left_controller (原始VR手柄)
          │   └── human_left_hand (处理后的人手位姿)
          └── vr_right_controller
              └── human_right_hand
```

## 📚 参考文档

- [TF设计文档](../TF_design.md) - 完整坐标系设计
- [ROS2 TF2教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Mobile ALOHA](https://mobile-aloha.github.io/) - VR遥操作参考实现

## 📝 TODO

- [ ] 添加工作空间限制（安全边界）
- [ ] 支持双手协同约束
- [ ] 记录和回放遥操作轨迹
- [ ] 可视化Clutch状态（LED/音效）
- [ ] 自适应滤波参数
