# 伺服控制功能说明

## 概述

伺服控制是JAKA机器人的高级控制模式，已成功添加到`jaka_control`包中。

## 新增文件

### 服务定义 (srv/)
- `ServoMoveEnable.srv` - 使能/关闭伺服模式
- `ServoJ.srv` - 伺服关节空间运动
- `ServoP.srv` - 伺服笛卡尔空间运动  
- `SetServoFilter.srv` - 设置伺服滤波器

### 示例脚本
- `scripts/example_servo_control.py` - 伺服控制Python示例
  - 正弦波关节运动
  - 圆形笛卡尔轨迹
  - 增量运动示例

## 功能特点

### 1. 高频率控制
- 支持100-1000Hz的控制频率
- 低延迟响应（<1ms）
- 适合实时轨迹跟踪

### 2. 多种滤波器
- **无滤波器** (filter_type=0)
  - 最快响应
  - 适合精确位置控制
  - 可能有轻微抖动

- **关节低通滤波器** (filter_type=1)
  - 平衡响应速度和平滑度
  - 可调截止频率
  - 推荐用于一般轨迹跟踪

- **关节非线性滤波器** (filter_type=2)
  - 优化姿态变化
  - 可设置速度/加速度/加加速度限制

- **笛卡尔非线性滤波器** (filter_type=3)
  - 优化末端轨迹
  - 分别限制位置和姿态

### 3. 支持的运动模式
- **绝对模式** (ABS): 运动到指定位置
- **增量模式** (INCR): 相对当前位置增量运动

### 4. EtherCAT同步支持
- `edg_recv()` - 接收同步数据
- `edg_send()` - 发送同步数据
- `edg_get_stat()` - 获取同步状态

## 使用场景

### 适用场景
✅ 复杂轨迹跟踪（圆、螺旋、自由曲线等）
✅ 视觉伺服（实时跟踪目标）
✅ 力控应用（需要快速响应力反馈）
✅ 路径规划（动态避障）
✅ 高精度装配
✅ 画图、焊接等连续轨迹任务

### 不适用场景
❌ 简单点到点运动（用MoveJ/MoveL更简单）
❌ 低频率控制（<10Hz）
❌ 不需要实时性的任务

## API接口

### C++ 接口

```cpp
// 使能伺服模式
bool servoMoveEnable(bool enable, int robot_id);

// 伺服关节运动
bool servoJ(int robot_id, const std::vector<double>& joint_positions, bool move_mode);

// 伺服笛卡尔运动
bool servoP(int robot_id, const geometry_msgs::msg::Pose& target_pose, bool move_mode);

// 设置滤波器
bool servoMoveUseNoneFilter();
bool servoMoveUseJointLPF(double cutoff_freq);
bool servoMoveUseJointNLF(double max_vr, double max_ar, double max_jr);
bool servoMoveUseCarteNLF(double max_vp, double max_ap, double max_jp,
                          double max_vr, double max_ar, double max_jr);

// EtherCAT同步
bool edgRecv(struct timespec *next = nullptr);
bool edgSend();
bool edgGetStat(int robot_id, JointValue& joint_pos, CartesianPose& cartesian_pose);
```

### ROS 2 服务

```bash
# 使能伺服
/jaka_robot_node/servo_move_enable

# 伺服关节运动
/jaka_robot_node/servo_j

# 伺服笛卡尔运动
/jaka_robot_node/servo_p

# 设置滤波器
/jaka_robot_node/set_servo_filter
```

## 使用流程

### 标准流程

```python
# 1. 设置滤波器（在使能前）
set_filter_lpf(125.0)

# 2. 使能伺服模式
servo_move_enable(True, robot_id=0)

# 3. 等待1秒让系统准备好
time.sleep(1.0)

# 4. 循环发送伺服指令
for i in range(1000):
    target = calculate_target(i)
    servo_j(0, target)
    time.sleep(0.01)  # 100Hz

# 5. 关闭伺服模式
servo_move_enable(False, robot_id=0)
```

### 注意事项

⚠️ **重要限制**
1. 使能伺服前必须先设置滤波器
2. 伺服模式下必须周期性发送指令
3. 不要在伺服模式下调用MoveJ/MoveL
4. 退出伺服前应逐渐减速
5. 伺服模式关闭后才能切换到其他模式

⚠️ **性能建议**
1. 控制频率建议：100-500Hz（Python）, 500-1000Hz（C++）
2. Python由于GIL限制，建议使用C++实现高频控制
3. 使用LPF滤波器可以提高稳定性
4. 增量模式比绝对模式计算量更小

## 示例代码

### 正弦波运动

```python
import math
import time

# 使能伺服
client.enable_servo_mode(0)
time.sleep(1.0)

# 正弦波参数
amplitude = 0.3  # 弧度
frequency = 0.5  # Hz
control_rate = 100  # Hz

# 执行10秒
for i in range(10 * control_rate):
    t = i / control_rate
    angle = amplitude * math.sin(2 * math.pi * frequency * t)
    target = [angle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    client.servo_j(0, target)
    time.sleep(1.0 / control_rate)

# 关闭伺服
client.disable_servo_mode(0)
```

### 圆形轨迹

```python
# 使能伺服
client.enable_servo_mode(0)
time.sleep(1.0)

# 圆形参数
center = [0.5, 0.0, 0.3]  # 米
radius = 0.05  # 米
angular_vel = 0.5  # rad/s

# 执行一圈
duration = 2 * math.pi / angular_vel
control_rate = 100

for i in range(int(duration * control_rate)):
    t = i / control_rate
    angle = angular_vel * t
    
    pose = Pose()
    pose.position.x = center[0] + radius * math.cos(angle)
    pose.position.y = center[1] + radius * math.sin(angle)
    pose.position.z = center[2]
    pose.orientation.w = 1.0
    
    client.servo_p(0, pose)
    time.sleep(1.0 / control_rate)

# 关闭伺服
client.disable_servo_mode(0)
```

## 性能对比

| 特性 | 普通运动(MoveJ/MoveL) | 伺服运动 |
|------|---------------------|---------|
| 控制频率 | 1-10 Hz | 100-1000 Hz |
| 延迟 | 50-100 ms | <1 ms |
| 轨迹精度 | 插补点间距较大 | 插补点间距极小 |
| 使用复杂度 | 简单 | 需要周期调用 |
| CPU占用 | 低 | 中等 |
| 适用场景 | 点到点运动 | 连续轨迹跟踪 |

## 故障排除

### 问题：伺服使能失败
**原因**：
- 机器人未上电或未使能
- 已经在伺服模式下
- 滤波器未设置

**解决**：
```bash
# 1. 确保机器人使能
ros2 service call /jaka_robot_node/enable_robot ...

# 2. 设置滤波器
ros2 service call /jaka_robot_node/set_servo_filter ...

# 3. 重试使能伺服
ros2 service call /jaka_robot_node/servo_move_enable ...
```

### 问题：伺服运动不响应
**原因**：
- 未周期性发送指令
- 控制频率太低
- 目标位置超出限位

**解决**：
- 确保以足够高的频率(>50Hz)发送指令
- 检查目标位置是否合法
- 查看错误日志

### 问题：运动抖动
**原因**：
- 使用了无滤波器模式
- 控制频率不稳定
- 目标位置计算不平滑

**解决**：
- 使用LPF或NLF滤波器
- 确保控制周期稳定
- 优化轨迹规划算法

## 参考资料

- JAKA SDK示例：`30.edgservo.cpp`, `31.edgservo_fct.cpp`
- 本包示例：`scripts/example_servo_control.py`
- API文档：`README.md`

## 版本历史

- v1.0.0 (2025-01-11): 初始版本，添加基本伺服控制功能
  - 支持ServoJ/ServoP
  - 支持4种滤波器
  - 提供Python示例
