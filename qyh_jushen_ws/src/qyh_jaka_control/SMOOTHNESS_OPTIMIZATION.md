# 机械臂运动平滑度优化指南

## 🎯 优化目标
- 消除微量抖动
- 提高路径平滑度
- 保持合理的响应速度

## 📊 关键参数说明

### 1. **插值权重 (interpolation_weight)** ⭐ 最重要
```
作用：控制每个周期向目标靠近的速度
公式：output = last_output + weight × (target - last_output)

- 0.5 (默认): 每次移动50%，响应快但可能抖动
- 0.3 (推荐): 每次移动30%，平衡
- 0.2: 每次移动20%，非常平滑但响应慢
- 0.1: 超平滑，但延迟明显

调整位置：launch参数 interpolation_weight
```

### 2. **缓冲区大小 (buffer_size)**
```
作用：存储未来指令，提供平滑空间

- 10 (旧默认): 标准
- 16 (新推荐): 更平滑
- 20: 最平滑，但增加延迟

调整位置：launch参数 buffer_size
```

### 3. **速度安全系数 (velocity_safety_factor)**
```
作用：限制最大速度

- 0.8 (旧默认): 80%速度
- 0.65 (新推荐): 65%速度，更平滑
- 0.5: 50%速度，非常平滑但慢

调整位置：launch参数 velocity_safety_factor
```

### 4. **Stale阈值 (stale_threshold_sec)**
```
作用：判断缓存是否过期

当前：0.3秒 (300ms)
- 适应手柄30Hz输入
- 过小可能导致频繁重新同步（跳变）
- 过大可能使用陈旧数据

调整位置：smooth_servo_bridge.cpp 第21行
```

## 🚀 快速优化方案

### 方案A: 超平滑模式（VR/演示用）
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py \
    buffer_size:=20 \
    interpolation_weight:=0.2 \
    velocity_safety_factor:=0.6
```
**效果**: 极度平滑，无抖动，但响应较慢

### 方案B: 平衡模式（推荐，日常操作）
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py \
    buffer_size:=16 \
    interpolation_weight:=0.3 \
    velocity_safety_factor:=0.65
```
**效果**: 平滑且响应合理

### 方案C: 快速响应模式（精细操作）
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py \
    buffer_size:=12 \
    interpolation_weight:=0.4 \
    velocity_safety_factor:=0.75
```
**效果**: 响应快，但可能有轻微抖动

## 🔧 进阶优化

### 1. 添加低通滤波器（需要代码修改）

在 `smooth_servo_bridge.cpp` 的 `interpolate()` 函数后添加低通滤波：

```cpp
std::vector<double> applyLowPassFilter(
    const std::vector<double>& current,
    const std::vector<double>& previous,
    double alpha = 0.3  // 越小越平滑
) {
    std::vector<double> filtered(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        filtered[i] = alpha * current[i] + (1.0 - alpha) * previous[i];
    }
    return filtered;
}
```

### 2. 启用 SimpleJointSmoother（高级平滑）

在 `jaka_control_node.cpp` 中已有 `SimpleJointSmoother` 类，它提供：
- 速度限制
- 加速度限制
- Jerk限制（急动度）
- 低通滤波

可以在 Bridge 的基础上再套一层 Smoother。

### 3. 调整JAKA SDK滤波器

在 `robot_config.yaml` 中设置：

```yaml
# 笛卡尔空间非线性滤波（最强）
servo_filter_type: "carte_nlf"
max_linear_velocity: 400.0      # 降低速度
max_linear_acceleration: 1500.0  # 降低加速度
max_linear_jerk: 8000.0         # 降低急动度

# 或使用关节空间低通滤波
servo_filter_type: "joint_lpf"
joint_lpf_cutoff_freq: 20.0  # 降低截止频率（越低越平滑）
```

## 📈 效果对比

| 场景 | buffer_size | interp_weight | velocity_factor | 延迟 | 平滑度 | 响应性 |
|------|-------------|---------------|-----------------|------|--------|--------|
| **默认** | 10 | 0.5 | 0.8 | 80ms | ★★★☆☆ | ★★★★★ |
| **平衡** | 16 | 0.3 | 0.65 | 128ms | ★★★★☆ | ★★★★☆ |
| **超平滑** | 20 | 0.2 | 0.6 | 160ms | ★★★★★ | ★★★☆☆ |

## 🐛 故障排查

### 抖动仍然存在？

1. **检查输入源**
   ```bash
   ros2 topic hz /jaka/left_bridge/joint_command
   ros2 topic echo /jaka/left_bridge/joint_command --once
   ```
   - 频率是否稳定（30Hz左右）？
   - 数值变化是否过大？

2. **查看Bridge日志**
   ```bash
   ros2 run qyh_jaka_control jaka_control_node --ros-args --log-level debug
   ```
   观察：
   - "Inserted N intermediate points" - 是否频繁插值？
   - "Using current position" - 是否频繁重新同步？

3. **检查机械臂负载**
   - 负载过大可能导致抖动
   - 检查 payload 设置

### 响应太慢？

1. 增加 `interpolation_weight` (0.3 → 0.4)
2. 减小 `buffer_size` (16 → 12)
3. 增加 `velocity_safety_factor` (0.65 → 0.75)

### 运动不连续（有跳变）？

1. 增加 `stale_threshold_sec` (0.3 → 0.5)
2. 确保输入频率稳定
3. 检查是否有命令丢失

## 📝 修改记录

### 当前优化（2025-12-18）

**修改的文件**：
1. `jaka_control_node.cpp`:
   - buffer_size: 10 → 16
   - interpolation_weight: 0.5 → 0.3
   - velocity_safety_factor: 0.8 → 0.65
   - 添加velocity_safety_factor参数支持

2. `smooth_servo_bridge.cpp`:
   - stale_threshold_sec: 0.05 → 0.3
   - 移除current_position传递（Bridge模式）

3. 新增 `config/smooth_params.yaml`:
   - 预设配置模板

**预期改善**：
- ✅ 减少50%的抖动幅度
- ✅ 提高路径平滑度
- ⚠️ 增加约50ms延迟（可接受）

## 🔄 实时调整（无需重启）

可以使用动态参数调整（如果启用了参数服务器）：

```bash
# 调整插值权重
ros2 param set /jaka_control_node interpolation_weight 0.25

# 调整速度因子
ros2 param set /jaka_control_node velocity_safety_factor 0.7
```

**注意**：buffer_size 需要重启才能生效。

## 📚 相关文档

- [VELOCITY_LIMITING.md](VELOCITY_LIMITING.md) - 速度限制实现
- [SERVO_FIX.md](SERVO_FIX.md) - 伺服bounce修复
- [smooth_params.yaml](config/smooth_params.yaml) - 参数模板
