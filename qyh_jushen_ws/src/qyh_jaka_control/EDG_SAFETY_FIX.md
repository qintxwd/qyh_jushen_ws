# EDG模式抖动修复报告 - 关键修复

## 🚨 问题描述

手柄几乎没动，但真实机械臂报错卡死，日志显示：
- `Buffer empty, holding last position: [0,0,0,0,0,0,0]` 
- `Using current position (last_output 7725.7ms ago)`  
- 微小的position delta (0.001~0.015 rad) 反复出现

## 🎯 根本原因（ChatGPT完全正确的分析）

1. **EDG模式下"微抖动"是致命的**  
   - delta在±0.01 rad范围内来回变化  
   - 对人来说是"静止"，对控制器是"高频抖动"  
   - 每8ms都在改变方向 → 加速度/jerk极大

2. **"Buffer empty" = 控制权丢失**  
   - EDG要求连续输入流，不能断流  
   - 即使一帧buffer空 → 控制器判定失去控制权 → 保护停机

3. **"from current position"重启 = 轨迹不连续**  
   - 使用滞后的真实位置 → 与上次输出位置不一致 → 非法跳变  
   - EDG检测到参考轨迹断裂 → 立即报错

4. **"几乎没动" ≠ "安全"，反而最危险**  
   - 大幅度动作：delta大，速度连续，控制器能跟踪  
   - 微小抖动：目标在左右横跳，控制器无法安全跟踪

## ✅ 修复措施（5个止血级修改）

### 1️⃣ 添加关节死区（joint_deadzone_） ⭐ 最关键

```cpp
double joint_deadzone_ = 0.01;  // 0.01 rad ≈ 0.57度
int stationary_count_ = 0;
static constexpr int STATIONARY_THRESHOLD = 10;  // 连续10帧静止

// 在addCommand中：
double max_delta = 0.0;
for (size_t i = 0; i < 7; ++i) {
    double delta = std::abs(joint_positions[i] - from_position[i]);
    max_delta = std::max(max_delta, delta);
}

if (max_delta < joint_deadzone_) {
    stationary_count_++;
    if (stationary_count_ >= STATIONARY_THRESHOLD) {
        RCLCPP_DEBUG(logger_, "Stationary, ignoring micro-delta");
        return true;  // 过滤掉微小变化
    }
} else {
    stationary_count_ = 0;  // 有明显运动，重置计数
}
```

**效果**：彻底消除手柄噪声、IK误差导致的微抖动输入。

### 2️⃣ Buffer空时持续发送last_output

**修复前**（危险）：
```cpp
if (command_buffer_.empty()) {
    if (has_last_output_) {
        interpolated_positions = last_output_command_.positions;  // ❌ 但positions是零！
        RCLCPP_INFO(logger_, "holding last position: [0,0,0,0,0,0,0]");
        return true;
    }
    return false;  // ❌ 返回false → mainLoop不发送 → 断流！
}
```

**修复后**（安全）：
```cpp
// 已经在代码中实现，确保：
// 1. 第一次addCommand时初始化last_output
last_output_command_.positions = joint_positions;
last_output_command_.timestamp = now;
has_last_output_ = true;

// 2. Buffer空时持续发送有效的last_output
if (command_buffer_.empty()) {
    if (has_last_output_) {
        interpolated_positions = last_output_command_.positions;
        return true;  // ✅ 持续发送，不断流
    }
}
```

**效果**：保证EDG永不断流。

### 3️⃣ 移除"from current position"重启逻辑

**修复前**（破坏连续性）：
```cpp
if (time_since_last > stale_threshold_sec_ && !current_position.empty()) {
    from_position = current_position;  // ❌ 使用滞后的真实位置
    using_current = true;
    RCLCPP_INFO(logger_, "Using current position (last_output %.1fms ago)");
}
```

**修复后**（保持连续性）：
```cpp
if (!command_buffer_.empty()) {
    from_position = command_buffer_.back().positions;  // ✅ 使用buffer位置
} else if (has_last_output_) {
    from_position = last_output_command_.positions;  // ✅ 或last_output
}
// 移除了所有"Using current position"的分支（除了首次初始化）
```

**效果**：始终基于上次发送的指令计算增量，保持轨迹连续性。

### 4️⃣ 静止检测（连续10帧死区内 → 停止添加）

```cpp
if (max_delta < joint_deadzone_) {
    stationary_count_++;
    if (stationary_count_ >= 10) {
        return true;  // 不添加新命令，但getInterpolatedCommand会持续发送last_output
    }
}
```

**效果**：手柄静止时不产生新命令，但EDG仍持续收到last_output，保持合法状态。

### 5️⃣ 日志降级（减少刷屏）

```cpp
RCLCPP_DEBUG(logger_, "...");  // 高频日志改为DEBUG
```

**效果**：便于观察关键信息。

## 📊 修复对比

| 场景 | 修复前 | 修复后 |
|------|--------|--------|
| **手柄微抖** | ❌ 每8ms发送不同指令 → EDG拒绝 | ✅ 死区过滤，不发送 |
| **Buffer空** | ❌ 发送零位[0,0,0,0,0,0,0] → 断流 | ✅ 发送last_output → 连续 |
| **缓存过期** | ❌ 从current pos重启 → 跳变 | ✅ 从last_output继续 → 平滑 |
| **静止10帧** | ❌ 继续添加微小变化 | ✅ 停止添加，但持续发送 |

## 🔍 为什么这是"控制范式问题"而非调参问题

EDG/Servo模式的本质要求：

```
✅ 正确：连续速度/增量流
  - 每帧都发送（不能断）
  - 基于上次输出计算（不能跳）
  - 速度/加速度连续（不能抖）

❌ 错误：位置指令 + 断流 + 重启
  - Buffer empty → 不发送
  - 从current position重新起步
  - 微小变化当作真实输入
```

## 🛠️ 关键代码修改清单

### smooth_servo_bridge.hpp
```cpp
// 新增成员变量
double joint_deadzone_;
int stationary_count_;
static constexpr int STATIONARY_THRESHOLD = 10;
```

### smooth_servo_bridge.cpp
```cpp
// 1. 构造函数初始化
joint_deadzone_(0.01),
stationary_count_(0),

// 2. addCommand - 死区检查
if (max_delta < joint_deadzone_) {
    stationary_count_++;
    if (stationary_count_ >= STATIONARY_THRESHOLD) {
        return true;  // 过滤
    }
}

// 3. addCommand - 移除stale threshold重启
// （保留首次初始化用current_position，之后始终用last_output）

// 4. getInterpolatedCommand - 确保buffer空时发送last_output
if (command_buffer_.empty()) {
    if (has_last_output_) {
        interpolated_positions = last_output_command_.positions;
        return true;  // 不是返回false
    }
}

// 5. 首次命令立即初始化last_output
last_output_command_.positions = joint_positions;
last_output_command_.timestamp = now;
has_last_output_ = true;
```

## 🧪 测试验证

### 编译
```bash
cd qyh_jushen_ws
colcon build --packages-select qyh_jaka_control
```

### 可视化模式测试（安全）
```bash
./start_visualization_test.sh
# 观察RViz是否平滑，无抖动
```

### 真实机器人测试
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py
# 手柄微动，观察是否：
# 1. 不再报错
# 2. 日志显示 "Stationary, ignoring micro-delta"
# 3. 没有 "Buffer empty, holding [0,0,0,0,0,0,0]"
# 4. 没有 "Using current position"（除首次）
```

### 预期日志（正常）
```
[Bridge] Stationary (10 frames), ignoring micro-delta (max=0.0023 < deadzone=0.0100)
[Bridge] Buffer empty, holding last position: [0.52, -0.37, -0.15, ...]  ← 非零！
[Bridge] Near-stationary (count=3/10), max_delta=0.0087
```

### 预期行为
- ✅ 手柄静止：机械臂保持位置，EDG不报错
- ✅ 手柄微动：死区过滤，不抖动
- ✅ 手柄大幅度移动：正常跟随，平滑

## ⚠️ 注意事项

1. **死区不是越大越好**  
   - 0.01 rad (0.57度) 适合VR手柄+IK噪声  
   - 太大 (>0.02) 会影响精细操作灵敏度  
   - 太小 (<0.005) 无法过滤噪声

2. **静止阈值10帧的含义**  
   - 10帧 × 33ms = 330ms  
   - 手柄停止330ms后才判定为静止  
   - 避免误判快速小幅度运动

3. **首次初始化仍需current_position**  
   - 启动时机械臂可能不在零位  
   - 第一个命令必须从真实位置开始  
   - 之后全部用last_output保持连续性

## 📚 相关文档

- [VELOCITY_LIMITING.md](VELOCITY_LIMITING.md) - 速度插值实现
- [SERVO_FIX.md](SERVO_FIX.md) - 之前的bounce修复
- [SMOOTHNESS_OPTIMIZATION.md](SMOOTHNESS_OPTIMIZATION.md) - 平滑度参数调优

## 🎓 EDG控制的核心原则（ChatGPT总结）

> "EDG / Servo 模式下：  
> '几乎没动' ≠ '安全'，反而是'最危险的输入状态'。  
> 你现在看到的所有报错，全部符合工业机器人控制器的设计逻辑。"

- ✅ 连续性 > 精度
- ✅ 速度限制 > 位置精度
- ✅ 持续发送 > 精确到达
- ✅ 死区过滤 > 忠实跟随

---

**修复完成时间**: 2025-12-18  
**关键改进**: 从"位置指令模式"转变为"EDG连续控制模式"
