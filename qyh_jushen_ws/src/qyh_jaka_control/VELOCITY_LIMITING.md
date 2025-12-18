# 速度限制与自动插值功能

## 🎯 功能说明

已在 `SmoothServoBridge` 中实现**速度限制和自动插值**功能，防止机械臂运动速度超限。

## ⚙️ 工作原理

### 1. 智能起始位置选择 ⭐ NEW
`addCommand` 接受可选的 `current_position` 参数，自动选择最佳起始位置：

**决策逻辑**：
```cpp
if (buffer非空 && 最后cmd时间戳 < 50ms):
    使用 buffer.back()  // 缓存新鲜
else if (buffer非空 && current_position提供):
    使用 current_position  // 缓存过期，用真实位置
else if (has_last_output && 输出时间戳 < 50ms):
    使用 last_output  // 缓存新鲜
else if (has_last_output && current_position提供):
    使用 current_position  // 缓存过期，用真实位置
else if (current_position提供):
    使用 current_position  // 首次命令
else:
    直接添加，无插值  // 无参考位置
```

**时间阈值**：`stale_threshold_sec_ = 50ms` (约6个控制周期)
- 正常情况下每8ms有新命令
- 超过50ms说明可能中断或延迟
- 此时使用缓存位置可能不准确，改用真实位置更安全

### 2. 速度限制检查
当新命令添加到缓冲区时，自动计算从起始位置到目标位置需要的时间：

```cpp
required_time = max(|target[i] - start[i]| / (vel_max[i] * safety_factor))
```

### 3. 自动插值
如果 `required_time > 2 * cycle_time` (16ms)，自动插入中间点：

**示例**：
- 当前位置 → 目标位置需要 20ms
- 控制周期 = 8ms
- 需要周期数 = ceil(20/8) = 3
- 插入点数 = 3 - 1 = 2个中间点
- 插入位置：I1(7ms), I2(14ms)

**队列变化**：
```
原始: [当前] → [目标]
插值后: [当前] → [I1] → [I2] → [目标]
```

### 4. 连续插值
对buffer中的每个命令间隔都进行检查：
```
目标1 → 目标2 (检查并插值)
目标2 → 目标3 (检查并插值)
...
```

## 📊 速度限制参数

### JAKA Zu7 速度限制
```cpp
关节1: 1.5708 rad/s (90°/s)
关节2: 1.5708 rad/s (90°/s)
关节3: 2.0944 rad/s (120°/s)
关节4: 2.0944 rad/s (120°/s)
关节5: 2.6180 rad/s (150°/s)
关节6: 2.6180 rad/s (150°/s)
关节7: 2.6180 rad/s (150°/s)
```

### 安全系数
```cpp
SAFETY_MARGIN_VEL = 0.8  // 使用80%的最大速度
```

实际允许速度 = `vel_max * 0.8`

## 🔧 API接口

### 添加命令（智能模式）
```cpp
// 推荐：传入当前真实位置
JointValue current_pos;
jaka_interface_.getJointPositions(robot_id, current_pos);
std::vector<double> current(7);
for (size_t i = 0; i < 7; ++i) current[i] = current_pos.jVal[i];

left_bridge_->addCommand(target_positions, current);  // ⭐ 智能选择起始位置
```

```cpp
// 兼容：不传current_position（使用缓存）
left_bridge_->addCommand(target_positions);  // 总是使用缓存位置
```

### 设置速度限制
```cpp
std::vector<double> velocity_limits(7);
for (size_t i = 0; i < 7; ++i) {
    velocity_limits[i] = JAKA_ZU7_LIMITS[i].vel_max;
}
left_bridge_->setVelocityLimits(velocity_limits);
```

### 设置安全系数
```cpp
left_bridge_->setVelocitySafetyFactor(0.8);  // 80%安全速度
```

## 📝 实现细节

### addCommand() 流程
```cpp
1. 智能选择起始位置：
   - 检查buffer/last_output时间戳
   - 如果 < 50ms: 使用缓存
   - 如果 > 50ms 且提供current_position: 使用真实位置
   
2. 计算到新目标需要的时间
   required_time = calculateRequiredTime(start, target)
   
3. if (required_time > 2 * cycle_time):
     计算需要的周期数 n = ceil(required_time / cycle_time)
     插入 n-1 个中间点
     
4. 添加最终目标到buffer
```

### 中间点计算
```cpp
for i in [1, 2, ..., n-1]:
    t = i / n
    intermediate[j] = from[j] + t * (to[j] - from[j])
```

## ✅ 优势

1. **智能位置选择** ⭐ - 自动检测缓存是否过期，决定使用缓存还是真实位置
2. **防止速度超限** - 自动限制最大运动速度
3. **平滑运动** - 避免大幅度跳变
4. **高效实现** - 在入队时完成插值，无运行时开销
5. **自适应插值** - 根据实际需要的时间动态插入点数
6. **连续检查** - 对所有目标间隔都进行速度限制
7. **更安全** - 中断后恢复时使用真实位置，避免大幅跳变

## 🧪 测试场景

### 场景1：小幅度运动（无需插值）
```
当前: [0, 0, 0, 0, 0, 0, 0]
目标: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

速度: 0.01 / 0.008 = 1.25 rad/s < 1.5708 * 0.8
结果: 直接添加，无插值
```

### 场景2：大幅度运动（需要插值）
```
当前: [0, 0, 0, 0, 0, 0, 0]
目标: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

关节1速度: 0.5 / 0.008 = 62.5 rad/s >> 1.5708 * 0.8
需要时间: 0.5 / (1.5708 * 0.8) = 0.398s
需要周期: ceil(0.398 / 0.008) = 50
插入点数: 49个

结果: 自动插入49个中间点，平滑运动
```

### 场景3：VR遥操作（连续目标）
```
T=0ms:  目标1 [0.1, ...]
T=8ms:  目标2 [0.2, ...]  ← 检查 目标1→目标2
T=16ms: 目标3 [0.3, ...]  ← 检查 目标2→目标3

每个间隔都会检查速度限制并插值
```

## 🔍 日志输出

### 使用缓存位置（正常情况）
```
[Bridge] Command added: [0.01, 0.01, ...], buffer_size=1
```

### 使用当前真实位置（缓存过期）
```
[Bridge] Using current position (last_cmd 60.0ms ago > 50.0ms threshold)
[Bridge] Command added: [0.01, 0.01, ...], buffer_size=1 [from current pos]
```

### 插值添加（速度限制触发）
```
[Bridge] Inserted 2 intermediate points (required_time=20.0ms, cycle=8.0ms) [from current pos]
[Bridge] Command added: [0.50, 0.50, ...], buffer_size=4 [from current pos]
```

## 📈 性能影响

- **计算开销**: O(n) 其中n是关节数（7），非常小
- **内存开销**: 额外的中间点占用buffer空间
- **实时性**: 插值在addCommand时完成，不影响125Hz主循环

## 🎯 建议配置

### 保守配置（更安全）
```cpp
SAFETY_MARGIN_VEL = 0.6  // 60%速度
buffer_size = 20         // 更大缓冲区
```

### 激进配置（更快响应）
```cpp
SAFETY_MARGIN_VEL = 0.9  // 90%速度
buffer_size = 10         // 标准缓冲区
```

### VR遥操作推荐
```cpp
SAFETY_MARGIN_VEL = 0.8  // 80%速度（当前设置）
buffer_size = 10         // 标准缓冲区
```

## 🔄 与现有功能的配合

1. **插值平滑** - 仍然生效，作用于最终的buffer序列
2. **Buffer清空** - 到达目标后仍会清空buffer
3. **位置保持** - Buffer空时仍保持最后位置

速度限制插值是**第一层保护**，插值平滑是**第二层优化**。
