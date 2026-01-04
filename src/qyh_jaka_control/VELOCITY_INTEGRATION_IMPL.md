# 速度积分模型实现方案

## 🎯 目标

在保留Bridge结构的前提下，切换到**速度积分+加速度限制**模式，实现工业级遥操控制。

## 📊 核心原理

### 当前模式（位置控制）
```
VR输入 → IK → 绝对关节角q_target → 插值 → EDG
问题：位置跳变、速度不可控、加速度不可控
```

### 速度积分模式
```
VR输入 → IK → 目标q_target
       ↓
    计算误差 err = q_target - q_current
       ↓
    死区过滤 if |err| < deadband: err = 0
       ↓
    速度限制 dq = clamp(err/dt, ±max_vel)
       ↓
    加速度限制 acc = (dq - dq_last)/dt
                 if |acc| > max_acc:
                     dq = dq_last + sign(acc)*max_acc*dt
       ↓
    积分 q_cmd = q_last + dq * dt
       ↓
    发送EDG (q_cmd)
```

**关键**：每帧输出是基于上一帧的**增量积分**，而非绝对位置。

---

## 🔧 实现方案A：在Bridge内部改造（最小侵入）

### 1. 修改SmoothServoBridge数据结构

```cpp
// smooth_servo_bridge.hpp
class SmoothServoBridge {
private:
    // 速度积分模式状态
    bool velocity_mode_{true};  // 启用速度积分模式
    std::vector<double> last_cmd_positions_;  // 上帧发送的关节角
    std::vector<double> last_velocities_;     // 上帧速度
    std::vector<double> max_accelerations_;   // 加速度限制
    
    // 原有的删除：command_buffer_（不再需要buffer）
};
```

### 2. 核心函数改造

#### addCommand → setTarget
```cpp
bool SmoothServoBridge::setTarget(
    const std::vector<double>& target_positions,
    const std::vector<double>& current_positions)
{
    // 只更新目标，不做其他事情
    std::lock_guard<std::mutex> lock(mutex_);
    target_positions_ = target_positions;
    
    // 第一次初始化
    if (!initialized_ && !current_positions.empty()) {
        last_cmd_positions_ = current_positions;
        last_velocities_.assign(7, 0.0);
        initialized_ = true;
    }
    return true;
}
```

#### getInterpolatedCommand → computeVelocityCommand
```cpp
bool SmoothServoBridge::computeVelocityCommand(
    std::vector<double>& output_positions,
    const std::vector<double>& current_positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_WARN(logger_, "Not initialized");
        return false;
    }
    
    output_positions.resize(7);
    
    for (size_t i = 0; i < 7; ++i) {
        // 1. 计算误差
        double err = target_positions_[i] - current_positions[i];
        
        // 2. 死区
        if (std::abs(err) < joint_deadzone_) {
            err = 0.0;
        }
        
        // 3. 速度限制
        double max_vel = velocity_limits_[i] * velocity_safety_factor_;
        double desired_vel = std::clamp(err / cycle_time_sec_, 
                                        -max_vel, max_vel);
        
        // 4. 加速度限制
        double max_acc = max_accelerations_[i];
        double acc = (desired_vel - last_velocities_[i]) / cycle_time_sec_;
        double actual_vel = desired_vel;
        
        if (std::abs(acc) > max_acc) {
            actual_vel = last_velocities_[i] + 
                         std::copysign(max_acc * cycle_time_sec_, acc);
        }
        
        // 5. 积分
        output_positions[i] = last_cmd_positions_[i] + 
                              actual_vel * cycle_time_sec_;
        
        // 6. 更新状态
        last_velocities_[i] = actual_vel;
    }
    
    // 7. 更新上次输出
    last_cmd_positions_ = output_positions;
    
    return true;
}
```

---

## 🔧 实现方案B：新增VelocityServoController（独立模块）

如果想保持Bridge不变，创建新的速度积分模块：

```cpp
// velocity_servo_controller.hpp
class VelocityServoController {
public:
    VelocityServoController(
        rclcpp::Logger logger,
        double control_dt = 0.008  // 125Hz
    );
    
    // 设置目标（来自VR/手柄）
    void setTarget(const std::vector<double>& target);
    
    // 计算本帧指令（在mainLoop调用）
    std::vector<double> computeCommand(
        const std::vector<double>& current_joints
    );
    
    // 配置
    void setLimits(
        const std::vector<double>& max_vels,
        const std::vector<double>& max_accs
    );
    
    void setDeadband(double deadband);
    
private:
    rclcpp::Logger logger_;
    double dt_;
    double deadband_{0.01};
    
    std::vector<double> target_;
    std::vector<double> q_last_;
    std::vector<double> dq_last_;
    std::vector<double> max_vel_;
    std::vector<double> max_acc_;
    
    bool initialized_{false};
};
```

### 在jaka_control_node中使用

```cpp
// 初始化
velocity_controller_ = std::make_unique<VelocityServoController>(
    get_logger(), cycle_time_ms_ / 1000.0
);

// 设置限制
velocity_controller_->setLimits(velocity_limits, max_accelerations);
velocity_controller_->setDeadband(0.01);

// VR回调：只更新目标
void leftVRTargetCallback(const PoseStamped::SharedPtr msg) {
    if (solveLeftArmIK()) {
        velocity_controller_->setTarget(ik_result);
    }
}

// mainLoop：计算并发送
void mainLoop() {
    // 读当前关节
    jaka_interface_.getJointPositions(0, current_left);
    
    // 计算速度积分指令
    auto cmd = velocity_controller_->computeCommand(current_left);
    
    // 发送EDG
    jaka_interface_.edgServoJ(0, cmd, true);
    jaka_interface_.edgSend(&cmd_index_);
}
```

---

## 📊 两种方案对比

| 方案 | 改动量 | 兼容性 | 推荐度 |
|------|--------|--------|--------|
| **方案A** | 中等（改造Bridge） | 可能影响现有代码 | ⭐⭐⭐ |
| **方案B** | 小（新增模块） | 完全兼容 | ⭐⭐⭐⭐⭐ |

**我推荐方案B**：创建独立的VelocityServoController，保持Bridge不变。

---

## 🎯 关键参数配置

```yaml
# velocity_servo_config.yaml
velocity_servo:
  control_frequency: 125.0  # Hz
  
  # 死区（过滤微抖动）
  joint_deadband: 0.01  # rad (0.57度)
  
  # 速度限制（rad/s）
  max_joint_velocities: [1.57, 1.57, 2.09, 2.09, 2.62, 2.62, 2.62]
  velocity_safety_factor: 0.65  # 使用65%最大速度
  
  # 加速度限制（rad/s²）- 关键！
  max_joint_accelerations: [2.0, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0]
  
  # Jerk限制（可选，高级）
  max_joint_jerks: [10.0, 10.0, 12.0, 12.0, 15.0, 15.0, 15.0]
```

---

## 🧪 测试验证

### 1. 静止测试
```
手柄不动 → dq = 0 → q_cmd = q_last → 持续发送
预期：机械臂静止，不报错
```

### 2. 微动测试
```
手柄微动(delta < 0.01) → 死区过滤 → dq = 0
预期：过滤噪声，不抖动
```

### 3. 大幅运动测试
```
手柄大幅移动 → 速度限制 → 加速度限制 → 平滑运动
预期：无jerk，平滑跟随
```

### 4. IK失败测试
```
IK失败 → 不更新target → dq计算基于旧target → 继续运动
预期：不报错，逐渐减速到旧目标
```

---

## 📈 性能提升预期

| 指标 | 当前方案 | 速度积分 |
|------|----------|----------|
| **EDG报错** | 偶发 | 几乎消除 |
| **微抖动** | 有（已改善） | 完全消除 |
| **Jerk** | 明显 | 平滑 |
| **响应延迟** | 低 | 略增（可接受） |
| **稳定性** | 良好 | 优秀 |

---

## 🚀 实施路线

### 阶段1：最小验证（1小时）
1. 创建 `VelocityServoController` 类
2. 在mainLoop中集成
3. 测试静止+微动场景

### 阶段2：完整实现（2小时）
1. 添加加速度限制
2. 添加配置文件
3. 完整测试所有场景

### 阶段3：优化调参（按需）
1. 调整deadband
2. 调整加速度限制
3. 可选：添加jerk限制

---

## 💡 核心代码片段（立即可用）

```cpp
// 速度积分核心循环（125Hz）
for (size_t i = 0; i < 7; ++i) {
    // 误差
    double err = target[i] - current[i];
    
    // 死区
    if (fabs(err) < 0.01) err = 0;
    
    // P控制器（速度指令）
    double dq_desired = err / 0.008;  // 125Hz
    
    // 速度限幅
    dq_desired = std::clamp(dq_desired, -max_vel[i], max_vel[i]);
    
    // 加速度限幅
    double acc = (dq_desired - dq_last[i]) / 0.008;
    if (fabs(acc) > max_acc[i]) {
        dq_desired = dq_last[i] + copysign(max_acc[i] * 0.008, acc);
    }
    
    // 积分
    q_cmd[i] = q_last[i] + dq_desired * 0.008;
    
    // 更新
    dq_last[i] = dq_desired;
}
q_last = q_cmd;
```

---

## ❓ FAQ

**Q1: 为什么不删除Bridge？**  
A: Bridge结构可以保留用于非速度积分场景（如轨迹回放），新增VelocityServoController更灵活。

**Q2: 加速度限制值如何确定？**  
A: 从保守值开始（2.0 rad/s²），测试后逐步提高到3.0-4.0。

**Q3: 静止时为什么要持续发送？**  
A: EDG模式要求连续输入，发送"dq=0"告诉控制器"我在主动保持，不是断流"。

**Q4: IK失败怎么办？**  
A: 不更新target，速度积分会基于旧target继续计算，机械臂会平滑减速到旧目标位置。

---

## 🎓 理论补充：为什么速度积分更稳定

### 位置控制模式
```
Δq = q_target - q_current （每帧重新计算基准）
问题：q_current滞后 → Δq不稳定 → 速度跳变
```

### 速度积分模式  
```
q_cmd[n] = q_cmd[n-1] + dq * dt （基于上次输出）
优点：即使输入抖动，输出连续
```

这就是为什么工业控制器更喜欢速度积分：
- ✅ 输出天然连续
- ✅ 速度可控
- ✅ 加速度可控
- ✅ 对输入噪声鲁棒

---

**结论**：速度积分模型是当前工业遥操的**最佳实践**，强烈建议实施方案B（独立模块）。
