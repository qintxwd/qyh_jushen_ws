# 速度积分控制模式 - 关键修复

## ✅ 已修复的问题

### 1. **Jacobian基准状态不一致** (CRITICAL)
**问题**：原代码使用积分状态`integrated_q_`计算FK和Jacobian，导致每次循环基准都在变化。

**修复**：
```cpp
// 修复前（错误）
KDL::JntArray q_calc(chain_.getNrOfJoints());
for(size_t i=0; i<chain_.getNrOfJoints(); ++i) {
    q_calc(i) = integrated_q_[i];  // 使用积分状态 ❌
}
fk_solver_->JntToCart(q_calc, current_pose);
jac_solver_->JntToJac(q_calc, jac);

// 修复后（正确）
fk_solver_->JntToCart(current_q_, current_pose);  // 使用真实机器人状态 ✅
jac_solver_->JntToJac(current_q_, jac);          // 使用真实机器人状态 ✅
```

**影响**：避免数值漂移，确保控制精度。

---

### 2. **死区缺失导致目标附近抖动**
**问题**：即使已经到达目标，仍然发送微小速度命令，导致抖动。

**修复**：
```cpp
// 添加位置和角度死区
double position_error = twist.vel.Norm();
double orientation_error = twist.rot.Norm();
const double POSITION_DEADZONE = 0.001;      // 1mm
const double ORIENTATION_DEADZONE = 0.017;   // ~1度

if (position_error < POSITION_DEADZONE && orientation_error < ORIENTATION_DEADZONE) {
    // 目标已到达，保持当前位置
    next_joints = integrated_q_;
    return true;
}
```

**影响**：消除目标附近的抖动，提高稳定性。

---

### 3. **增益应用顺序优化** (之前已修复)
**修复**：先应用增益再限制速度，确保响应性。

---

## 控制流程

```
VR Pose (30Hz)
  ↓
TF Transform (base_link_left/right)
  ↓
setTargetPose()
  ↓
mainLoop (125Hz)
  ↓
updateRobotState(current_joints)  ← 从SDK获取真实状态
  ↓
computeNextCommand()
  ├─ FK(current_q) → current_pose          ← 使用真实状态
  ├─ Twist = diff(current, target)
  ├─ 检查死区 (1mm, 1°)
  ├─ 应用增益 & 速度限制
  ├─ Jacobian(current_q)                   ← 使用真实状态
  ├─ Diff IK: q_dot = J† * twist
  └─ Integrate: integrated_q += q_dot * dt
  ↓
edgServoJ(integrated_q) + edgSend()
  ↓
JAKA Robot (持续125Hz命令流)
```

---

## 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `dt` | 0.008s (125Hz) | 控制周期 |
| `linear_gain` | 2.0 | 线速度增益 |
| `angular_gain` | 1.0 | 角速度增益 |
| `max_linear_vel` | 0.5 m/s | 最大线速度 |
| `max_angular_vel` | 1.0 rad/s | 最大角速度 |
| `joint_vel_limit` | 1.5 rad/s | 关节速度限制 |
| `lambda` (阻尼) | 0.01 | 奇异点阻尼系数 |
| **POSITION_DEADZONE** | 0.001 m (1mm) | 位置死区 |
| **ORIENTATION_DEADZONE** | 0.017 rad (~1°) | 角度死区 |

---

## 安全保障

1. ✅ **持续命令流**：即使无目标，也发送当前位置保持EDG连接
2. ✅ **速度限制**：笛卡尔空间和关节空间双重限制
3. ✅ **奇异点处理**：阻尼伪逆避免奇异点附近的速度爆炸
4. ✅ **死区保护**：避免目标附近的微小抖动

---

## 代码清理

已删除：
- ❌ `SmoothServoBridge` 及所有位置插值逻辑
- ❌ `solveLeftArmIK/solveRightArmIK` (JAKA SDK IK)
- ❌ Bridge回调和参数配置
- ❌ IK统计和旧的订阅者

保留：
- ✅ `VelocityServoController` (唯一控制器)
- ✅ KDL正向/逆运动学
- ✅ 阻尼伪逆微分IK
- ✅ 速度积分

---

## 编译说明

⚠️ **当前编译错误不是代码问题**：
```
error adding symbols: file in wrong format
```
这是因为JAKA SDK `.so`文件是ARM64格式（Jetson），无法在WSL x64环境编译。

**解决方案**：
- 在Jetson设备上编译（正式部署环境）
- 或使用Docker ARM64容器
