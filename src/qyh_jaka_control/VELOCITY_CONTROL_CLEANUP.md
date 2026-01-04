# 速度积分控制模式清理总结

## 已删除的旧代码

### 1. 移除的组件
- ✅ `SmoothServoBridge` 类及其所有引用
- ✅ `leftBridgeCallback` / `rightBridgeCallback`
- ✅ `solveLeftArmIK` / `solveRightArmIK` (标记为废弃)
- ✅ Bridge相关参数（buffer_size, interpolation_weight等）
- ✅ Bridge相关订阅者（left_bridge_sub_, right_bridge_sub_）
- ✅ IK统计变量（ik_*_success_count, ik_*_error_count）

### 2. CMakeLists.txt清理
- ✅ 移除smooth_servo_bridge库的编译和链接
- ✅ 只保留velocity_servo_controller

## 速度积分模型问题修复

### ⚠️ 发现的关键问题

1. **增益应用顺序错误** (已修复)
   - **错误**：先限制速度再应用增益 → 响应太慢
   - **修复**：先应用增益再限制速度
   
2. **Jacobian基准状态不一致** (待修复)
   - **问题**：使用`integrated_q_`计算Jacobian，但每次循环`integrated_q_`都在变化
   - **症状**：可能导致轻微的数值漂移
   - **修复方案**：应该基于真实机器人状态`current_q_`计算Jacobian，而不是积分状态

3. **死区和收敛条件缺失** (待添加)
   - **问题**：目标接近时仍然发送微小速度，容易抖动
   - **修复方案**：添加位置死区（1mm）和角度死区（1度）

## 推荐的进一步优化

1. **添加死区逻辑**：
   ```cpp
   // 距离死区
   if (linear_norm < 0.001 && angular_norm < 0.017) {
       return false; // 不发送命令，保持当前位置
   }
   ```

2. **修复Jacobian基准**：
   ```cpp
   // 应该使用真实机器人状态而非积分状态
   fk_solver_->JntToCart(current_q_, current_pose);
   jac_solver_->JntToJac(current_q_, jac);
   ```

3. **添加速度平滑**：
   ```cpp
   // 对q_dot进行一阶低通滤波
   q_dot_filtered = alpha * q_dot + (1-alpha) * q_dot_prev;
   ```

## 当前架构

**VR Pose (30Hz)** 
→ TF Transform 
→ `VelocityServoController::setTargetPose` 
→ mainLoop (125Hz) 
→ `computeNextCommand` 
  - FK(integrated_q) → current_pose
  - Twist = diff(current, target)
  - Apply Gains & Limits
  - Jacobian(integrated_q)
  - Diff IK: q_dot = J† * twist
  - Integrate: q += q_dot * dt
→ edgServoJ + edgSend

## 安全保障

- ✅ 关节速度限制：1.5 rad/s
- ✅ 笛卡尔线速度限制：0.5 m/s
- ✅ 笛卡尔角速度限制：1.0 rad/s
- ✅ 奇异点阻尼：lambda = 0.01
- ✅ 无目标时发送当前位置（保持EDG连接）
