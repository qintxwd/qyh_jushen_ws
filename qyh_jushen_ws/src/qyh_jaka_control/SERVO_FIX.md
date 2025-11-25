# 伺服模式不移动问题修复

## 问题描述

GUI程序开启伺服测试后，机械臂不移动，但SDK示例程序 `30.edgservo.cpp` 可以正常移动。

## 根本原因

对比SDK示例代码，发现关键差异：

### SDK正确流程 (30.edgservo.cpp)

```cpp
// 1. 启用伺服模式
robot.servo_move_enable(1, 0);  // 左臂
robot.servo_move_enable(1, 1);  // 右臂

// 2. 伺服循环
for(int i = 0;;i++) {
    // 准备指令
    robot.edg_servo_j(0, &jpos_cmd, MoveMode::ABS);
    robot.edg_servo_j(1, &jpos_cmd, MoveMode::ABS);
    
    // ⚠️ 关键：发送指令到机器人
    robot.edg_send(&cmd_index);
    
    // 等待下一个周期
    std::this_thread::sleep_until(next_time);
}
```

### 原代码问题

ROS接口中：
- ✅ `servoJ()` 调用了 `edg_servo_j()` 准备指令
- ❌ **缺少 `edg_send()` 调用** - 指令从未发送到机器人！

## 修复方案

### 修改的文件

1. **jaka_interface.cpp**
   - `servoJ()` 和 `servoP()` 只准备指令，不自动发送
   - 调用者必须显式调用 `edgSend()` 发送指令

2. **jaka_control_node.cpp**
   - 伺服定时器回调中，准备好指令后调用 `edgSend()`
   - VR跟随模式中，准备好两个臂的指令后统一 `edgSend()`

### 修复后的流程

```cpp
// 关节伺服
void processServoControl() {
    // 准备双臂指令
    jaka_interface_.servoJ(-1, positions, is_abs);
    
    // ⭐ 发送到机器人
    jaka_interface_.edgSend();
}

// VR跟随
void processVRFollow() {
    // 准备左臂指令
    jaka_interface_.servoP(0, left_target, true);
    // 准备右臂指令
    jaka_interface_.servoP(1, right_target, true);
    
    // ⭐ 统一发送
    jaka_interface_.edgSend();
}
```

## 为什么这样设计？

### SDK的两阶段设计

1. **准备阶段** (`edg_servo_j/p`)
   - 将指令写入内部缓冲区
   - 可以准备多个机器人的指令
   - 不会立即执行

2. **发送阶段** (`edg_send`)
   - 将所有准备好的指令一次性发送
   - 确保双臂同步
   - 触发机器人执行

### 同步的重要性

双臂机器人需要确保：
- 左右臂指令同时到达
- 避免时间差导致的不协调
- 保持8ms控制周期的精确性

## 测试验证

### 编译

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_jaka_control
source install/setup.bash
```

### 运行测试

```bash
# 1. 启动控制节点
ros2 launch qyh_jaka_control jaka_control.launch.py

# 2. 启动GUI
ros2 run qyh_jaka_control_gui jaka_complete_gui

# 3. GUI操作
#    - 基础控制 -> 上电 -> 使能
#    - 伺服模式 -> 启动伺服模式
#    - 伺服模式 -> 开始测试
#    - 观察机械臂应该开始缓慢的正弦波运动
```

### 预期结果

- ✅ 机械臂开始移动
- ✅ 运动流畅（8ms周期）
- ✅ 正弦波轨迹清晰可见
- ✅ 左臂关节0、1、3协调运动

## 关键代码对比

### 修复前

```cpp
bool JakaInterface::servoJ(...) {
    robot_->edg_servo_j(0, &jpos[0], ...);
    robot_->edg_servo_j(1, &jpos[1], ...);
    // ❌ 缺少 edg_send()
    return true;
}
```

### 修复后

```cpp
bool JakaInterface::servoJ(...) {
    robot_->edg_servo_j(0, &jpos[0], ...);
    robot_->edg_servo_j(1, &jpos[1], ...);
    // NOTE: Caller must call edgSend()
    return true;
}

// 调用处
void processServoControl() {
    jaka_interface_.servoJ(-1, positions, is_abs);
    jaka_interface_.edgSend();  // ⭐ 关键修复
}
```

## 注意事项

1. **每个控制周期必须调用 `edgSend()`**
   - 125Hz控制频率 = 8ms周期
   - 每个周期准备指令 + 发送

2. **双臂协调**
   - 先准备所有臂的指令
   - 再统一发送
   - 避免分开发送导致不同步

3. **实时性要求**
   - 必须在8ms内完成
   - 使用实时调度（SCHED_FIFO）
   - 避免在控制循环中有阻塞操作

## 参考

- SDK示例：`30.edgservo.cpp` - 关节伺服
- SDK示例：`31.edgservo_fct.cpp` - 力控伺服
- SDK示例：`32.controlloop.cpp` - 控制环测试

## 总结

修复的核心：**`edg_servo_j/p` 只是准备指令，`edg_send` 才真正发送到机器人！**

这是双臂机器人SDK的设计模式，确保：
- 指令同步
- 高频控制
- 实时响应

修复后，GUI的伺服测试应该能够正常移动机械臂了！
