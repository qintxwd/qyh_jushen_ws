# 状态栏更新问题诊断与修复

## 问题描述

用户反馈Web界面状态栏显示的以下状态不会更新：
1. 底盘未连接
2. 双臂未上电

## 根本原因分析

### 1. 机械臂状态更新问题

**问题位置：** `jaka_control_node.cpp`

**问题分析：**
- `connected_`, `powered_`, `enabled_` 这些状态变量只在节点初始化时设置一次
- `publishStatus()` 函数虽然每33ms发布一次状态，但使用的是缓存的变量值
- JAKA SDK提供了`isPoweredOn()`, `isEnabled()`等API可以实时查询状态，但代码中没有调用

**当前逻辑：**
```cpp
// jaka_control_node.cpp, 行 314-322
connected_ = false;
powered_ = false;
enabled_ = false;

if (jaka_interface_.connect(robot_ip_)) {
    connected_ = true;
    // ...
    if (jaka_interface_.powerOn()) {
        powered_ = true;  // 只在初始化时设置
    }
    if (jaka_interface_.enableRobot()) {
        enabled_ = true;  // 只在初始化时设置
    }
}
```

**发布状态时：**
```cpp
// jaka_control_node.cpp, 行 742-744
robot_state_msg.connected = connected_;
robot_state_msg.robot_ip = robot_ip_;
robot_state_msg.powered_on = powered_;
robot_state_msg.enabled = enabled_;
```

**问题：**
如果机器人在运行过程中被断电或禁用（比如按下急停），节点不会感知到状态变化，仍然发布 `powered_on = true`。

### 2. 底盘连接状态更新问题

**问题位置：** `qyh_jushen_web/backend/app/ros2_bridge/bridge.py`

**问题分析：**
- 底盘状态的 `connected` 字段硬编码为 `True`
- 即使底盘节点未运行或topic没有数据，前端也会显示"已连接"

**当前逻辑：**
```python
# bridge.py, 行 641
def chassis_status_callback(msg: StandardRobotStatus):
    self.chassis_status = {
        "connected": True,  # 硬编码为True
        "system_status": msg.system_status,
        # ...
    }
```

**问题：**
- 即使 `/standard_robot_status` topic 没有数据发布，`chassis_status` 会保持为 `None` 或旧数据
- 如果订阅器创建失败，`chassis_status` 为 `None`，前端API会返回mock数据或错误

## 修复方案

### 方案1：机械臂状态实时查询（推荐）

在 `publishStatus()` 函数中，调用JAKA SDK的实时查询API：

```cpp
void publishStatus() {
    // 实时查询机器人状态（而不是使用缓存变量）
    bool is_powered = false;
    bool is_enabled = false;
    
    if (connected_) {
        // JAKA SDK提供了这些API
        is_powered = jaka_interface_.isPoweredOn();
        is_enabled = jaka_interface_.isEnabled();
    }
    
    robot_state_msg.connected = connected_;
    robot_state_msg.powered_on = is_powered;
    robot_state_msg.enabled = is_enabled;
    // ...
}
```

**需要修改的文件：**
1. `jaka_interface.hpp` - 添加 `isPoweredOn()`, `isEnabled()` 方法声明
2. `jaka_interface.cpp` - 实现这些方法
3. `jaka_control_node.cpp` - 在 `publishStatus()` 中调用

### 方案2：底盘连接状态检测

在 `ros2_bridge/bridge.py` 中添加连接状态超时检测：

```python
import time

class ROS2Bridge:
    def __init__(self):
        self.chassis_status = None
        self.chassis_last_update_time = None
        self.chassis_timeout_seconds = 2.0  # 2秒未收到数据视为断连
    
    def _setup_chassis_subscribers(self):
        def chassis_status_callback(msg: StandardRobotStatus):
            # 更新时间戳
            self.chassis_last_update_time = time.time()
            
            self.chassis_status = {
                "connected": True,  # 收到消息时为True
                "system_status": msg.system_status,
                # ...
            }
    
    def get_chassis_status(self) -> Optional[Dict[str, Any]]:
        """获取底盘状态（带超时检测）"""
        if self.mock_mode:
            return None
        
        if self.chassis_status is None:
            return None
        
        # 检查是否超时
        if self.chassis_last_update_time is not None:
            elapsed = time.time() - self.chassis_last_update_time
            if elapsed > self.chassis_timeout_seconds:
                # 超时，标记为断连
                status = self.chassis_status.copy()
                status["connected"] = False
                return status
        
        return self.chassis_status
```

### 方案3：前端显示优化（临时方案）

如果后端无法立即修复，可以在前端添加更智能的状态判断：

```typescript
// stores/layout.ts
const chassisStatusInfo = computed(() => {
  if (!chassisStatus.value) {
    return { text: '未连接', color: 'grey' };
  }
  
  // 如果status对象存在但connected为false
  if (chassisStatus.value.connected === false) {
    return { text: '已断连', color: 'red' };
  }
  
  // 根据system_status判断
  if (chassisStatus.value.system_status === 1) {
    return { text: '正常', color: 'green' };
  }
  // ...
});
```

## 建议的修复优先级

1. **高优先级**：修复机械臂状态实时查询（方案1）
   - 影响：核心功能，安全相关
   - 难度：中等，需要在JAKA SDK中找到对应API

2. **中优先级**：修复底盘连接状态检测（方案2）
   - 影响：用户体验
   - 难度：低，纯软件逻辑

3. **低优先级**：前端显示优化（方案3）
   - 影响：仅改善显示
   - 难度：低

## 需要检查的API

在JAKA SDK文档或头文件中查找以下API：
- `get_robot_status()` - 获取机器人完整状态
- `is_powered_on()` - 查询是否上电
- `is_enabled()` - 查询是否使能
- `get_error_code()` - 获取错误码

如果SDK不提供单独的查询API，可能需要调用 `get_robot_status()` 获取完整状态结构体。

## 相关文件清单

### 机械臂状态相关
- `qyh_jushen_ws/src/qyh_jaka_control/include/qyh_jaka_control/jaka_interface.hpp`
- `qyh_jushen_ws/src/qyh_jaka_control/src/jaka_interface.cpp`
- `qyh_jushen_ws/src/qyh_jaka_control/src/jaka_control_node.cpp`

### 底盘状态相关
- `qyh_jushen_web/backend/app/ros2_bridge/bridge.py`
- `qyh_jushen_web/backend/app/api/chassis.py`

### 前端显示相关
- `qyh_jushen_web/frontend/src/layouts/MainLayout.vue`
- `qyh_jushen_web/frontend/src/stores/layout.ts`

## 测试方案

### 测试机械臂状态更新
1. 启动jaka_control_node
2. 观察Web界面状态栏显示"双臂已上电"
3. 在示教器上断电机械臂
4. 观察Web界面是否更新为"双臂未上电"

### 测试底盘状态更新
1. 启动qyh_standard_robot节点
2. 观察Web界面状态栏显示"底盘已连接"
3. 停止qyh_standard_robot节点
4. 等待2秒后观察Web界面是否更新为"底盘未连接"

## 下一步行动

1. ✅ 诊断完成：已找到根本原因
2. ⏳ 查阅JAKA SDK文档，确认可用的状态查询API
3. ⏳ 实现方案1：机械臂状态实时查询
4. ⏳ 实现方案2：底盘连接状态超时检测
5. ⏳ 测试验证修复效果
