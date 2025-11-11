# ⚠️ 重要说明：关节数组必须是14个值

## 概述

**所有涉及关节位置的服务调用都必须提供14个关节值，而不是7个！**

即使你只想控制单个机械臂（左臂或右臂），也必须提供完整的14个关节值。

## 为什么需要14个值？

JAKA双臂机器人SDK的底层实现要求所有运动指令必须包含两个臂的完整关节信息：

```cpp
// SDK内部结构
JointValue jpos[2];  // 2个机械臂，每个7个关节
// jpos[0] = 左臂 (关节1-7)
// jpos[1] = 右臂 (关节1-7)
```

SDK示例代码（如 `23.multi_movj.cpp`, `30.edgservo.cpp`）都使用这种结构，即使是控制单个臂，也会填充完整的14个值。

## 关节数组格式

### 格式说明
```
[左臂关节1, 左臂关节2, ..., 左臂关节7, 右臂关节1, 右臂关节2, ..., 右臂关节7]
 ←──────────── 左臂(7个) ────────────→  ←──────────── 右臂(7个) ────────────→
```

### 示例

#### 只控制左臂
```python
# 左臂移动到指定位置，右臂保持零位
joint_positions = [
    0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0,  # 左臂运动
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0     # 右臂保持零位
]
```

#### 只控制右臂
```python
# 左臂保持零位，右臂移动到指定位置
joint_positions = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # 左臂保持零位
    0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0   # 右臂运动
]
```

#### 双臂同步运动
```python
# 两个臂同步运动到对称位置
joint_positions = [
    0.0, 0.3, 0.0, -1.2, 0.0, -0.5, 0.0,  # 左臂
    0.0, 0.3, 0.0, -1.2, 0.0, -0.5, 0.0   # 右臂（镜像运动）
]
```

## 影响的服务

以下服务都需要14个关节值：

### 1. MoveJ (关节空间运动)
```bash
ros2 service call /jaka_robot_node/move_j jaka_control/srv/MoveJ "{
  robot_id: 0,
  joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  move_mode: false,
  velocity: 0.5,
  acceleration: 1.0,
  is_block: true
}"
```

### 2. ServoJ (伺服关节运动)
```bash
ros2 service call /jaka_robot_node/servo_j jaka_control/srv/ServoJ "{
  robot_id: 0,
  joint_positions: [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  move_mode: false
}"
```

## Python示例

### 完整的Python调用示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from jaka_control.srv import MoveJ

def main():
    rclpy.init()
    node = Node('test_node')
    client = node.create_client(MoveJ, '/jaka_robot_node/move_j')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('等待服务...')
    
    # 创建请求 - 必须是14个值！
    request = MoveJ.Request()
    request.robot_id = 0  # 控制左臂
    
    # 左臂运动，右臂保持当前位置（这里用零位演示）
    request.joint_positions = [
        0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0,  # 左臂 (7个)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0     # 右臂 (7个)
    ]
    
    request.move_mode = False
    request.velocity = 0.5
    request.acceleration = 1.0
    request.is_block = True
    
    # 调用服务
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    node.get_logger().info(f'运动结果: {result.message}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 常见错误

### ❌ 错误示例：只提供7个值
```python
# 这是错误的！！！
joint_positions = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]  # 只有7个值
```

**错误信息**：
```
Joint positions must have 14 values (7 for each arm)
```

### ✅ 正确示例：提供完整的14个值
```python
# 这是正确的
joint_positions = [
    0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0,  # 左臂
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0     # 右臂
]
```

## robot_id参数的作用

`robot_id` 参数决定哪个臂会执行运动，但**不影响数组大小要求**：

- `robot_id = 0` (LEFT): 只有左臂运动，但仍需提供14个值
- `robot_id = 1` (RIGHT): 只有右臂运动，但仍需提供14个值  
- `robot_id = -1` (DUAL): 两个臂同时运动，需提供14个值

## 实际应用建议

### 1. 获取当前位置后修改
最安全的做法是先获取当前关节位置，然后只修改需要运动的臂：

```python
# 获取当前位置（假设有获取关节状态的服务）
current_joints = get_current_joint_positions()  # 返回14个值

# 只修改左臂的某些关节
current_joints[0] = 0.5  # 修改左臂关节1
current_joints[3] = -1.57  # 修改左臂关节4

# 发送完整的14个值
move_j(0, current_joints)
```

### 2. 使用辅助函数
创建辅助函数简化调用：

```python
def move_left_arm(joint_angles_left):
    """
    只移动左臂，右臂保持零位
    
    Args:
        joint_angles_left: 左臂7个关节角度
    """
    full_joints = joint_angles_left + [0.0] * 7
    return move_j(0, full_joints)

def move_right_arm(joint_angles_right):
    """
    只移动右臂，左臂保持零位
    
    Args:
        joint_angles_right: 右臂7个关节角度
    """
    full_joints = [0.0] * 7 + joint_angles_right
    return move_j(1, full_joints)

# 使用
move_left_arm([0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0])
```

## 技术原因

这个设计来自SDK的EtherCAT实时通信实现。SDK使用同步周期下发（edg_servo_j_fct），需要同时更新两个臂的状态，即使某个臂不运动。这确保了：

1. **实时性**: 两个臂的控制周期同步
2. **数据一致性**: 避免某个臂的数据过期
3. **安全性**: 系统始终知道两个臂的完整状态

## 总结

**记住这一条规则**：

> 📌 **所有关节运动指令都需要14个关节值：前7个是左臂，后7个是右臂。无论你控制哪个臂，都必须提供完整的14个值！**

如有疑问，请参考SDK示例代码：
- `资料/双机械臂/SDK-2.3.0.5/SDK/samples/30.edgservo.cpp`
- `资料/双机械臂/SDK-2.3.0.5/SDK/samples/23.multi_movj.cpp`
