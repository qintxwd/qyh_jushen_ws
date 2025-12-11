# VR 遥操作真机故障排查指南

## ⚡ 最常见原因 (先检查这里!)

**80% 的问题是伺服模式未启动！**

```bash
# 1. 检查伺服状态
ros2 topic echo /jaka/servo/status --once

# 2. 如果 is_running: false，启动伺服模式:
ros2 service call /jaka/bridge/start_servo std_srvs/srv/Trigger

# 或者使用完整的服务接口:
ros2 service call /jaka/start_servo qyh_jaka_control_msgs/srv/StartServo
```

**前提条件** - 启动伺服前必须满足:
1. 机器人已连接 (`connected: true`)
2. 机器人已上电 (`powered: true`) 
3. 机器人已使能 (`enabled: true`)

```bash
# 检查机器人状态
ros2 topic echo /jaka/robot_state --once

# 如果未使能，先使能机器人:
ros2 service call /jaka/enable_robot std_srvs/srv/SetBool "{data: true}"
```

**工作原理**: `jaka_control_node` 中的 `servo_running_` 标志控制是否处理关节指令：
```cpp
void leftBridgeCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (servo_running_ && msg->position.size() >= 7) {  // 必须为 true!
        // ... 处理指令
    }
    // 否则静默忽略所有指令!
}
```

---

## 数据流概述

```
PICO VR 头显 (UDP:9999)
       │
       ▼
┌──────────────────┐
│  vr_bridge_node  │  ← 接收 VR 原始数据，坐标变换
│   /vr/*/pose     │
│   /vr/*/joy      │
└──────────────────┘
       │
       ▼
┌──────────────────┐
│  vr_clutch_node  │  ← Clutch 逻辑: grip按下=跟踪, 松开=保持
│   /vr/*_target   │
│   /vr/*_clutch   │
└──────────────────┘
       │
       ▼
┌──────────────────────────┐
│ teleoperation_controller │  ← IK 计算: 末端位姿 → 关节角度
│   /left_arm/joint_cmd    │
│   /right_arm/joint_cmd   │
└──────────────────────────┘
       │
       ▼
┌──────────────────────┐
│  jaka_control_node   │  ← 发送到机械臂
│  (伺服模式运行)       │
└──────────────────────┘
       │
       ▼
    JAKA 机械臂
```

---

## 🔍 排查步骤

### 第1步：检查 VR 头显数据是否到达

```bash
# 检查 vr_bridge_node 是否在接收数据
ros2 topic hz /vr/left_hand/pose
ros2 topic hz /vr/right_hand/pose
```

**预期结果**: 约 72Hz (PICO 的刷新率)

**如果无数据**:
- 检查 PICO VR 是否启动遥操作应用
- 检查 UDP 端口 9999 是否被占用
- 检查 VR 和机器人是否在同一网络
- 检查防火墙: `sudo ufw allow 9999/udp`

```bash
# 查看 vr_bridge_node 日志
ros2 node info /vr_bridge_node
```

---

### 第2步：检查 Grip 按键数据

```bash
# 查看 Joy 消息 (axes[3] 是 grip 值)
ros2 topic echo /vr/left_hand/joy

# 或只看频率
ros2 topic hz /vr/left_hand/joy
```

**预期结果**: 
- `axes[3]` (grip) 应该在 0.0-1.0 之间变化
- 按下 grip 时应该接近 1.0

**Joy axes 索引说明**:
- `axes[0]`: trigger (扳机)
- `axes[1]`: thumbstick X
- `axes[2]`: thumbstick Y  
- `axes[3]`: grip (握把) ← Clutch 使用这个

**如果 grip 值不变**:
- VR 应用可能没有正确发送按键数据
- 检查 VR 手柄电量

---

### 第3步：检查 Clutch 是否激活

```bash
# 查看 clutch 状态
ros2 topic echo /vr/left_clutch_engaged
ros2 topic echo /vr/right_clutch_engaged
```

**预期结果**: 按下 grip 时应该显示 `data: true`

**如果一直是 false**:
```bash
# 检查 vr_clutch_node 是否运行
ros2 node list | grep clutch

# 查看节点详情
ros2 node info /vr_clutch_node
```

**调试 vr_clutch_node**:
```bash
# 查看日志
ros2 launch qyh_vr_calibration vr_clutch.launch.py simulation_mode:=false

# 检查参数
ros2 param get /vr_clutch_node grip_threshold  # 默认 0.5
```

---

### 第4步：检查目标位姿发布

```bash
# 查看 vr_clutch_node 是否发布目标位姿
ros2 topic hz /vr/left_target_pose
ros2 topic hz /vr/right_target_pose

# 查看具体位姿值
ros2 topic echo /vr/left_target_pose
```

**预期结果**: Clutch 激活时应该有持续的位姿发布

**如果无目标位姿**:
- vr_clutch_node 可能没收到机器人当前状态
```bash
# 检查是否收到机器人状态
ros2 topic echo /jaka/robot_state
ros2 topic hz /joint_states
```

---

### 第5步：检查 IK 关节指令

```bash
# 查看 teleoperation_controller 输出
ros2 topic hz /left_arm/joint_command
ros2 topic hz /right_arm/joint_command

# 查看具体指令
ros2 topic echo /left_arm/joint_command
```

**预期结果**: 应该有约 125Hz 的关节指令

**如果无关节指令**:
```bash
# 检查 teleoperation_controller 是否运行
ros2 node list | grep teleoperation

# 检查 MoveIt 是否正常
ros2 service list | grep compute_ik
```

**IK 可能失败的原因**:
- 目标位姿超出机械臂工作空间
- MoveIt 配置问题
- 查看日志: `ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py`

---

### 第6步：检查 JAKA 伺服模式

```bash
# 检查伺服状态
ros2 topic echo /jaka/servo/status
```

**预期结果**: `is_running: true`

**⚠️ 这是最常见的问题！**

即使通过 Web 界面点击了"启动伺服"，也需要确认 ROS 节点的伺服状态：

```bash
# 启动伺服模式 (方法1: 标准服务)
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo "{}"

# 启动伺服模式 (方法2: Bridge 兼容服务)
ros2 service call /jaka/bridge/start_servo std_srvs/srv/Trigger

# 验证伺服已启动
ros2 topic echo /jaka/servo/status --once
```

**关键代码解释**:
`jaka_control_node` 中的 `leftBridgeCallback` 有这个检查：
```cpp
void leftBridgeCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (servo_running_ && msg->position.size() >= 7) {  // ← 必须 servo_running_ 为 true!
        // ... 处理指令
    }
}
```

如果 `servo_running_` 为 `false`，所有关节指令都会被忽略！

---

### 第7步：检查话题连接 (关键!)

```bash
# 检查 teleoperation_controller 发布的话题
ros2 topic info /left_arm/joint_command

# 检查 jaka_control_node 订阅的话题
ros2 node info /jaka_control_node
```

**常见问题**: 话题名称不匹配！

如果 `jaka_control_node` 订阅的是 `/jaka/servo/joint_cmd`，但 `teleoperation_controller` 发布的是 `/left_arm/joint_command`，则需要添加桥接或修改配置。

---

### 第8步：检查机器人连接和使能状态

```bash
# 检查机器人状态
ros2 topic echo /jaka/robot_state

# 确认:
# - connected: true
# - powered: true  
# - enabled: true
# - servo_mode: true (伺服模式)
```

**通过服务检查**:
```bash
# 获取完整状态
ros2 service call /jaka/get_robot_state qyh_jaka_control_msgs/srv/GetRobotState
```

---

## 📋 快速诊断脚本

在机器人端创建诊断脚本 `~/diagnose_vr.sh`:

```bash
#!/bin/bash
# VR 遥操作诊断脚本

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========== VR 遥操作诊断 =========="
echo ""

# 1. VR 数据
echo -e "${YELLOW}[1] VR 数据检查...${NC}"
VR_HZ=$(timeout 2 ros2 topic hz /vr/left_hand/pose 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$VR_HZ" ]; then
    echo -e "  ${GREEN}✓ /vr/left_hand/pose: ${VR_HZ} Hz${NC}"
else
    echo -e "  ${RED}✗ /vr/left_hand/pose 无数据${NC}"
fi

JOY_HZ=$(timeout 2 ros2 topic hz /vr/left_hand/joy 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$JOY_HZ" ]; then
    echo -e "  ${GREEN}✓ /vr/left_hand/joy: ${JOY_HZ} Hz${NC}"
else
    echo -e "  ${RED}✗ /vr/left_hand/joy 无数据${NC}"
fi

# 2. Clutch 状态
echo ""
echo -e "${YELLOW}[2] Clutch 状态检查...${NC}"
CLUTCH=$(timeout 1 ros2 topic echo /vr/left_clutch_engaged --once 2>/dev/null | grep "data:")
if [ -n "$CLUTCH" ]; then
    echo -e "  ${GREEN}✓ Clutch 话题正常: $CLUTCH${NC}"
else
    echo -e "  ${RED}✗ Clutch 话题无数据${NC}"
fi

# 3. 目标位姿
echo ""
echo -e "${YELLOW}[3] 目标位姿检查 (按下 Grip 后应有数据)...${NC}"
TARGET_HZ=$(timeout 3 ros2 topic hz /vr/left_target_pose 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$TARGET_HZ" ]; then
    echo -e "  ${GREEN}✓ /vr/left_target_pose: ${TARGET_HZ} Hz${NC}"
else
    echo -e "  ${YELLOW}⚠ /vr/left_target_pose 无数据 (可能 Clutch 未激活)${NC}"
fi

# 4. 关节指令
echo ""
echo -e "${YELLOW}[4] 关节指令检查...${NC}"
CMD_HZ=$(timeout 3 ros2 topic hz /left_arm/joint_command 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$CMD_HZ" ]; then
    echo -e "  ${GREEN}✓ /left_arm/joint_command: ${CMD_HZ} Hz${NC}"
else
    echo -e "  ${YELLOW}⚠ /left_arm/joint_command 无数据${NC}"
fi

# 5. 伺服状态 (关键!)
echo ""
echo -e "${YELLOW}[5] 伺服状态检查 (关键!)...${NC}"
SERVO_STATUS=$(timeout 1 ros2 topic echo /jaka/servo/status --once 2>/dev/null)
if [ -n "$SERVO_STATUS" ]; then
    IS_RUNNING=$(echo "$SERVO_STATUS" | grep "is_running:" | awk '{print $2}')
    if [ "$IS_RUNNING" == "true" ]; then
        echo -e "  ${GREEN}✓ 伺服模式: 运行中${NC}"
    else
        echo -e "  ${RED}✗ 伺服模式: 未运行！这是问题所在！${NC}"
        echo -e "  ${YELLOW}  → 请运行: ros2 service call /jaka/bridge/start_servo std_srvs/srv/Trigger${NC}"
    fi
else
    echo -e "  ${RED}✗ 无法获取伺服状态${NC}"
fi

# 6. 机器人状态
echo ""
echo -e "${YELLOW}[6] 机器人状态检查...${NC}"
ROBOT_STATE=$(timeout 1 ros2 topic echo /jaka/robot_state --once 2>/dev/null)
if [ -n "$ROBOT_STATE" ]; then
    CONNECTED=$(echo "$ROBOT_STATE" | grep "connected:" | head -1 | awk '{print $2}')
    POWERED=$(echo "$ROBOT_STATE" | grep "powered:" | awk '{print $2}')
    ENABLED=$(echo "$ROBOT_STATE" | grep "enabled:" | awk '{print $2}')
    
    [ "$CONNECTED" == "true" ] && echo -e "  ${GREEN}✓ 已连接${NC}" || echo -e "  ${RED}✗ 未连接${NC}"
    [ "$POWERED" == "true" ] && echo -e "  ${GREEN}✓ 已上电${NC}" || echo -e "  ${RED}✗ 未上电${NC}"
    [ "$ENABLED" == "true" ] && echo -e "  ${GREEN}✓ 已使能${NC}" || echo -e "  ${RED}✗ 未使能${NC}"
else
    echo -e "  ${RED}✗ 无法获取机器人状态${NC}"
fi

# 7. 节点检查
echo ""
echo -e "${YELLOW}[7] 关键节点检查...${NC}"
NODES=$(ros2 node list 2>/dev/null)
for node in "vr_bridge" "vr_clutch" "teleoperation" "jaka_control"; do
    if echo "$NODES" | grep -q "$node"; then
        echo -e "  ${GREEN}✓ $node 节点运行中${NC}"
    else
        echo -e "  ${RED}✗ $node 节点未运行${NC}"
    fi
done

echo ""
echo "========== 诊断完成 =========="
```

使用方法：
```bash
chmod +x ~/diagnose_vr.sh
~/diagnose_vr.sh
```

---

## 🔧 常见问题解决

### 问题1: VR 数据正常，但机械臂不动

**原因**: 话题名称不匹配

**解决方案**: 检查并修改配置，或使用 topic remap:

```bash
# 启动 jaka_control_node 时重映射话题
ros2 launch qyh_jaka_control jaka_control.launch.py \
    --ros-args -r /left_arm/joint_command:=/jaka/servo/joint_cmd
```

或者修改 `jaka_control_node` 使其订阅正确的话题。

### 问题2: Grip 按下但 clutch 不激活

**原因**: grip 阈值设置过高

**解决方案**:
```bash
ros2 param set /vr_clutch_node grip_threshold 0.3
```

### 问题3: 目标位姿发布但无关节指令

**原因**: IK 计算失败（目标超出工作空间）

**解决方案**:
- 检查 VR 标定位置
- 减小位置缩放系数
- 查看 teleoperation_controller 日志

### 问题4: 关节指令发布但机械臂不动

**原因**: 伺服模式未正确启动

**解决方案**:
```bash
# 重新启动伺服
ros2 service call /jaka/servo/stop std_srvs/srv/Trigger
sleep 1
ros2 service call /jaka/servo/start qyh_jaka_control_msgs/srv/StartServo "{use_cartesian: false}"
```

### 问题5: 机械臂抖动或不平滑

**原因**: 平滑滤波参数不合适

**解决方案**:
```bash
ros2 param set /jaka_control_node default_filter_cutoff 0.5
```

---

## 📊 话题完整列表

| 节点 | 发布话题 | 订阅话题 |
|------|----------|----------|
| vr_bridge_node | `/vr/*/pose`, `/vr/*/joy` | UDP:9999 |
| vr_clutch_node | `/vr/*_target_pose`, `/vr/*_clutch_engaged` | `/vr/*/pose`, `/vr/*/joy`, `/jaka/robot_state` |
| teleoperation_controller | `/left_arm/joint_command`, `/right_arm/joint_command` | `/vr/*_target_pose`, `/vr/*_clutch_engaged`, `/joint_states` |
| jaka_control_node | `/joint_states`, `/jaka/robot_state`, `/jaka/servo/status` | `/jaka/servo/joint_cmd` 或 `/*/joint_command` |

---

## 🆘 如果还是不工作

1. **重启所有节点**:
```bash
# Ctrl+C 停止所有
# 然后重新启动
./start_vr_real_robot.sh
```

2. **检查 ROS_DOMAIN_ID**:
```bash
echo $ROS_DOMAIN_ID  # 应该一致
```

3. **收集完整日志**:
```bash
ros2 launch qyh_jaka_control jaka_control.launch.py robot_ip:=192.168.2.200 2>&1 | tee jaka.log
```

4. **联系开发者**，提供以上诊断信息。
