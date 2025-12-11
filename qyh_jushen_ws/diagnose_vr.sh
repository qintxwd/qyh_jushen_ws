#!/bin/bash
# VR 遥操作诊断脚本

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========== VR 遥操作诊断 =========="
echo ""

# Source ROS2 环境
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Source 工作空间
source "$HOME/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash"

# 读取 ROS_DOMAIN_ID
DOMAIN_ID_FILE="$HOME/qyh_jushen_ws/persistent/ros/ROS_DOMAIN_ID"
if [ -f "$DOMAIN_ID_FILE" ]; then
    export ROS_DOMAIN_ID=$(cat "$DOMAIN_ID_FILE")
    echo -e "${GREEN}✓ ROS_DOMAIN_ID: $ROS_DOMAIN_ID${NC}"
else
    export ROS_DOMAIN_ID=0
    echo -e "${YELLOW}⚠ 使用默认 ROS_DOMAIN_ID: 0${NC}"
fi

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