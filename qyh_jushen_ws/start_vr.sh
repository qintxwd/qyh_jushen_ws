#!/bin/bash
#
# VR 遥操作启动脚本
# 启动 VR Bridge + Teleop + JAKA Control
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# 默认参数
ROBOT_IP="${1:-192.168.2.200}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo -e "${BLUE}${BOLD}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                    VR 遥操作系统启动                          ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo -e "机器人 IP: ${GREEN}$ROBOT_IP${NC}"
echo -e "工作空间: ${GREEN}$WS_DIR${NC}"
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}[INFO] Sourcing ROS2 Humble...${NC}"
    source /opt/ros/humble/setup.bash
fi

# Source 工作空间
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo -e "${YELLOW}[INFO] Sourcing workspace...${NC}"
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${RED}[ERROR] Workspace not built!${NC}"
    echo -e "Run: cd $WS_DIR && colcon build"
    exit 1
fi

# 设置日志格式
export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [{severity}] [{name}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

# 清理函数
cleanup() {
    trap - SIGINT SIGTERM
    echo ""
    echo -e "${YELLOW}[INFO] 停止所有节点...${NC}"
    pkill -P $$ 2>/dev/null || true
    sleep 1
    echo -e "${GREEN}[OK] 所有节点已停止${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo -e "${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                      启动 ROS2 节点                           ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 1. 启动 VR Bridge
echo -e "${GREEN}[1/3] 启动 VR Bridge...${NC}"
ros2 run qyh_vr_bridge vr_bridge_node &
VR_BRIDGE_PID=$!
sleep 2

# 2. 启动 Teleop
echo -e "${GREEN}[2/3] 启动 Teleop...${NC}"
ros2 launch qyh_dual_arm_teleop teleop.launch.py &
TELEOP_PID=$!
sleep 2

# # 3. 启动 JAKA Control
# echo -e "${GREEN}[3/3] 启动 JAKA Control...${NC}"
# ros2 launch qyh_jaka_control jaka_control.launch.py robot_ip:=$ROBOT_IP &
# JAKA_PID=$!
# sleep 3

echo ""
echo -e "${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                      所有节点已启动!                          ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "节点状态:"
echo -e "  - VR Bridge:    PID ${VR_BRIDGE_PID}"
echo -e "  - Teleop:       PID ${TELEOP_PID}"
echo -e "  - JAKA Control: PID ${JAKA_PID}"
echo ""
echo -e "${YELLOW}数据流:${NC}"
echo -e "  VR → vr_bridge → teleop → jaka_control → Robot"
echo ""
echo -e "${GREEN}使用方法:${NC}"
echo -e "  1. 戴上 PICO VR 头显"
echo -e "  2. 按住 Grip 键 → 机械臂跟随"
echo -e "  3. 松开 Grip 键 → 机械臂保持"
echo ""
echo -e "${RED}${BOLD}按 Ctrl+C 停止所有节点${NC}"
echo ""

# 等待
wait
