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

 ros2 service call /jaka/servo/stop qyh_jaka_control_msgs/srv/StopServo "{}"