#!/bin/bash
#
# VR 遥操作真机启动脚本
# 用 PICO 4 VR 手柄控制真实 JAKA 双臂机器人
#
# 使用方法:
#   cd ~/qyh_jushen_ws/qyh_jushen_ws
#   ./src/start_vr_real_robot.sh [robot_ip]
#
# 参数:
#   robot_ip: JAKA 机器人 IP (默认: 192.168.2.200)
#
# ⚠️ 安全警告:
#   - 确保机械臂工作空间无障碍物
#   - 急停按钮在手边
#   - 首次使用低速测试
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

# 获取脚本所在目录的父目录作为工作空间
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo -e "${RED}${BOLD}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║              ⚠️  真机 VR 遥操作启动脚本  ⚠️                    ║"
echo "╠═══════════════════════════════════════════════════════════════╣"
echo "║  警告: 即将控制真实机械臂！                                   ║"
echo "║                                                               ║"
echo "║  请确认:                                                      ║"
echo "║  1. 机械臂工作空间内无人员和障碍物                            ║"
echo "║  2. 急停按钮在手边                                            ║"
echo "║  3. 已通过 Web 界面完成: 上电 → 使能 → 启动伺服               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""
echo -e "机器人 IP: ${GREEN}$ROBOT_IP${NC}"
echo -e "工作空间: ${GREEN}$WS_DIR${NC}"
echo ""

# 确认继续
read -p "确认继续? (y/N): " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}已取消${NC}"
    exit 0
fi

echo ""

# 检查网络连通性
echo -e "${YELLOW}[INFO] 检查机器人网络连通性...${NC}"
if ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}[OK] 机器人 $ROBOT_IP 可达${NC}"
else
    echo -e "${RED}[ERROR] 无法连接机器人 $ROBOT_IP${NC}"
    echo -e "${RED}请检查网络连接后重试${NC}"
    exit 1
fi

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

# 启动bringup 并将日志输出到文件
export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

# 检查必要的包
check_package() {
    if ! ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${RED}[ERROR] Package '$1' not found!${NC}"
        return 1
    fi
    return 0
}

echo -e "${YELLOW}[INFO] 检查必要的 ROS2 包...${NC}"
MISSING=0
for pkg in qyh_dual_arms_moveit_config qyh_vr_bridge qyh_vr_calibration qyh_teleoperation_controller qyh_jaka_control; do
    if ! check_package "$pkg"; then
        MISSING=1
    fi
done

if [ $MISSING -eq 1 ]; then
    echo -e "${RED}[ERROR] 缺少必要的包，请先编译${NC}"
    exit 1
fi
echo -e "${GREEN}[OK] 所有包已就绪${NC}"
echo ""

# 清理函数
cleanup() {
    # 恢复默认信号处理，允许再次 Ctrl+C 强制退出
    trap - SIGINT SIGTERM

    echo ""
    echo -e "${YELLOW}[INFO] 停止所有节点...${NC}"
    
    # 尝试优雅停止伺服 (增加超时限制)
    if command -v timeout >/dev/null 2>&1; then
        timeout 2s ros2 service call /jaka/bridge/stop_servo std_srvs/srv/Trigger >/dev/null 2>&1 || true
    else
        ros2 service call /jaka/bridge/stop_servo std_srvs/srv/Trigger >/dev/null 2>&1 || true
    fi
    
    # 杀死所有子进程
    pkill -P $$ 2>/dev/null || true
    
    sleep 1
    echo -e "${GREEN}[OK] 所有节点已停止${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo -e "${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    启动 ROS2 节点                              ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 1. 启动 MoveIt (用于 IK 和可视化)
echo -e "${GREEN}[1/5] 启动 MoveIt...${NC}"
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py &
MOVEIT_PID=$!
sleep 5

if ! kill -0 $MOVEIT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR] MoveIt 启动失败${NC}"
    exit 1
fi

# 2. 启动 VR Bridge
echo -e "${GREEN}[2/5] 启动 VR Bridge...${NC}"
ros2 run qyh_vr_bridge vr_bridge_node --ros-args -p grip_offset_deg:=35.0 &
VR_BRIDGE_PID=$!
sleep 2

# 3. 启动 VR Clutch (真机模式)
echo -e "${GREEN}[3/5] 启动 VR Clutch (真机模式)...${NC}"
ros2 launch qyh_vr_calibration vr_clutch.launch.py simulation_mode:=false &
VR_CLUTCH_PID=$!
sleep 2

# 4. 启动 Teleoperation Controller
echo -e "${GREEN}[4/5] 启动 Teleoperation Controller...${NC}"
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py &
TELEOP_PID=$!
sleep 2

# 5. 启动 JAKA Bridge
echo -e "${GREEN}[5/5] 启动 JAKA Bridge...${NC}"
ros2 launch qyh_jaka_control jaka_bridge.launch.py robot_ip:=$ROBOT_IP &
JAKA_PID=$!
sleep 3

echo ""
echo -e "${BLUE}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                    所有节点已启动!                            ║${NC}"
echo -e "${BLUE}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "节点状态:"
echo -e "  - MoveIt:            PID ${MOVEIT_PID}"
echo -e "  - VR Bridge:         PID ${VR_BRIDGE_PID}"
echo -e "  - VR Clutch:         PID ${VR_CLUTCH_PID}"
echo -e "  - Teleoperation:     PID ${TELEOP_PID}"
echo -e "  - JAKA Bridge:       PID ${JAKA_PID}"
echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}操作前请确认:${NC}"
echo -e "  1. Web 界面已完成: ${BOLD}上电 → 使能 → 启动伺服${NC}"
echo -e "  2. VR 头显已连接并启动应用"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${GREEN}使用方法:${NC}"
echo -e "  1. 戴上 PICO VR 头显"
echo -e "  2. 按住 Grip 键 → 机械臂跟随"
echo -e "  3. 松开 Grip 键 → 机械臂保持"
echo ""
echo -e "${YELLOW}验证命令:${NC}"
echo -e "  ros2 topic hz /vr/left_hand/pose"
echo -e "  ros2 topic echo /vr/left_clutch_engaged"
echo -e "  ros2 topic hz /left_arm/joint_command"
echo ""
echo -e "${RED}${BOLD}按 Ctrl+C 停止所有节点${NC}"
echo ""

# 等待
wait
