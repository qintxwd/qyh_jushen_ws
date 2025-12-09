#!/bin/bash
#
# VR 遥操作仿真启动脚本
# 用 PICO 4 VR 手柄控制仿真双机械臂
#
# 使用方法:
#   cd ~/qyh_jushen_ws
#   ./start_vr_simulation.sh
#
# 控制方式: Clutch 离合器模式
#   - 按住 Grip 键: 机械臂跟随 VR 手柄移动
#   - 松开 Grip 键: 机械臂保持当前位置
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录作为工作空间
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   VR 遥操作仿真系统启动脚本${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
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
    echo -e "${RED}[ERROR] Workspace not built! Run:${NC}"
    echo -e "  cd $WS_DIR"
    echo -e "  colcon build --packages-up-to qyh_vr_bridge qyh_vr_calibration qyh_dual_arms_moveit_config"
    exit 1
fi

# 检查必要的包是否存在
check_package() {
    if ! ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${RED}[ERROR] Package '$1' not found!${NC}"
        return 1
    fi
    return 0
}

echo -e "${YELLOW}[INFO] Checking required packages...${NC}"
MISSING=0
for pkg in qyh_dual_arms_moveit_config qyh_vr_bridge qyh_vr_calibration; do
    if ! check_package "$pkg"; then
        MISSING=1
    fi
done

if [ $MISSING -eq 1 ]; then
    echo -e "${RED}[ERROR] Some packages are missing. Please build them first.${NC}"
    exit 1
fi

echo -e "${GREEN}[OK] All packages found.${NC}"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}[INFO] Shutting down all nodes...${NC}"
    
    # 杀死所有子进程
    pkill -P $$ 2>/dev/null || true
    
    # 等待进程结束
    sleep 1
    
    echo -e "${GREEN}[OK] All nodes stopped.${NC}"
    exit 0
}

# 注册清理函数
trap cleanup SIGINT SIGTERM

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   启动节点 (按 Ctrl+C 停止所有)${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 启动 MoveIt + RViz
echo -e "${GREEN}[1/4] Starting MoveIt + RViz...${NC}"
ros2 launch qyh_dual_arms_moveit_config demo.launch.py &
MOVEIT_PID=$!
sleep 5  # 等待 MoveIt 完全启动

# 检查 MoveIt 是否启动成功
if ! kill -0 $MOVEIT_PID 2>/dev/null; then
    echo -e "${RED}[ERROR] MoveIt failed to start!${NC}"
    exit 1
fi

# 启动 VR Bridge
echo -e "${GREEN}[2/4] Starting VR Bridge...${NC}"
ros2 run qyh_vr_bridge vr_bridge_node --ros-args -p grip_offset_deg:=35.0 &
VR_BRIDGE_PID=$!
sleep 2

# 启动 VR Clutch
echo -e "${GREEN}[3/4] Starting VR Clutch...${NC}"
ros2 launch qyh_vr_calibration vr_clutch.launch.py &
VR_CLUTCH_PID=$!
sleep 2

# 启动仿真机械臂控制器
echo -e "${GREEN}[4/4] Starting Sim Arm Controller...${NC}"
ros2 run qyh_vr_calibration sim_arm_controller &
SIM_ARM_PID=$!
sleep 2

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}   所有节点已启动!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "节点状态:"
echo -e "  - MoveIt + RViz:     PID ${MOVEIT_PID}"
echo -e "  - VR Bridge:         PID ${VR_BRIDGE_PID}"
echo -e "  - VR Clutch:         PID ${VR_CLUTCH_PID}"
echo -e "  - Sim Arm Controller: PID ${SIM_ARM_PID}"
echo ""
echo -e "${YELLOW}使用方法:${NC}"
echo -e "  1. 戴上 PICO VR 头显"
echo -e "  2. 按住 Grip 键 (侧面握键)"
echo -e "  3. 移动手柄 → 机械臂跟随"
echo -e "  4. 松开 Grip → 机械臂停止"
echo ""
echo -e "${YELLOW}验证数据流:${NC}"
echo -e "  ros2 topic hz /vr/left_hand/pose"
echo -e "  ros2 topic echo /vr/left_clutch_engaged"
echo -e "  ros2 topic echo /sim/left_target_pose --once"
echo ""
echo -e "${RED}按 Ctrl+C 停止所有节点${NC}"
echo ""

# 等待任意子进程退出
wait
