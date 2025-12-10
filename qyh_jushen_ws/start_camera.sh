#!/bin/bash
# 相机启动脚本 - 启动 Orbbec 相机和 web_video_server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    相机系统启动脚本${NC}"
echo -e "${BLUE}========================================${NC}"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# 读取 ROS_DOMAIN_ID
DOMAIN_ID_FILE="$HOME/qyh_jushen_ws/persistent/ros/ROS_DOMAIN_ID"
if [ -f "$DOMAIN_ID_FILE" ]; then
    export ROS_DOMAIN_ID=$(cat "$DOMAIN_ID_FILE")
    echo -e "${GREEN}✓ ROS_DOMAIN_ID: $ROS_DOMAIN_ID${NC}"
else
    export ROS_DOMAIN_ID=0
    echo -e "${YELLOW}⚠ 使用默认 ROS_DOMAIN_ID: 0${NC}"
fi

# 相机配置
HEAD_CAMERA_IP="192.168.1.10"
HEAD_CAMERA_SERIAL=""  # 可选：指定序列号

# 检查 web_video_server 是否已安装
check_web_video_server() {
    if ros2 pkg list | grep -q "web_video_server"; then
        echo -e "${GREEN}✓ web_video_server 已安装${NC}"
        return 0
    else
        echo -e "${RED}✗ web_video_server 未安装${NC}"
        echo -e "${YELLOW}  安装方法: sudo apt install ros-humble-web-video-server${NC}"
        return 1
    fi
}

# 检查相机连接
check_camera_connection() {
    echo -e "${BLUE}检查相机连接...${NC}"
    
    # 检查 USB 相机
    if lsusb | grep -i "orbbec\|astra\|2bc5" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 检测到 USB 相机${NC}"
        return 0
    fi
    
    # 检查网络相机
    if ping -c 1 -W 1 "$HEAD_CAMERA_IP" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ 检测到网络相机 ($HEAD_CAMERA_IP)${NC}"
        return 0
    fi
    
    echo -e "${YELLOW}⚠ 未检测到相机，将继续启动（相机可能稍后连接）${NC}"
    return 0
}

# 启动头部相机 (Orbbec Gemini 335Le)
start_head_camera() {
    echo -e "${BLUE}启动头部相机...${NC}"
    # 启动bringup 并将日志输出到文件
    export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
    # Orbbec 相机启动参数
    ros2 launch orbbec_camera gemini_330_series.launch.py \
        camera_name:=head_camera \
        usb_port:="" \
        device_num:=1 \
        enable_colored_point_cloud:=false \
        depth_registration:=true \
        enable_point_cloud:=false \
        color_width:=640 \
        color_height:=480 \
        color_fps:=30 \
        depth_width:=640 \
        depth_height:=480 \
        depth_fps:=30 \
        &
    
    HEAD_CAMERA_PID=$!
    echo -e "${GREEN}✓ 头部相机已启动 (PID: $HEAD_CAMERA_PID)${NC}"
}

# 启动 web_video_server
start_web_video_server() {
    echo -e "${BLUE}启动 web_video_server...${NC}"
    
    # 等待相机话题可用
    sleep 3

    # 启动bringup 并将日志输出到文件
    export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
    
    ros2 run web_video_server web_video_server \
        --ros-args \
        -p port:=8080 \
        -p server_threads:=4 \
        -p ros_threads:=2 \
        &
    
    WEB_VIDEO_PID=$!
    echo -e "${GREEN}✓ web_video_server 已启动 (PID: $WEB_VIDEO_PID)${NC}"
    echo -e "${GREEN}  访问地址: http://localhost:8080${NC}"
    echo -e "${GREEN}  视频流:   http://localhost:8080/stream?topic=/head_camera/color/image_raw${NC}"
}

# 显示可用话题
show_available_topics() {
    echo -e "${BLUE}等待相机话题...${NC}"
    sleep 5
    
    echo -e "${BLUE}可用的图像话题:${NC}"
    ros2 topic list | grep -E "image|camera" || echo "  (暂无图像话题)"
}

# 清理函数
cleanup() {
    echo -e "\n${YELLOW}正在关闭相机系统...${NC}"
    
    # 终止所有相关进程
    if [ ! -z "$HEAD_CAMERA_PID" ]; then
        kill $HEAD_CAMERA_PID 2>/dev/null || true
    fi
    if [ ! -z "$WEB_VIDEO_PID" ]; then
        kill $WEB_VIDEO_PID 2>/dev/null || true
    fi
    
    # 终止所有相关节点
    pkill -f "orbbec_camera" 2>/dev/null || true
    pkill -f "web_video_server" 2>/dev/null || true
    
    echo -e "${GREEN}✓ 相机系统已关闭${NC}"
    exit 0
}

# 注册清理函数
trap cleanup SIGINT SIGTERM

# 主流程
main() {
    echo ""
    
    # 检查依赖
    check_web_video_server || exit 1
    check_camera_connection
    
    echo ""
    echo -e "${BLUE}启动相机节点...${NC}"
    
    # 启动相机
    start_head_camera
    
    # 启动 web_video_server
    start_web_video_server
    
    # 显示可用话题
    show_available_topics
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  相机系统启动完成!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${BLUE}可用服务:${NC}"
    echo -e "  - 头部相机话题: /head_camera/color/image_raw"
    echo -e "  - 深度图话题:   /head_camera/depth/image_raw"
    echo -e "  - 视频流服务:   http://localhost:8080"
    echo ""
    echo -e "${YELLOW}按 Ctrl+C 停止所有服务${NC}"
    echo ""
    
    # 保持运行
    wait
}

# 显示帮助
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help     显示帮助信息"
    echo "  --head-only    只启动头部相机"
    echo "  --web-only     只启动 web_video_server (假设相机已启动)"
    echo ""
    echo "相机配置:"
    echo "  头部相机: Orbbec Gemini 335Le (Ethernet: $HEAD_CAMERA_IP)"
    echo ""
}

# 解析参数
case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    --head-only)
        source /opt/ros/humble/setup.bash
        [ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
        start_head_camera
        wait
        ;;
    --web-only)
        source /opt/ros/humble/setup.bash
        [ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
        start_web_video_server
        wait
        ;;
    *)
        main
        ;;
esac
