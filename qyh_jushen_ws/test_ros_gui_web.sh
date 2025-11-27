#!/bin/bash
# ROS GUI Web 集成快速测试脚本

set -e

echo "================================================"
echo "  QYH Jushen - ROS GUI Web 集成测试"
echo "================================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1 已安装"
        return 0
    else
        echo -e "${RED}✗${NC} $1 未安装"
        return 1
    fi
}

# 1. 检查依赖
echo "步骤 1: 检查系统依赖..."
check_command "ros2" || { echo "请先安装 ROS2 Humble"; exit 1; }
check_command "python3" || { echo "请先安装 Python3"; exit 1; }
check_command "node" || { echo "请先安装 Node.js"; exit 1; }
check_command "npm" || { echo "请先安装 npm"; exit 1; }
echo ""

# 2. 检查 ROS 环境
echo "步骤 2: 检查 ROS2 环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}警告:${NC} ROS2 环境未配置"
    echo "尝试 source ROS2 环境..."
    source /opt/ros/humble/setup.bash
fi
echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
echo ""

# 3. 编译 ROS GUI 包
echo "步骤 3: 编译 ROS GUI 包..."
cd ~/qyh_jushen_ws/qyh_jushen_ws

if [ ! -d "install" ]; then
    echo "首次编译，这可能需要几分钟..."
fi

colcon build --packages-select \
    qyh_gripper_msgs \
    qyh_gripper_control \
    qyh_gripper_gui \
    qyh_jaka_control_msgs \
    qyh_jaka_control_gui \
    qyh_standard_robot_msgs \
    qyh_standard_robot_gui

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} ROS 包编译成功"
else
    echo -e "${RED}✗${NC} ROS 包编译失败"
    exit 1
fi

source install/setup.bash
echo ""

# 4. 检查后端
echo "步骤 4: 检查后端配置..."
cd ~/qyh_jushen_ws/qyh_jushen_web/backend

if [ ! -d "venv" ]; then
    echo "创建 Python 虚拟环境..."
    python3 -m venv venv
fi

source venv/bin/activate

echo "安装后端依赖..."
pip install -r requirements.txt -q

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓${NC} 后端依赖已安装"
else
    echo -e "${RED}✗${NC} 后端依赖安装失败"
    exit 1
fi
echo ""

# 5. 检查前端
echo "步骤 5: 检查前端配置..."
cd ~/qyh_jushen_ws/qyh_jushen_web/frontend

if [ ! -d "node_modules" ]; then
    echo "安装前端依赖..."
    npm install
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} 前端依赖已安装"
    else
        echo -e "${RED}✗${NC} 前端依赖安装失败"
        exit 1
    fi
else
    echo -e "${GREEN}✓${NC} 前端依赖已存在"
fi
echo ""

# 6. 启动服务
echo "================================================"
echo "  准备启动服务"
echo "================================================"
echo ""
echo "将在以下终端启动服务："
echo "  - 终端 1: 后端服务 (http://localhost:8000)"
echo "  - 终端 2: 前端服务 (http://localhost:3000)"
echo ""
read -p "按 Enter 继续，或 Ctrl+C 取消..."

# 启动后端（新终端）
echo ""
echo "启动后端服务..."
cd ~/qyh_jushen_ws/qyh_jushen_web/backend
gnome-terminal -- bash -c "
    source venv/bin/activate
    source ~/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash
    echo '========================================='
    echo '  后端服务启动中...'
    echo '  API 文档: http://localhost:8000/docs'
    echo '========================================='
    echo ''
    MOCK_MODE=true uvicorn app.main:app --reload
    exec bash
" &

sleep 3

# 启动前端（新终端）
echo "启动前端服务..."
cd ~/qyh_jushen_ws/qyh_jushen_web/frontend
gnome-terminal -- bash -c "
    echo '========================================='
    echo '  前端服务启动中...'
    echo '  访问地址: http://localhost:3000'
    echo '  默认账号: admin / admin123'
    echo '========================================='
    echo ''
    npm run dev
    exec bash
" &

sleep 3

# 7. 完成
echo ""
echo "================================================"
echo -e "${GREEN}✓ 服务启动完成！${NC}"
echo "================================================"
echo ""
echo "📝 快速开始："
echo ""
echo "1. 在浏览器中打开: ${GREEN}http://localhost:3000${NC}"
echo "2. 使用管理员账户登录:"
echo "   用户名: ${YELLOW}admin${NC}"
echo "   密码: ${YELLOW}admin123${NC}"
echo "3. 点击 Dashboard 上的 \"ROS GUI控制\" 按钮"
echo "4. 选择要启动的 GUI（夹爪/机械臂/底盘）"
echo "5. 点击 \"启动\" 按钮"
echo ""
echo "📚 API 文档: ${GREEN}http://localhost:8000/docs${NC}"
echo ""
echo "⚠️  注意事项："
echo "  - 确保对应的 ROS 控制节点已启动（如夹爪控制节点）"
echo "  - GUI 窗口将在桌面上打开（需要 X11 显示）"
echo "  - 可以在 Web 界面中停止 GUI"
echo ""
echo "📖 完整文档: ROS_GUI_WEB_INTEGRATION.md"
echo ""

# 打开浏览器
if command -v xdg-open &> /dev/null; then
    echo "5秒后自动打开浏览器..."
    sleep 5
    xdg-open http://localhost:3000 &
fi

echo ""
echo "================================================"
echo "  测试完成后，按 Ctrl+C 停止所有服务"
echo "================================================"
echo ""

# 等待用户中断
wait
