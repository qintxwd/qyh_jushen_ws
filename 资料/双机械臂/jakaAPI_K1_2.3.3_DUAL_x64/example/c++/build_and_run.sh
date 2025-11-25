#!/bin/bash
# JAKA 2.3.3 伺服测试快速编译和运行脚本

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}JAKA SDK 2.3.3 伺服测试程序${NC}"
echo -e "${BLUE}========================================${NC}"

# 检查当前目录
if [ ! -f "30.edgservo.cpp" ]; then
    echo -e "${RED}错误: 请在 example/c++ 目录下运行此脚本${NC}"
    echo -e "${YELLOW}正确路径: jakaAPI_K1_2.3.3_DUAL_x64/example/c++${NC}"
    exit 1
fi

# 检查SDK库文件
SDK_LIB="../../Linux/c&c++/x86_64-linux-gnu/shared/libjakaAPI_2_3_3.so"
if [ ! -f "$SDK_LIB" ]; then
    echo -e "${RED}错误: 找不到JAKA SDK库文件${NC}"
    echo -e "${YELLOW}期望路径: $SDK_LIB${NC}"
    exit 1
fi

echo -e "${GREEN}✓ SDK库文件检查通过${NC}"
echo ""

# 1. 创建build目录
echo -e "${YELLOW}[1/4] 创建build目录...${NC}"
mkdir -p build
cd build

# 2. 运行cmake
echo -e "\n${YELLOW}[2/4] 配置CMake...${NC}"
if ! cmake .. ; then
    echo -e "${RED}CMake配置失败！${NC}"
    exit 1
fi

# 3. 编译
echo -e "\n${YELLOW}[3/4] 编译程序...${NC}"
CORES=$(nproc)
echo -e "${BLUE}使用 $CORES 个CPU核心进行并行编译${NC}"
if ! make -j$CORES ; then
    echo -e "${RED}编译失败！${NC}"
    exit 1
fi

# 4. 显示编译结果
echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}✓ 编译成功！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}可执行文件：${NC}"
ls -lh *_test 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}'
echo ""

# 5. 检查网络连接
echo -e "${YELLOW}[4/4] 检查机器人连接...${NC}"
ROBOT_IP="192.168.2.200"
if ping -c 1 -W 1 $ROBOT_IP &> /dev/null; then
    echo -e "${GREEN}✓ 机器人控制器 $ROBOT_IP 在线${NC}"
else
    echo -e "${RED}✗ 无法连接到机器人控制器 $ROBOT_IP${NC}"
    echo -e "${YELLOW}  请检查网络连接和机器人电源${NC}"
fi
echo ""

# 6. 询问是否运行
echo -e "${GREEN}========================================${NC}"
echo -e "${YELLOW}请选择要运行的测试程序：${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "  1) 关节伺服测试 (edgservo_test)"
echo "     - 正弦波关节运动"
echo "     - 频率: 125Hz"
echo "     - 幅度: J1=±30°, J2=±20°, J4=±10°"
echo ""
echo "  2) 力控伺服测试 (edgservo_fct_test)"
echo "     - 笛卡尔空间力控运动"
echo "     - 需要力传感器"
echo ""
echo "  3) 多臂MoveJ测试 (multi_movj_test)"
echo "     - 双臂关节空间点到点运动"
echo ""
echo "  4) 多臂MoveL测试 (multi_movl_test)"
echo "     - 双臂笛卡尔空间直线运动"
echo ""
echo "  5) 不运行，退出"
echo ""
read -p "请选择 [1-5]: " choice

case $choice in
    1)
        PROGRAM="./edgservo_test"
        TEST_NAME="关节伺服测试"
        ;;
    2)
        PROGRAM="./edgservo_fct_test"
        TEST_NAME="力控伺服测试"
        ;;
    3)
        PROGRAM="./multi_movj_test"
        TEST_NAME="多臂MoveJ测试"
        ;;
    4)
        PROGRAM="./multi_movl_test"
        TEST_NAME="多臂MoveL测试"
        ;;
    5)
        echo ""
        echo -e "${GREEN}编译完成！${NC}"
        echo -e "${YELLOW}可手动运行测试：${NC}"
        echo "  cd build"
        echo "  sudo ./edgservo_test"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

# 显示安全提示
echo ""
echo -e "${RED}========================================${NC}"
echo -e "${RED}⚠️  安全提示${NC}"
echo -e "${RED}========================================${NC}"
echo -e "${YELLOW}1. 确保机器人周围无障碍物${NC}"
echo -e "${YELLOW}2. 确保急停按钮可用${NC}"
echo -e "${YELLOW}3. 保持安全距离（≥1米）${NC}"
echo -e "${YELLOW}4. 全程监控机器人运动${NC}"
echo -e "${RED}========================================${NC}"
echo ""
read -p "我已确认上述安全事项，按Enter继续，Ctrl+C取消..."

# 运行程序
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}运行: $TEST_NAME${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查sudo权限
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}需要sudo权限以获取实时调度优先级${NC}"
    echo -e "${BLUE}命令: sudo $PROGRAM${NC}"
    echo ""
    sudo $PROGRAM
else
    echo -e "${BLUE}命令: $PROGRAM${NC}"
    echo ""
    $PROGRAM
fi

EXIT_CODE=$?

# 显示结果
echo ""
echo -e "${GREEN}========================================${NC}"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓ 测试完成${NC}"
else
    echo -e "${RED}✗ 测试失败 (退出码: $EXIT_CODE)${NC}"
fi
echo -e "${GREEN}========================================${NC}"

exit $EXIT_CODE
