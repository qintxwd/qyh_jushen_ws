#!/bin/bash
# 数据处理管线测试脚本

set -e

echo "============================================================"
echo "数据处理管线测试"
echo "============================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查演示数据
echo -e "${YELLOW}1. 检查演示数据...${NC}"
DEMO_DIR="$HOME/demo_recordings"
if [ -d "$DEMO_DIR" ]; then
    NUM_DEMOS=$(find "$DEMO_DIR" -maxdepth 1 -type d -name "demo_*" | wc -l)
    echo -e "${GREEN}✓ 找到 $NUM_DEMOS 个演示数据${NC}"
    
    if [ $NUM_DEMOS -eq 0 ]; then
        echo -e "${RED}❌ 没有演示数据！请先录制一些演示。${NC}"
        echo ""
        echo "使用以下命令录制演示："
        echo "  ros2 launch sim_training_env record_demo.launch.py"
        echo "  python3 src/sim_training_env/scripts/teleop_keyboard.py"
        exit 1
    fi
else
    echo -e "${RED}❌ 演示数据目录不存在: $DEMO_DIR${NC}"
    exit 1
fi

echo ""

# 处理演示数据
echo -e "${YELLOW}2. 处理演示数据...${NC}"
python3 src/sim_training_env/scripts/data_processor.py \
    --demos_dir "$DEMO_DIR" \
    --output_dir "$HOME/processed_data" \
    --output_file training_data.npz

echo ""

# 检查输出文件
OUTPUT_FILE="$HOME/processed_data/training_data.npz"
if [ -f "$OUTPUT_FILE" ]; then
    FILE_SIZE=$(du -h "$OUTPUT_FILE" | cut -f1)
    echo -e "${GREEN}✓ 数据处理完成${NC}"
    echo -e "  输出文件: $OUTPUT_FILE"
    echo -e "  文件大小: $FILE_SIZE"
else
    echo -e "${RED}❌ 处理失败，未生成输出文件${NC}"
    exit 1
fi

echo ""

# 可视化数据
echo -e "${YELLOW}3. 验证数据质量...${NC}"
python3 src/sim_training_env/scripts/visualize_data.py \
    "$OUTPUT_FILE" \
    --stats --check

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}✓ 数据处理管线测试完成！${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "后续步骤："
echo "  1. 可视化图像: python3 src/sim_training_env/scripts/visualize_data.py $OUTPUT_FILE --images --episode 0"
echo "  2. 绘制关节轨迹: python3 src/sim_training_env/scripts/visualize_data.py $OUTPUT_FILE --joints --episode 0"
echo "  3. 生成视频: python3 src/sim_training_env/scripts/visualize_data.py $OUTPUT_FILE --video output.mp4 --episode 0"
echo ""
