#!/bin/bash
# 启动 QYH Bag Recorder 节点
# 用于录制 rosbag 数据

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== 启动 QYH Bag Recorder ==="

# Source ROS2 环境
source /opt/ros/humble/setup.bash

# Source 工作空间
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "警告: 未找到工作空间 setup.bash"
    echo "请先编译: cd $WS_DIR && colcon build --packages-select qyh_bag_recorder"
fi

# 设置默认参数
BASE_PATH="${1:-$HOME/qyh_jushen_ws/DATA}"

echo "录制数据保存路径: $BASE_PATH"

# 确保目录存在
mkdir -p "$BASE_PATH"

# 启动bringup 并将日志输出到文件
export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

# 启动节点
ros2 launch qyh_bag_recorder bag_recorder.launch.py \
    base_path:="$BASE_PATH"
