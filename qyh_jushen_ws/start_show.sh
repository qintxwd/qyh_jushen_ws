#!/bin/bash

# 启动 start_show.sh

# 定义清理函数
cleanup() {
    echo ""
    echo "🛑 接收到中断信号 (Ctrl+C)，正在停止所有服务..."
    
    # 杀死所有记录的子进程
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "正在停止 PID: $pid"
            kill -SIGTERM "$pid"
        fi
    done
    
    echo "⏳ 等待所有进程退出..."
    wait
    echo "✅ 所有服务已安全停止"
    exit 0
}

# 注册信号捕获 (SIGINT: Ctrl+C, SIGTERM: kill)
trap cleanup SIGINT SIGTERM

# 存储 PID 的数组
PIDS=()

# ==========================================
# 1. 代码更新与编译阶段
# ==========================================

# 函数：安全切换 Tag
checkout_tag() {
    local dir=$1
    local tag=$2
    
    echo "📂 进入目录: $dir"
    if [ ! -d "$dir" ]; then
        echo "❌ 目录不存在: $dir"
        exit 1
    fi
    cd "$dir" || exit 1
    
    # 检查是否有未提交的更改
    if [ -n "$(git status --porcelain)" ]; then
        echo "❌ [错误] 目录 $dir 中有未提交的代码！"
        echo "   请先提交或暂存(stash)您的更改，然后重试。"
        git status --short
        exit 1
    fi
    
    echo "🔄 正在切换到 tag: $tag ..."
    # 尝试获取最新 tags
    git fetch --tags >/dev/null 2>&1
    
    if git checkout "$tag"; then
        echo "✅ 成功切换到 $tag"
    else
        echo "❌ 切换到 tag $tag 失败！请检查 tag 是否存在。"
        exit 1
    fi
    echo "----------------------------------------"
}

# 1.1 更新 Web 前端代码
checkout_tag "$HOME/qyh_jushen_ws/qyh_jushen_web" "show"

# 1.2 更新主工作空间代码
checkout_tag "$HOME/qyh_jushen_ws" "show"

# 1.3 编译 ROS 工作空间
BUILD_DIR="$HOME/qyh_jushen_ws/qyh_jushen_ws"
echo "🔨 进入编译目录: $BUILD_DIR"
cd "$BUILD_DIR" || { echo "❌ 无法进入编译目录"; exit 1; }

echo "⏳ 开始编译 (colcon build)..."
if colcon build; then
    echo "✅ 编译成功！"
else
    echo "❌ 编译失败！请检查代码错误。"
    exit 1
fi
echo "----------------------------------------"

# ==========================================
# 2. 启动服务阶段
# ==========================================

# 读取 ROS_DOMAIN_ID
ROS_DOMAIN_ID_FILE="$HOME/qyh_jushen_ws/persistent/ros/ROS_DOMAIN_ID"
if [ -f "$ROS_DOMAIN_ID_FILE" ]; then
    export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
else
    export ROS_DOMAIN_ID=0
fi
echo "🔧 ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
# Source 工作空间 (编译后重新 source)
source "$HOME/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash"

# 启动bringup 并将日志输出到文件
export RCUTILS_LOGGING_FORMAT='[{time:%Y-%m-%d %H:%M:%S.%e}] [Version:'"$GLOBAL_SLAM_VERSION"'] [{severity}] [{name}] [{file_name}:{line_number}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

# jaka臂控制节点
echo "🚀 启动JAKA臂控制节点..."
ros2 launch qyh_jaka_control jaka_control.launch.py &
PIDS+=($!)

# 底盘控制节点
echo "🚀 启动底盘控制节点..."
ros2 launch qyh_standard_robot standard_robot.launch.py &
PIDS+=($!)

# 任务引擎节点
echo "🚀 启动任务引擎节点..."
ros2 launch qyh_task_engine task_engine.launch.py &
PIDS+=($!)

# 升降台控制节点
echo "🚀 启动升降台控制节点..."
ros2 launch qyh_lift_control lift_control.launch.py &
PIDS+=($!)

# 夹爪控制节点
echo "🚀 启动夹爪控制节点..."
# 因为目前只有左夹爪，所以只启动左夹爪节点
ros2 launch qyh_gripper_control gripper_control_left.launch.py &
PIDS+=($!)

# 腰部控制节点
echo "🚀 启动腰部控制节点..."
ros2 launch qyh_waist_control waist_control.launch.py &
PIDS+=($!)

# 除此之外，我们还需要启动前后端
echo "🚀 启动后端节点..."
pushd "$HOME/qyh_jushen_ws/qyh_jushen_web/backend" > /dev/null
./start.sh &
PIDS+=($!)
popd > /dev/null

echo "🚀 启动前端节点..."
pushd "$HOME/qyh_jushen_ws/qyh_jushen_web/frontend" > /dev/null
./start.sh &
PIDS+=($!)
popd > /dev/null

echo "✅ 所有节点已启动 (PIDS: ${PIDS[*]})"
echo "按 Ctrl+C 停止所有服务并退出"

# 挂起脚本，等待信号
wait
