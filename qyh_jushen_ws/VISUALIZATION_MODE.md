# 可视化测试模式使用说明

## 功能说明

添加了 `visualization_only` 模式，允许在不连接真实机器人的情况下，在 RViz 中预览伺服运动轨迹。

## 主要修改

1. **jaka_control_node.cpp**:
   - 添加 `visualization_only_` 参数
   - 在 `mainLoop()` 中添加条件判断：
     - 可视化模式：将伺服指令发布到 `/joint_states`
     - 真实模式：调用 `edgServoJ()` 发送给机器人

2. **launch文件**:
   - 添加 `visualization_only` 参数（默认false）

3. **启动脚本**:
   - `start_visualization_test.sh` (Linux/WSL)
   - `start_visualization_test.ps1` (Windows PowerShell)

## 使用方法

### 方式一：直接启动（推荐）

**Linux/WSL:**
```bash
cd qyh_jushen_ws
chmod +x start_visualization_test.sh
./start_visualization_test.sh
```

**Windows PowerShell:**
```powershell
cd qyh_jushen_ws
.\start_visualization_test.ps1
```

### 方式二：手动启动

```bash
# 终端1: 启动控制节点（可视化模式）
source install/setup.bash
ros2 launch qyh_jaka_control jaka_control.launch.py \
    visualization_only:=true \
    cycle_time_ms:=8.0

# 终端2: 启动 RViz
rviz2

# 终端3: 发送测试指令
ros2 topic pub /jaka/left_bridge/joint_command sensor_msgs/msg/JointState \
    "{position: [0.5, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0]}" --once
```

## RViz 配置

1. 添加 **RobotModel** 显示
   - Fixed Frame: `world` 或 `base_link`
   - Robot Description: `/robot_description`

2. 订阅 **JointState**:
   - Topic: `/joint_states`

3. 观察机械臂运动是否平滑

## 测试场景

### 1. 测试速度限制和插值
```bash
# 发送较大增量，观察是否自动插值
ros2 topic pub /jaka/left_bridge/joint_command sensor_msgs/msg/JointState \
    "{position: [1.0, -0.5, 0.5, -1.0, 1.0, -0.5, 0.5]}" --once
```

### 2. 测试连续运动
```bash
# 循环发送指令
while true; do
  ros2 topic pub /jaka/left_bridge/joint_command sensor_msgs/msg/JointState \
      "{position: [$(python3 -c 'import random; print(",".join([str(random.uniform(-1,1)) for _ in range(7)]))')}" --once
  sleep 0.05
done
```

### 3. 测试抖动修复
- 使用手柄发送指令，观察 RViz 中运动是否平滑
- 应该看到平滑的轨迹，无抖动

## 切换回真实机器人模式

```bash
ros2 launch qyh_jaka_control jaka_control.launch.py \
    visualization_only:=false \
    robot_ip:=192.168.2.200
```

或使用原有启动脚本（默认为真实模式）:
```bash
./start_jaka.sh
```

## 注意事项

1. **可视化模式下不会连接机器人**，因此：
   - 不需要机器人IP
   - 不会上电/使能
   - 状态查询功能无效

2. **joint_states 发布频率** = 控制循环频率（125Hz）

3. **适配器节点**：可视化模式下可以禁用 `qyh_jaka_joint_adapter`，因为直接发布标准格式

4. **调试日志**：如需详细日志，设置：
   ```bash
   ros2 run qyh_jaka_control jaka_control_node --ros-args --log-level debug
   ```

## 预期效果

✅ RViz 中看到机械臂平滑运动  
✅ 无抖动、无跳变  
✅ 速度限制生效（大增量自动插值）  
✅ 日志显示 "VISUALIZATION ONLY MODE"
