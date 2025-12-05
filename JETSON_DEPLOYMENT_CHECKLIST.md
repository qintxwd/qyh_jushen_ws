# VR遥操作系统 - Jetson部署检查清单

## 当前开发状态

### ✅ 已完成的模块

1. **MoveIt2配置** (`qyh_dual_arms_moveit_config`)
   - SRDF配置（左右臂planning groups）
   - KDL运动学求解器
   - 关节限位配置
   - 碰撞矩阵
   - OMPL运动规划器

2. **遥操作控制器** (`qyh_teleoperation_controller`)
   - 差分IK控制器（Damped Least Squares）
   - 轨迹平滑器（速度/加速度/jerk限制 + 低通滤波）
   - 安全检查器（关节限位/碰撞/奇异检测）
   - 虚拟臂跟随器（RViz可视化）
   - 发布: `/left_arm/joint_command`, `/right_arm/joint_command` (JointState)

3. **VR接口** (`qyh_vr_calibration`)
   - **VR Clutch 节点** (`vr_clutch_node`) - 离合器控制
     - 订阅: `/vr/*/pose`, `/vr/*/joy`
     - Grip > 0.8: 接合离合器，跟踪 VR 增量
     - Grip < 0.2: 释放离合器，保持位置
     - 发布: `/sim/left_target_pose`, `/sim/right_target_pose`
     - 发布: `/vr/left_clutch_engaged`, `/vr/right_clutch_engaged`
   - VR姿态接收和处理
   - TF2坐标变换
   - 移动平均滤波（位置5窗口，姿态3窗口）
   - 死区过滤（2mm位置，0.57°姿态）
   - VR模拟器（测试用）
   - 仿真机械臂控制器 (`sim_arm_controller`)

4. **JAKA桥接** (`qyh_jaka_control`)
   - 平滑伺服桥接类（`SmoothServoBridge`）
     - 轨迹缓冲器（默认10个点）
     - 线性插值平滑
     - 性能统计（频率/延迟/错误）
   - JAKA桥接节点（`jaka_bridge_node`）
     - 订阅: `/left_arm/joint_command`, `/right_arm/joint_command` (JointState)
     - 125Hz定时器
     - EtherCAT同步伺服（edgServoJ + edgSend）
     - 发布: `/joint_states`

5. **系统集成launch**
   - `full_system.launch.py` - 完整系统
   - `test_vr_interface.launch.py` - VR接口测试
   - `jaka_bridge.launch.py` - JAKA桥接

## ⚠️ 当前限制

### 编译环境
- **当前**: Windows + WSL x86_64
- **JAKA SDK**: ARM64版本（libjakaAPI_2_3_0_13.so）
- **结果**: qyh_jaka_control无法在WSL x86_64上编译成功

### 架构不匹配错误
```
/usr/bin/ld: libjakaAPI_2_3_0_13.so: error adding symbols: file in wrong format
```

## 🎯 Jetson部署步骤

### 1. 环境准备

**检查Jetson环境**:
```bash
# 确认架构
uname -m  # 应该是 aarch64

# 确认ROS2 Humble已安装
ros2 --version

# 确认必要的依赖
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-control-toolbox \
    python3-numpy \
    python3-scipy
```

### 2. 代码同步

```bash
# 在Jetson上克隆或同步代码
cd ~
git clone https://github.com/qintxwd/qyh_jushen_ws.git
cd qyh_jushen_ws/qyh_jushen_ws

# 或者从开发机rsync
# 在开发机上:
rsync -avz --exclude='build' --exclude='install' --exclude='log' \
    ~/qyh_jushen_ws/qyh_jushen_ws/ \
    jetson@<jetson_ip>:~/qyh_jushen_ws/qyh_jushen_ws/
```

### 3. 编译完整系统

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws

# 清理旧的build
rm -rf build install log

# 编译所有包
source /opt/ros/humble/setup.bash
colcon build

# 如果出错，分别编译
colcon build --packages-select qyh_dual_arms_description
colcon build --packages-select qyh_dual_arms_moveit_config
colcon build --packages-select qyh_teleoperation_msgs
colcon build --packages-select qyh_teleoperation_controller
colcon build --packages-select qyh_vr_calibration_msgs
colcon build --packages-select qyh_vr_calibration
colcon build --packages-select qyh_jaka_control_msgs
colcon build --packages-select qyh_jaka_control  # ← 重点：这个包在WSL上编译失败

source install/setup.bash
```

### 4. 配置网络

**机器人网络**:
```bash
# 确保可以ping通机器人
ping 192.168.2.200

# 如果需要修改IP，编辑配置文件
nano src/qyh_jaka_control/config/jaka_bridge_params.yaml
# 修改 robot_ip 参数
```

### 5. 测试各模块

**阶段1: 测试MoveIt**
```bash
# 终端1: 启动MoveIt
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py

# 终端2: 检查话题
ros2 topic list | grep move_group
ros2 service list | grep compute_ik
```

**阶段2: 测试VR模拟器+接口+Clutch**
```bash
# 启动VR模拟器、接口和Clutch节点
ros2 launch qyh_vr_calibration test_vr_interface.launch.py \
    motion_type:=circle \
    amplitude:=0.05

# 检查VR输入
ros2 topic hz /vr/left_target_pose  # 应该~90Hz
ros2 topic echo /vr/left_target_pose --once

# 检查Clutch状态
ros2 topic echo /vr/left_clutch_engaged  # Bool: data=True/False
ros2 topic echo /vr/right_clutch_engaged

# 检查仿真目标姿态
ros2 topic hz /sim/left_target_pose      # Clutch接合时输出
ros2 topic echo /sim/left_target_pose --once
```

**阶段3: 测试遥操作控制器**
```bash
# 终端1: MoveIt
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py

# 终端2: VR模拟器
ros2 launch qyh_vr_calibration test_vr_interface.launch.py

# 终端3: 遥操作控制器
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py

# 终端4: 检查关节命令
ros2 topic hz /left_arm/joint_command  # 应该~125Hz
ros2 topic echo /left_arm/joint_command --once
```

**阶段4: 测试JAKA桥接（不连真实机器人）**
```bash
# 修改配置为不自动连接
nano src/qyh_jaka_control/config/jaka_bridge_params.yaml
# 设置 auto_connect: false

# 启动桥接节点
ros2 launch qyh_jaka_control jaka_bridge.launch.py

# 应该能看到节点启动，只是不会连接机器人
```

**阶段5: 完整系统测试（连真实机器人）**
```bash
# ⚠️ 确保机器人处于安全状态，工作空间清空，急停在手边

# 恢复自动连接配置
nano src/qyh_jaka_control/config/jaka_bridge_params.yaml
# 设置 auto_connect: true

# 启动完整系统
ros2 launch qyh_teleoperation_bringup full_system.launch.py \
    robot_ip:=192.168.2.200 \
    use_simulator:=true  # 先用VR模拟器

# 启动JAKA伺服模式
ros2 service call /jaka/bridge/start_servo std_srvs/srv/Trigger

# 观察机器人运动是否平滑
# 如果正常，按Ctrl+C停止，再用真实VR测试
```

### 6. 性能监控

```bash
# 查看话题频率
ros2 topic hz /vr/left_target_pose        # VR输入 ~90Hz
ros2 topic hz /left_arm/joint_command      # 遥操作输出 ~125Hz
ros2 topic hz /joint_states                # 机器人状态 ~125Hz

# 查看节点CPU占用
top -p $(pgrep -f teleoperation_node)
top -p $(pgrep -f jaka_bridge_node)

# 停止伺服查看统计
ros2 service call /jaka/bridge/stop_servo std_srvs/srv/Trigger
# 会输出：频率、延迟、错误数、缓冲区溢出数
```

## 🔧 可能需要调整的参数

### 如果机器人报"位置跳变过大"

**降低速度限制** (`teleoperation_params.yaml`):
```yaml
max_joint_velocity: 0.5      # 从1.0降到0.5
max_joint_acceleration: 0.3  # 从0.5降到0.3
max_joint_jerk: 3.0          # 从5.0降到3.0
```

**增加平滑** (`jaka_bridge_params.yaml`):
```yaml
buffer_size: 15              # 从10增加到15
interpolation_weight: 0.3    # 从0.5降到0.3
```

### 如果延迟太大

**减少滤波** (`vr_interface_params.yaml`):
```yaml
position_smoothing_window: 3  # 从5降到3
orientation_smoothing_window: 2  # 从3降到2
```

**减少缓冲** (`jaka_bridge_params.yaml`):
```yaml
buffer_size: 5               # 从10降到5
interpolation_weight: 0.7    # 从0.5增到0.7
```

## 📊 预期性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| VR输入频率 | 90Hz | Pico4典型刷新率 |
| 遥操作控制器输出 | 125Hz | 匹配JAKA伺服频率 |
| JAKA伺服发送 | 125Hz | EtherCAT周期 |
| 端到端延迟 | <100ms | VR到机器人响应 |
| 位置平滑度 | 无报错 | 消除"位置跳变过大"错误 |
| CPU占用 | <50% | Jetson单核 |

## 🐛 故障排查

### 问题1: 编译失败 - JAKA SDK not found
**检查**:
```bash
ls -la src/qyh_jaka_control/thirdparty/lib/
# 应该看到 libjakaAPI_2_3_0_13.so
file src/qyh_jaka_control/thirdparty/lib/libjakaAPI_2_3_0_13.so
# 应该显示 ELF 64-bit LSB shared object, ARM aarch64
```

### 问题2: 运行时 - 无法连接机器人
**检查**:
```bash
ping 192.168.2.200
telnet 192.168.2.200 10000  # JAKA默认端口
# 查看日志
ros2 topic echo /rosout
```

### 问题3: 伺服启动失败
**可能原因**:
- 机器人未上电/使能
- 已有其他程序占用伺服
- 机器人处于错误状态

**解决**:
```bash
# 通过JAKA示教器检查机器人状态
# 或使用原有的jaka_control_node服务
ros2 service call /jaka/robot/power_on std_srvs/srv/Trigger
ros2 service call /jaka/robot/enable std_srvs/srv/Trigger
ros2 service call /jaka/robot/clear_error std_srvs/srv/Trigger
```

### 问题4: 缓冲区频繁溢出
**现象**: stop_servo时显示大量buffer_overflow_count

**原因**: 输入频率 > 输出频率

**解决**:
```yaml
# jaka_bridge_params.yaml
buffer_size: 20  # 增大缓冲
```

## 📝 开发备注

### 话题流向
```
VR手柄 (90Hz)
  ↓
/vr/left_hand/pose, /vr/right_hand/pose
/vr/left_hand/joy, /vr/right_hand/joy (含grip值)
  ↓ [vr_interface_node: 坐标变换+滤波]
  ↓
/vr/left_target_pose, /vr/right_target_pose
  ↓ [vr_clutch_node: 离合器控制]
  ↓ grip > 0.8: 接合, grip < 0.2: 释放
  ↓
/sim/left_target_pose, /sim/right_target_pose (离合接合时)
/vr/left_clutch_engaged, /vr/right_clutch_engaged (Bool状态)
  ↓ [teleoperation_node: 差分IK+平滑+安全]
  ↓
/left_arm/joint_command, /right_arm/joint_command (JointState, 125Hz)
  ↓ [jaka_bridge_node: 缓冲+插值]
  ↓
JAKA SDK edgServoJ (125Hz)
  ↓
真实机器人
```

### 关键文件位置
- 遥操作参数: `src/qyh_teleoperation_controller/config/teleoperation_params.yaml`
- VR接口参数: `src/qyh_vr_calibration/config/vr_interface_params.yaml`
- JAKA桥接参数: `src/qyh_jaka_control/config/jaka_bridge_params.yaml`
- 完整系统launch: `src/qyh_teleoperation_bringup/launch/full_system.launch.py`

### 下一步优化方向
1. 添加VR按键功能（紧急停止、速度调节）
2. 添加力反馈支持
3. 优化插值算法（三次样条 vs 线性）
4. 添加工作空间限制
5. 记录和回放轨迹
6. 实时性能分析工具
7. 夹爪控制（Trigger按键）

## 联系信息

如有问题，检查:
1. 系统日志: `ros2 topic echo /rosout`
2. 节点信息: `ros2 node info /jaka_bridge_node`
3. 参数列表: `ros2 param list /jaka_bridge_node`
