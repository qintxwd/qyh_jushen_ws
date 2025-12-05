# VR 遥操作仿真快速启动指南

## 目标

用 PICO VR 手柄控制 ROS2 中的两个仿真机械臂，实时在 RViz 中看到机械臂跟随 VR 手柄移动。

**控制方式**: Clutch 离合器模式
- **按住 Grip 键**: 建立 VR 和机械臂的映射，机械臂跟随 VR 手柄移动
- **松开 Grip 键**: 机械臂保持当前位置不动，可以调整 VR 手柄位置

## 系统架构

```
Windows                              WSL Ubuntu (ROS2)
┌─────────────────────┐              ┌────────────────────────────────────┐
│  PICO 4 VR 头显     │              │                                    │
│       ↓             │              │  ① vr_bridge_node                  │
│  UDP 广播 (9999端口)│─────────────→│     接收UDP，发布ROS2话题          │
│                     │              │       ↓                            │
│  udp_forward_to_wsl │              │  /vr/left_hand/pose                │
│  (Windows UDP转发)  │              │  /vr/left_hand/joy (含grip值)      │
└─────────────────────┘              │       ↓                            │
                                     │  ② vr_clutch_node                  │
                                     │     Clutch离合器控制                │
                                     │     grip>0.8: 跟踪VR增量           │
                                     │     grip<0.2: 保持位置             │
                                     │       ↓                            │
                                     │  /sim/left_target_pose             │
                                     │  /sim/right_target_pose            │
                                     │       ↓                            │
                                     │  ③ sim_arm_controller              │
                                     │     MoveIt运动规划执行              │
                                     │       ↓                            │
                                     │  ④ MoveIt + RViz                   │
                                     │     显示双臂机械臂模型              │
                                     └────────────────────────────────────┘
```

## 第一步：安装依赖 (WSL Ubuntu)

```bash
# 一键安装所有依赖
sudo apt update && sudo apt install -y \
    ros-humble-moveit ros-humble-moveit-configs-utils \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher ros-humble-xacro \
    python3-colcon-common-extensions build-essential

# Python 依赖 (使用国内镜像加速)
pip3 install 'empy==3.3.4' numpy transforms3d scipy -i https://mirrors.aliyun.com/pypi/simple/
```

## 第二步：编译工作空间

```bash
# 进入工作空间 (建议复制到 WSL 本地以提高编译速度)
cd ~/qyh_jushen_ws
# 或使用 Windows 路径: cd /mnt/d/work/yc/qyh_jushen_ws/qyh_jushen_ws

source /opt/ros/humble/setup.bash
# 只编译相关包，节省时间
colcon build --symlink-install --packages-up-to qyh_vr_bridge qyh_vr_calibration qyh_dual_arms_moveit_config
source install/setup.bash
```

## 第三步：运行仿真 (需要 4 个 WSL 终端 + 1 个 Windows 终端)

### Windows 终端：UDP 转发 (如果 WSL 网络未配置镜像模式)

```powershell
cd D:\work\yc\qyh_jushen_ws\资料\pico4
python udp_forward_to_wsl.py
```

---

### WSL 终端 1：启动 MoveIt + RViz (机械臂可视化)

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qyh_dual_arms_moveit_config demo.launch.py
```

**作用**: 启动双臂机械臂模型 + RViz 可视化界面

---

### WSL 终端 2：启动 VR Bridge (接收 UDP 数据)

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qyh_vr_bridge vr_bridge_node
```

**作用**: 接收 PICO VR 的 UDP 数据，发布到 ROS2 话题

---

### WSL 终端 3：启动仿真机械臂控制器 (必须在 VR Clutch 之前!)

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qyh_vr_calibration sim_arm_controller
```

**作用**: 
- 接收目标位姿，使用 MoveIt 规划并执行机械臂运动
- **发布当前末端位姿** `/sim/left_current_pose`, `/sim/right_current_pose`

---

### WSL 终端 4：启动 VR Clutch (离合器控制)

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qyh_vr_calibration vr_clutch.launch.py
```

**作用**: 实现 Clutch 离合器逻辑
- 订阅 `/vr/left_hand/pose`, `/vr/left_hand/joy`
- **订阅当前末端位姿** `/sim/*_current_pose` (确保按下 Grip 时从真实位置开始)
- 按住 Grip (>0.8) 时跟踪 VR 增量
- 松开 Grip (<0.2) 时保持位置
- 发布目标位姿到 `/sim/left_target_pose`, `/sim/right_target_pose`

---

> **重要**: 启动顺序**作用**: 接收目标位姿，使用 MoveIt 规划并执行机械臂运动

---

## 快速启动脚本 (可选)

创建一键启动脚本 `start_vr_sim.sh`:

```bash
#!/bin/bash
# 保存到 ~/qyh_jushen_ws/start_vr_sim.sh

cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 在不同终端标签页中启动各节点
gnome-terminal --tab --title="MoveIt" -- bash -c "ros2 launch qyh_dual_arms_moveit_config demo.launch.py; exec bash"
sleep 3
gnome-terminal --tab --title="VR Bridge" -- bash -c "ros2 run qyh_vr_bridge vr_bridge_node; exec bash"
gnome-terminal --tab --title="VR Clutch" -- bash -c "ros2 launch qyh_vr_calibration vr_clutch.launch.py; exec bash"
gnome-terminal --tab --title="Sim Arm" -- bash -c "ros2 run qyh_vr_calibration sim_arm_controller; exec bash"

echo "所有节点已启动!"
```

## 第四步：WSL 网络配置 (重要!)

WSL2 默认无法直接接收外部 UDP 广播，有两种解决方案：

### 方案 A：WSL 镜像网络模式 (推荐，Windows 11)

1. 在 Windows 中创建文件 `C:\Users\你的用户名\.wslconfig`，内容：
```ini
[wsl2]
networkingMode=mirrored
```

2. 重启 WSL：
```powershell
wsl --shutdown
wsl
```

这样 WSL 和 Windows 共享网络，可以直接收到 UDP，无需运行转发脚本。

### 方案 B：Windows 上运行 UDP 转发 (通用)

由于 `netsh portproxy` 只支持 TCP，UDP 需要用脚本转发：

**Windows PowerShell 终端运行**：
```powershell
cd D:\work\yc\qyh_jushen_ws\资料\pico4
python udp_forward_to_wsl.py
```

脚本会自动检测 WSL IP 并转发 UDP 数据。

## 在哪里看到机械臂？

运行终端 1 后，会自动弹出 **RViz** 窗口，里面显示：
- 两个 JAKA ZU7 机械臂模型（左臂 + 右臂）
- 坐标轴和 TF 变换

### 操作方法

1. **戴上 PICO VR 头显**
2. **按住左手/右手的 Grip 键** (侧面握键)
3. **移动手柄** → 对应的机械臂会跟随移动
4. **松开 Grip 键** → 机械臂停止，保持当前位置
5. **重新按住 Grip 键** → 从当前位置继续跟随

## 验证数据流

```bash
# 查看 VR 原始数据
ros2 topic echo /vr/left_hand/pose --once
ros2 topic echo /vr/left_hand/joy --once   # axes[3] 是 grip 值

# 查看 Clutch 状态
ros2 topic echo /vr/left_clutch_engaged --once

# 查看目标位姿 (Clutch 输出)
ros2 topic echo /sim/left_target_pose --once

# 查看话题发布频率
ros2 topic hz /vr/left_hand/pose
ros2 topic hz /sim/left_target_pose
```

## 常见问题

| 问题 | 解决方案 |
|------|----------|
| 编译报错 `No module named 'em'` | `pip3 install 'empy==3.3.4'` |
| RViz 窗口不显示 | Windows 11 自带 WSLg，Windows 10 需安装 VcXsrv |
| VR 数据收不到 | 检查 WSL 网络配置或运行 UDP 转发脚本 |
| MoveIt 启动失败 | 检查是否 source 了 install/setup.bash |
| 按住 Grip 但机械臂不动 | 检查 `/vr/left_clutch_engaged` 是否为 true |
| 机械臂动作很慢 | 正常现象，MoveIt 规划需要时间，可调整参数 |

## Clutch 参数调整

配置文件: `qyh_vr_calibration/config/vr_clutch_params.yaml`

```yaml
vr_clutch_node:
  ros__parameters:
    clutch:
      engage_threshold: 0.8    # grip > 0.8 时接合
      release_threshold: 0.2   # grip < 0.2 时释放
      position_scale: 1.0      # VR位移缩放
      max_position_delta: 0.05 # 单步最大位移 (m)
```

---

## 附录：qyh_teleoperation_controller 包说明

### 为什么在 VR 仿真中没有用到？

在当前的 VR 仿真架构中：

```
vr_bridge_node → vr_clutch_node → sim_arm_controller (直接控制仿真机械臂)
```

- **`sim_arm_controller`** 直接使用 MoveIt 的 `compute_ik` 服务进行完整IK求解
- 仿真不需要"差分IK+轨迹平滑"，因为仿真机械臂可以瞬间移动

而 **`qyh_teleoperation_controller`** 是为 **真实机器人** 设计的：

```
vr_clutch_node → teleoperation_node → jaka_bridge_node → 真实JAKA机械臂
```

- 真实机器人需要平滑的轨迹（不能瞬移）
- 需要安全检查（防止碰撞、超速）
- 需要125Hz的高频控制循环

### qyh_teleoperation_controller 的主要功能

```
VR目标姿态 → 差分IK → 轨迹平滑 → 安全检查 → 机械臂关节命令
     ↓           ↓           ↓            ↓
 PoseStamped  雅可比求解   限速/限加速   关节限位检查
```

1. **差分IK求解** (`DifferentialIKController`)
   - 将VR末端姿态转换为机械臂关节速度
   - 使用 MoveIt 的机器人模型和雅可比矩阵
   - 阻尼最小二乘法防止奇异点

2. **轨迹平滑** (`TrajectorySmoother`)
   - 速度限幅: 1.0 rad/s
   - 加速度限幅: 0.5 rad/s²
   - Jerk限幅: 5.0 rad/s³
   - 低通滤波消除抖动

3. **安全检查** (`SafetyChecker`)
   - 关节限位检查
   - 速度超限检查
   - 碰撞检测
   - 奇异点检测

4. **虚拟臂可视化** (`VirtualArmFollower`)
   - 发布虚拟机械臂状态用于RViz显示

### 话题订阅/发布

| 方向 | 话题 | 类型 |
|------|------|------|
| 订阅 | `/vr/left_target_pose` | PoseStamped |
| 订阅 | `/vr/right_target_pose` | PoseStamped |
| 订阅 | `/vr/left_clutch_engaged` | Bool |
| 订阅 | `/joint_states` | JointState |
| 发布 | `/left_arm/joint_command` | JointState |
| 发布 | `/right_arm/joint_command` | JointState |

### 什么时候会用到？

当你要控制 **真实JAKA机械臂** 时，完整的数据流是：

```
VR手柄 → vr_bridge_node → vr_clutch_node 
                              ↓
                     teleoperation_node (差分IK+平滑)
                              ↓
                     jaka_bridge_node → 真实机械臂
```

