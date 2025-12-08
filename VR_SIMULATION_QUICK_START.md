# VR 遥操作仿真快速启动指南

## 目标

用 PICO 4 VR 手柄控制 ROS2 中的仿真双机械臂，实时在 RViz 中看到机械臂跟随 VR 手柄移动。

**控制方式**: Clutch 离合器模式
- **按住 Grip 键**: 建立 VR 和机械臂的映射，机械臂跟随 VR 手柄移动
- **松开 Grip 键**: 机械臂保持当前位置不动，可以调整 VR 手柄位置

## 系统架构

```
PICO 4 VR 头显                    WSL Ubuntu (ROS2)
┌─────────────────────┐           ┌────────────────────────────────────┐
│  OpenXR 应用        │           │                                    │
│       ↓             │           │  ① vr_bridge_node (C++)            │
│  UDP 广播 (9999端口)│──────────→│     接收UDP，坐标变换，发布TF      │
│                     │           │       ↓                            │
└─────────────────────┘           │  vr_origin -> vr_head              │
                                  │            -> vr_left_hand         │
                                  │            -> vr_right_hand        │
                                  │       ↓                            │
                                  │  ② vr_clutch_node (Python)         │
                                  │     Clutch离合器控制                │
                                  │     grip>0.8: 跟踪VR增量           │
                                  │     grip<0.2: 保持位置             │
                                  │       ↓                            │
                                  │  /sim/left_target_pose             │
                                  │  /sim/right_target_pose            │
                                  │       ↓                            │
                                  │  ③ sim_arm_controller              │
                                  │     接收目标位姿，执行IK规划        │
                                  │       ↓                            │
                                  │  ④ MoveIt + RViz                   │
                                  │     显示双臂机械臂模型              │
                                  └────────────────────────────────────┘
```

## 坐标系说明

**坐标变换在 vr_bridge_node 中完成：**

| 坐标系 | X | Y | Z |
|--------|---|---|---|
| PICO VR | 右 | 上 | 后(-Z=前) |
| ROS (vr_origin) | 前 | 左 | 上 |

**变换公式：**
- `ros_x = -vr_z` (VR的-Z前 → ROS的X前)
- `ros_y = -vr_x` (VR的-X左 → ROS的Y左)
- `ros_z = vr_y` (VR的Y上 → ROS的Z上)

**握持补偿：** 默认 35° pitch，补偿手柄自然握持角度

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

## 第三步：运行仿真 (需要 4 个 WSL 终端)

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
ros2 run qyh_vr_bridge vr_bridge_node --ros-args -p grip_offset_deg:=35.0
```

**作用**: 
- 接收 PICO VR 的 UDP 数据
- 执行 VR→ROS 坐标变换
- 发布 TF (`vr_origin → vr_head/vr_left_hand/vr_right_hand`)
- 发布 `/vr/*/pose`, `/vr/*/joy`

**参数**:
- `grip_offset_deg`: 握持补偿角度(默认35°)，可根据实际调整

---

### WSL 终端 3：启动 VR Clutch (离合器控制)

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qyh_vr_calibration vr_clutch.launch.py
```

**作用**: 实现 Clutch 离合器逻辑
- 订阅 `/vr/left_hand/pose`, `/vr/left_hand/joy`
- 按住 Grip (>0.8) 时跟踪 VR 增量
- 松开 Grip (<0.2) 时保持位置
- 发布目标位姿到 `/sim/left_target_pose`, `/sim/right_target_pose`
- 发布离合器状态 `/vr/left_clutch_engaged`, `/vr/right_clutch_engaged`

---

### WSL 终端 4：启动仿真机械臂控制器

```bash
cd ~/qyh_jushen_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run qyh_vr_calibration sim_arm_controller
```

**作用**: 
- 订阅目标位姿 `/sim/left_target_pose`, `/sim/right_target_pose`
- 使用 MoveIt `/compute_ik` 服务求解关节角度
- 发布 `/joint_states` 驱动 RViz 中的机械臂模型
- 启动时自动移动到 ready 位置（避免 IK 失败）

---

> **重要启动顺序**: MoveIt → VR Bridge → VR Clutch → Sim Arm Controller

---

## 快速启动脚本

使用一键启动脚本:

```bash
cd ~/qyh_jushen_ws
./src/start_vr_simulation.sh
```

或者在 Windows WSL 中:
```powershell
wsl -d Ubuntu -e bash -c "cd ~/qyh_jushen_ws && ./src/start_vr_simulation.sh"
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
| TF 坐标轴方向不对 | 调整 `grip_offset_deg` 参数 |

## Clutch 参数调整

配置文件: `qyh_vr_calibration/config/vr_clutch_params.yaml`

```yaml
vr_clutch_node:
  ros__parameters:
    clutch:
      engage_threshold: 0.8    # grip > 0.8 时接合
      release_threshold: 0.2   # grip < 0.2 时释放
      position_scale: 2.0      # VR位移缩放 (机械臂/人臂长度比)
      rotation_scale: 1.0      # 旋转缩放
      max_position_delta: 0.02 # 单帧最大位移 (m)
      max_rotation_delta: 0.05 # 单帧最大旋转 (rad)
```

## VR Bridge 参数

启动时可调整的参数：

```bash
ros2 run qyh_vr_bridge vr_bridge_node --ros-args \
    -p udp_port:=9999 \
    -p grip_offset_deg:=35.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `udp_port` | 9999 | UDP 监听端口 |
| `grip_offset_deg` | 35.0 | 手柄握持补偿角度 (pitch方向) |

---

## 下一步：真机控制

当前仿真阶段验证完成后，真机控制需要：

1. **qyh_teleoperation_controller** - 差分IK + 轨迹平滑 + 安全检查
2. **qyh_jaka_control** - JAKA 机械臂驱动
3. 125Hz 高频控制循环

架构：
```
vr_bridge_node → vr_clutch_node → teleoperation_node → jaka_bridge_node → 真实机械臂
```

