# QYH Jushen ROS2 Workspace

具身机器人ROS2工作空间。包含机器人所有核心控制、硬件驱动、任务引擎等功能包。

## 环境要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **编译工具**: colcon

## 快速开始

### 1. 安装ROS2 Humble

```bash
# 安装ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# 安装colcon编译工具
sudo apt install python3-colcon-common-extensions
```

### 2. 安装依赖

```bash
# 安装ROS2依赖
cd qyh_jushen_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译工作空间

```bash
cd qyh_jushen_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

编译选项说明：
- `--symlink-install`: 使用符号链接而不是复制文件，方便开发调试
- `--packages-select <package_name>`: 只编译指定的包
- `--cmake-args -DCMAKE_BUILD_TYPE=Release`: 发布版本优化编译

### 4. 设置环境变量

```bash
source install/setup.bash

# 建议添加到 ~/.bashrc 中
echo "source ~/qyh-robot-system/qyh_jushen_ws/install/setup.bash" >> ~/.bashrc
```

### 5. 启动机器人

```bash
# 启动完整机器人系统
ros2 launch bringup bringup.launch.py

# 或启动仿真环境
ros2 launch bringup bringup_sim.launch.py
```

## 功能包详细说明

### 核心控制

#### qyh_standard_robot & qyh_standard_robot_msgs
- **功能**: 机器人底盘控制和导航接口
- **通信协议**: Modbus TCP
- **主要服务**:
  - 导航控制 (导航到坐标/站点)
  - 地图管理 (切换地图、强制定位)
  - 运动控制 (手动控制、急停、暂停/恢复)
  - 充电管理
- **启动**: `ros2 launch qyh_standard_robot standard_robot.launch.py`

#### qyh_task_engine & qyh_task_engine_msgs
- **功能**: 基于行为树的任务编排引擎
- **支持的技能**:
  - 机械臂运动 (MoveJ, 伺服控制)
  - 夹爪控制
  - 升降控制
  - 头部控制
  - 腰部旋转
  - 逻辑控制 (循环、条件、等待)
- **任务定义**: JSON格式任务描述
- **启动**: `ros2 launch qyh_task_engine task_engine.launch.py`

### 硬件控制

#### qyh_jaka_control & qyh_jaka_control_msgs
- **功能**: 节卡JAKA双臂机械臂控制
- **控制模式**:
  - 关节空间运动 (MoveJ)
  - 笛卡尔空间运动 (MoveL)
  - 伺服模式 (高频关节/笛卡尔速度控制)
  - VR遥操作跟随
- **安全特性**:
  - 碰撞检测与等级设置
  - 速度平滑与限制
  - 力矩传感器零点校准
- **启动**: `ros2 launch qyh_jaka_control jaka_control.launch.py`

#### qyh_gripper_control & qyh_gripper_msgs
- **功能**: 电动夹爪控制
- **通信协议**: Modbus RTU
- **控制参数**: 位置、速度、力度
- **启动**: `ros2 launch qyh_gripper_control gripper.launch.py`

#### qyh_lift_control & qyh_lift_msgs
- **功能**: 升降机构控制
- **通信协议**: Modbus TCP
- **控制模式**: 绝对位置控制、相对运动、归零
- **启动**: `ros2 launch qyh_lift_control lift_control.launch.py`

#### qyh_head_motor_control
- **功能**: 头部俯仰电机控制
- **通信协议**: 串口总线舵机协议
- **控制参数**: 角度位置、速度
- **启动**: `ros2 launch qyh_head_motor_control head_motor.launch.py`

#### qyh_waist_control & qyh_waist_msgs
- **功能**: 腰部旋转控制
- **通信协议**: Modbus TCP
- **控制模式**: 角度控制
- **启动**: `ros2 launch qyh_waist_control waist_control.launch.py`

### 遥操作

#### qyh_dual_arm_teleop_python
- **功能**: Python实现的双臂遥操作
- **输入源**: VR手柄姿态
- **输出**: 机械臂笛卡尔伺服指令

#### qyh_vr_bridge
- **功能**: VR设备与ROS2桥接
- **发布话题**: VR手柄位姿、按钮状态
- **坐标系转换**: VR空间到机器人基坐标系

#### qyh_vr_button_event
- **功能**: VR按钮事件处理
- **支持操作**: 夹爪控制、模式切换等

### 工具

#### qyh_bag_recorder
- **功能**: ROS2 bag录制管理
- **服务接口**: 开始/停止录制、查询状态
- **启动**: `ros2 launch qyh_bag_recorder bag_recorder.launch.py`

#### qyh_shutdown
- **功能**: 安全关机管理
- **功能**: 通过Modbus控制机器人关机

### 传感器

#### OrbbecSDK_ROS2
- **功能**: Orbbec深度相机驱动
- **发布数据**: RGB图像、深度图、点云
- **启动**: `ros2 launch orbbec_camera qyh.launch.py`

### 配置

#### qyh_dual_arms_description
- **功能**: 双臂机器人URDF描述
- **包含**: 机器人模型、关节定义、可视化配置
- **工具**: VR映射标定工具、机器人姿态标定GUI

#### bringup
- **功能**: 系统启动配置
- **启动文件**:
  - `bringup.launch.py`: 完整系统启动
  - `bringup_sim.launch.py`: 仿真环境启动
- **启动脚本**: shell脚本（经过shc加密）

## 配置文件说明

各功能包的配置文件位于 `<package>/config/` 目录下：

- **jaka_bridge_params.yaml**: 机械臂控制参数
- **gripper.yaml**: 夹爪通信和控制参数
- **lift_params.yaml**: 升降机构参数
- **waist_params.yaml**: 腰部控制参数
- **head_motor_params.yaml**: 头部电机参数
- **task_engine.yaml**: 任务引擎配置

## 开发指南

### 编译单个功能包

```bash
colcon build --packages-select qyh_jaka_control
```

### 清理编译

```bash
rm -rf build/ install/ log/
```

### 查看节点和话题

```bash
# 查看运行的节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看所有服务
ros2 service list

# 查看话题详情
ros2 topic echo /robot/status
```

### 调试技巧

```bash
# 设置日志级别为DEBUG
ros2 run qyh_jaka_control jaka_control_node --ros-args --log-level debug

# 使用rqt查看节点图
rqt_graph

# 录制和回放bag
ros2 bag record -a  # 录制所有话题
ros2 bag play <bag_file>
```

## 常见问题

### 1. 编译错误：找不到JAKA SDK

确保JAKA SDK库文件位于 `qyh_jaka_control/thirdparty/` 目录下。

### 2. Modbus连接失败

检查网络连接和IP配置：
```bash
ping <robot_ip>
```

### 3. 权限不足

串口设备需要权限：
```bash
sudo usermod -a -G dialout $USER
# 重新登录生效
```

## 目录结构

```
qyh_jushen_ws/
├── src/                          # 源代码
│   ├── bringup/                  # 启动配置
│   ├── OrbbecSDK_ROS2/          # 相机驱动
│   ├── qyh_bag_recorder/        # Bag录制
│   ├── qyh_dual_arm_teleop_python/  # 双臂遥操作
│   ├── qyh_dual_arms_description/   # 机器人描述
│   ├── qyh_gripper_control/     # 夹爪控制
│   ├── qyh_gripper_msgs/        # 夹爪消息
│   ├── qyh_head_motor_control/  # 头部控制
│   ├── qyh_jaka_control/        # 机械臂控制
│   ├── qyh_jaka_control_msgs/   # 机械臂消息
│   ├── qyh_lift_control/        # 升降控制
│   ├── qyh_lift_msgs/           # 升降消息
│   ├── qyh_shutdown/            # 关机管理
│   ├── qyh_standard_robot/      # 底盘控制
│   ├── qyh_standard_robot_msgs/ # 底盘消息
│   ├── qyh_task_engine/         # 任务引擎
│   ├── qyh_task_engine_msgs/    # 任务消息
│   ├── qyh_vr_bridge/           # VR桥接
│   ├── qyh_vr_button_event/     # VR按钮
│   ├── qyh_waist_control/       # 腰部控制
│   └── qyh_waist_msgs/          # 腰部消息
├── build/                        # 编译输出（gitignore）
├── install/                      # 安装目录（gitignore）
└── log/                          # 日志（gitignore）
```

## 许可证

[待补充]

## 维护者

千岩护机器人团队
