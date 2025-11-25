# VR遥操作双臂机械臂系统 - 完整开发大纲

## 📋 项目概述

**目标**：构建一个基于VR的双臂机械臂遥操作系统，实现平滑、安全的模仿学习数据采集。

**核心问题**：当前直接伺服控制导致机械臂频繁报错（位置跳变过大）

**解决方案**：采用主流的 **VR → 虚拟机械臂 → 差分IK → 轨迹平滑 → 真实机械臂** 架构

---

## 🏗️ 系统架构图

```
┌─────────────┐     ┌──────────────┐     ┌────────────────┐
│  Pico4 VR   │────>│  VR Pose     │────>│  Virtual Arms  │
│  手柄追踪    │     │  Subscriber  │     │  (MoveIt2)     │
└─────────────┘     └──────────────┘     └────────────────┘
                                                   │
                                                   ▼
                                          ┌─────────────────┐
                                          │  Differential   │
                                          │  IK Solver      │
                                          └─────────────────┘
                                                   │
                                                   ▼
                                          ┌─────────────────┐
                                          │  Trajectory     │
                                          │  Smoother       │
                                          │  (限速/限加速度) │
                                          └─────────────────┘
                                                   │
                                                   ▼
                                          ┌─────────────────┐
                                          │  Real Robot     │
                                          │  Interface      │
                                          │  (125Hz Servo)  │
                                          └─────────────────┘
                                                   │
                                                   ▼
                                          ┌─────────────────┐
                                          │  JAKA Dual Arms │
                                          │  (Real Hardware)│
                                          └─────────────────┘
```

---

## 📦 ROS2 Package 架构

### 1️⃣ **qyh_dual_arms_description** (机械臂描述包)
**状态**: 🔄 需要修改现有 Dual-Arms 包

#### 功能
- 双臂机械臂URDF模型
- **关键配置**: 修正双臂安装位置
  - 间距: 18cm (Y轴偏移 ±9cm)
  - 朝向: 各向前旋转45°
  - 基座配置: 类似 `\_/` 俯视布局

#### 目录结构
```
qyh_dual_arms_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── dual_arms.urdf.xacro        # 主URDF (参数化)
│   ├── dual_arms_macro.xacro       # 单臂宏定义
│   ├── left_arm.xacro              # 左臂配置 (base +9cm Y, +45° Z)
│   ├── right_arm.xacro             # 右臂配置 (base -9cm Y, -45° Z)
│   ├── gazebo.xacro                # Gazebo插件
│   └── ros2_control.xacro          # ros2_control配置
├── meshes/                         # 复用Dual-Arms的mesh
├── config/
│   ├── joint_limits.yaml           # 关节限制
│   └── physical_properties.yaml   # 物理属性
└── launch/
    ├── display.launch.py           # RViz可视化
    └── load_description.launch.py # 加载URDF到参数服务器
```

#### 关键修改点
```xml
<!-- 左臂基座 -->
<xacro:arm_macro prefix="left" 
                 base_x="0.0" 
                 base_y="0.09"  <!-- +9cm -->
                 base_z="0.217"
                 base_yaw="0.785398"/>  <!-- +45° = π/4 -->

<!-- 右臂基座 -->
<xacro:arm_macro prefix="right" 
                 base_x="0.0" 
                 base_y="-0.09"  <!-- -9cm -->
                 base_z="0.217"
                 base_yaw="-0.785398"/>  <!-- -45° -->
```

#### 依赖
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`

---

### 2️⃣ **qyh_dual_arms_moveit_config** (MoveIt2配置包)
**状态**: 🆕 新建

#### 功能
- MoveIt2运动规划配置
- 虚拟机械臂运动学求解
- 碰撞检测
- 差分IK求解器

#### 目录结构
```
qyh_dual_arms_moveit_config/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── dual_arms.srdf              # 语义描述
│   ├── kinematics.yaml             # IK求解器配置
│   ├── joint_limits.yaml           # 关节限制
│   ├── pilz_cartesian_limits.yaml # 笛卡尔限制
│   ├── moveit_controllers.yaml    # 虚拟控制器
│   └── sensors_3d.yaml             # 深度相机配置（可选）
└── launch/
    ├── setup_assistant.launch.py  # MoveIt Setup Assistant
    ├── demo.launch.py              # 演示
    ├── move_group.launch.py        # MoveGroup节点
    └── moveit_rviz.launch.py       # RViz可视化
```

#### 规划组配置
- `left_arm`: 左臂7自由度
- `right_arm`: 右臂7自由度
- `dual_arms`: 双臂联合规划（14自由度）
- `left_gripper`: 左手夹爪（可选）
- `right_gripper`: 右手夹爪（可选）

#### IK求解器
使用 **KDL** 或 **TracIK** 求解器

```yaml
kinematics:
  left_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
  right_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.05
```

#### 依赖
- `moveit_ros_planning_interface`
- `moveit_ros_move_group`
- `moveit_kinematics`
- `moveit_planners_ompl`
- `moveit_servo`  # 用于差分IK

---

### 3️⃣ **qyh_vr_interface** (VR接口包)
**状态**: 🔄 增强现有 qyh_vr_calibration

#### 功能
- 接收Pico4 VR手柄位姿
- VR空间到机器人空间的坐标变换
- 手柄按键事件处理
- VR数据平滑滤波

#### 目录结构
```
qyh_vr_interface/
├── CMakeLists.txt
├── package.xml
├── include/qyh_vr_interface/
│   ├── vr_pose_receiver.hpp
│   ├── coordinate_transformer.hpp
│   └── button_handler.hpp
├── src/
│   ├── vr_pose_receiver.cpp
│   ├── coordinate_transformer.cpp
│   ├── button_handler.cpp
│   └── vr_interface_node.cpp
├── config/
│   ├── vr_transforms.yaml        # VR到机器人坐标系变换
│   └── button_mappings.yaml     # 按键映射配置
└── launch/
    └── vr_interface.launch.py
```

#### 话题接口
**订阅**:
- `/vr/left_hand/pose` (geometry_msgs/PoseStamped) - 左手位姿
- `/vr/right_hand/pose` (geometry_msgs/PoseStamped) - 右手位姿
- `/vr/buttons` (sensor_msgs/Joy) - 按键状态

**发布**:
- `/vr/left_target_pose` (geometry_msgs/PoseStamped) - 左臂目标
- `/vr/right_target_pose` (geometry_msgs/PoseStamped) - 右臂目标
- `/vr/control_mode` (std_msgs/String) - 控制模式状态

#### 依赖
- `tf2_ros`
- `geometry_msgs`
- `sensor_msgs`
- `qyh_vr_calibration_msgs`

---

### 4️⃣ **qyh_teleoperation_controller** (遥操作控制器 - 核心包)
**状态**: 🆕 新建

#### 功能
- **差分IK求解**: VR姿态 → 关节速度
- **轨迹平滑**: 速度/加速度/jerk限制
- **安全检查**: 关节限位、碰撞检测
- **虚拟机械臂跟随**: MoveIt Servo
- **数据录制**: 用于模仿学习

#### 目录结构
```
qyh_teleoperation_controller/
├── CMakeLists.txt
├── package.xml
├── include/qyh_teleoperation_controller/
│   ├── differential_ik_controller.hpp
│   ├── trajectory_smoother.hpp
│   ├── safety_checker.hpp
│   ├── virtual_arm_follower.hpp
│   └── data_recorder.hpp
├── src/
│   ├── differential_ik_controller.cpp    # Diff IK核心
│   ├── trajectory_smoother.cpp           # 轨迹平滑
│   ├── safety_checker.cpp                # 安全检查
│   ├── virtual_arm_follower.cpp          # 虚拟臂跟随
│   ├── data_recorder.cpp                 # 数据记录
│   └── teleoperation_node.cpp            # 主节点
├── config/
│   ├── controller_params.yaml            # 控制器参数
│   ├── safety_limits.yaml                # 安全限制
│   └── smoother_config.yaml              # 平滑器配置
└── launch/
    └── teleoperation_controller.launch.py
```

#### 核心算法

**1. 差分IK (Differential IK)**
```cpp
// 核心公式: dq = J^(-1) * dx
// J: 雅可比矩阵
// dx: 末端位姿变化 (从VR)
// dq: 关节速度

Eigen::VectorXd computeJointVelocities(
    const Eigen::Isometry3d& target_pose,
    const Eigen::Isometry3d& current_pose,
    double dt) {
    
    // 计算位姿差
    Eigen::Vector6d pose_delta = computePoseDelta(target_pose, current_pose);
    
    // 获取雅可比矩阵
    Eigen::MatrixXd jacobian = robot_model->getJacobian();
    
    // 伪逆求解
    Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    
    // 计算关节速度
    Eigen::VectorXd joint_velocities = jacobian_pinv * (pose_delta / dt);
    
    return joint_velocities;
}
```

**2. 轨迹平滑器 (Trajectory Smoother)**
```cpp
class TrajectorySmoother {
public:
    struct Limits {
        double max_velocity;      // 最大速度 (rad/s)
        double max_acceleration;  // 最大加速度 (rad/s²)
        double max_jerk;          // 最大加加速度 (rad/s³)
    };
    
    JointState smoothTrajectory(
        const JointState& target,
        const JointState& current,
        const JointState& previous,
        double dt);
        
private:
    // 速度限幅
    void clampVelocity(JointState& state);
    
    // 加速度限幅
    void clampAcceleration(JointState& state, const JointState& prev);
    
    // Jerk限幅
    void clampJerk(JointState& state, const JointState& prev, const JointState& prev_prev);
    
    // 低通滤波
    void applyLowPassFilter(JointState& state);
};
```

**3. 安全检查器**
```cpp
class SafetyChecker {
public:
    enum class SafetyStatus {
        SAFE,
        JOINT_LIMIT_VIOLATION,
        VELOCITY_LIMIT_VIOLATION,
        COLLISION_DETECTED,
        SINGULARITY_NEAR
    };
    
    SafetyStatus checkSafety(
        const JointState& planned_state,
        const robot_model::RobotModelPtr& robot_model);
};
```

#### 配置参数示例
```yaml
# controller_params.yaml
differential_ik:
  update_rate: 125.0  # Hz
  position_tolerance: 0.001  # m
  orientation_tolerance: 0.01  # rad
  damping_factor: 0.01

trajectory_smoother:
  max_joint_velocity: 1.0  # rad/s (保守值)
  max_joint_acceleration: 0.5  # rad/s²
  max_joint_jerk: 5.0  # rad/s³
  max_cartesian_velocity: 0.05  # m/s
  max_cartesian_acceleration: 0.1  # m/s²
  low_pass_filter_cutoff: 10.0  # Hz

safety_limits:
  min_distance_to_singularity: 0.05
  collision_check_distance: 0.02  # m
  joint_limit_margin: 0.05  # rad
```

#### 依赖
- `moveit_servo`
- `moveit_core`
- `eigen3`
- `control_toolbox`
- `qyh_dual_arms_moveit_config`

---

### 5️⃣ **qyh_robot_bridge** (机器人桥接包)
**状态**: 🔄 增强现有 qyh_jaka_control

#### 功能
- 平滑轨迹 → JAKA SDK伺服指令
- 实时位置反馈
- 错误处理与恢复
- 性能监控

#### 目录结构
```
qyh_robot_bridge/
├── CMakeLists.txt
├── package.xml
├── include/qyh_robot_bridge/
│   ├── smooth_servo_interface.hpp   # 新增平滑伺服接口
│   ├── trajectory_buffer.hpp        # 轨迹缓冲
│   └── performance_monitor.hpp      # 性能监控
├── src/
│   ├── smooth_servo_interface.cpp
│   ├── trajectory_buffer.cpp
│   ├── performance_monitor.cpp
│   └── robot_bridge_node.cpp
└── config/
    └── servo_config.yaml
```

#### 新增功能

**轨迹缓冲器**
```cpp
class TrajectoryBuffer {
public:
    // 缓冲多个轨迹点，平滑插值
    void addTrajectoryPoint(const JointState& state, double timestamp);
    
    // 获取插值后的指令
    JointState getSmoothedCommand(double current_time);
    
private:
    std::deque<TrajectoryPoint> buffer_;
    size_t buffer_size_ = 10;  // 缓冲10个点 (~80ms)
};
```

**性能监控**
```cpp
class PerformanceMonitor {
public:
    struct Metrics {
        double control_loop_frequency;  // 实际控制频率
        double command_latency;         // 指令延迟
        double trajectory_smoothness;   // 轨迹平滑度
        size_t error_count;             // 错误次数
    };
    
    Metrics getMetrics();
    void publishMetrics();  // 发布到ROS话题
};
```

#### 修改现有接口
```cpp
// jaka_interface.cpp 增强
class JakaInterface {
public:
    // 新增：平滑伺服接口
    bool smoothServoJ(
        int robot_id,
        const std::vector<double>& joint_positions,
        const std::vector<double>& joint_velocities,  // 新增速度信息
        bool is_abs);
    
    // 新增：速度伺服接口
    bool velocityServoJ(
        int robot_id,
        const std::vector<double>& joint_velocities);
};
```

#### 依赖
- `qyh_jaka_control` (现有)
- `qyh_jaka_control_msgs`
- `control_msgs`

---

### 6️⃣ **qyh_teleoperation_msgs** (消息定义包)
**状态**: 🆕 新建

#### 功能
- 定义系统内部通信消息
- 服务接口定义

#### 目录结构
```
qyh_teleoperation_msgs/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── VirtualArmState.msg       # 虚拟臂状态
│   ├── TeleopStatus.msg          # 遥操作状态
│   ├── SafetyStatus.msg          # 安全状态
│   └── PerformanceMetrics.msg    # 性能指标
└── srv/
    ├── StartTeleoperation.srv    # 启动遥操作
    ├── StopTeleoperation.srv     # 停止遥操作
    └── SetControlMode.srv        # 设置控制模式
```

#### 消息定义

**VirtualArmState.msg**
```
std_msgs/Header header
string arm_id  # "left" or "right"
sensor_msgs/JointState joint_state
geometry_msgs/Pose end_effector_pose
float64[] joint_velocities
bool is_tracking  # 是否正在跟踪VR
```

**TeleopStatus.msg**
```
std_msgs/Header header
uint8 IDLE = 0
uint8 TRACKING = 1
uint8 PAUSED = 2
uint8 ERROR = 3
uint8 status
string message
float64 tracking_quality  # 0.0-1.0
```

**SafetyStatus.msg**
```
std_msgs/Header header
bool is_safe
string[] warnings
string[] errors
float64 distance_to_singularity
float64 min_collision_distance
```

**PerformanceMetrics.msg**
```
std_msgs/Header header
float64 control_frequency  # Hz
float64 average_latency    # ms
float64 max_latency        # ms
float64 trajectory_smoothness  # 0.0-1.0
uint32 error_count
```

---

### 7️⃣ **qyh_data_collection** (数据采集包)
**状态**: 🆕 新建

#### 功能
- 模仿学习数据采集
- 轨迹记录与回放
- 数据集管理

#### 目录结构
```
qyh_data_collection/
├── CMakeLists.txt
├── package.xml
├── include/qyh_data_collection/
│   ├── trajectory_recorder.hpp
│   ├── data_synchronizer.hpp
│   └── dataset_manager.hpp
├── src/
│   ├── trajectory_recorder.cpp
│   ├── data_synchronizer.cpp
│   ├── dataset_manager.cpp
│   └── data_collection_node.cpp
├── scripts/
│   ├── visualize_trajectory.py
│   └── convert_to_hdf5.py
└── launch/
    └── data_collection.launch.py
```

#### 数据格式
```python
# HDF5数据集结构
dataset/
├── episode_0001/
│   ├── timestamp          # [N] 时间戳
│   ├── vr_left_pose       # [N, 7] VR左手位姿 (x,y,z,qx,qy,qz,qw)
│   ├── vr_right_pose      # [N, 7] VR右手位姿
│   ├── robot_left_joint   # [N, 7] 左臂关节角度
│   ├── robot_right_joint  # [N, 7] 右臂关节角度
│   ├── robot_left_vel     # [N, 7] 左臂关节速度
│   ├── robot_right_vel    # [N, 7] 右臂关节速度
│   ├── gripper_left       # [N, 1] 左夹爪状态
│   ├── gripper_right      # [N, 1] 右夹爪状态
│   └── camera_image       # [N, H, W, 3] 相机图像（可选）
└── episode_0002/
    └── ...
```

---

### 8️⃣ **qyh_teleoperation_gui** (监控GUI包)
**状态**: 🆕 新建

#### 功能
- 实时监控双臂状态
- 虚拟/真实机械臂对比显示
- 性能指标可视化
- 数据录制控制

#### 目录结构
```
qyh_teleoperation_gui/
├── CMakeLists.txt
├── package.xml
├── qyh_teleoperation_gui/
│   ├── __init__.py
│   ├── main_window.py
│   ├── arm_status_widget.py
│   ├── performance_widget.py
│   └── recording_widget.py
├── resources/
│   └── icons/
├── setup.py
└── launch/
    └── teleoperation_gui.launch.py
```

#### GUI界面布局
```
┌────────────────────────────────────────────────┐
│  VR遥操作控制台                                  │
├───────────────┬────────────────────────────────┤
│ 左臂状态       │ 右臂状态                        │
│ - 虚拟位置     │ - 虚拟位置                      │
│ - 真实位置     │ - 真实位置                      │
│ - 跟踪质量     │ - 跟踪质量                      │
├───────────────┴────────────────────────────────┤
│ 性能监控                                        │
│ - 控制频率: 125.0 Hz                           │
│ - 延迟: 8.2 ms                                 │
│ - 平滑度: 0.95                                  │
├────────────────────────────────────────────────┤
│ 数据录制                                        │
│ [开始录制] [停止录制] [保存]                    │
│ 已录制: 1234帧 (9.8秒)                         │
├────────────────────────────────────────────────┤
│ 系统日志                                        │
│ [INFO] Tracking started                        │
│ [WARN] Joint velocity limit approached         │
└────────────────────────────────────────────────┘
```

---

## 🔄 系统启动流程

### Phase 1: 基础初始化
```bash
# 1. 加载机械臂描述
ros2 launch qyh_dual_arms_description display.launch.py

# 2. 启动MoveIt2
ros2 launch qyh_dual_arms_moveit_config move_group.launch.py
```

### Phase 2: VR接口
```bash
# 3. 启动VR接口
ros2 launch qyh_vr_interface vr_interface.launch.py
```

### Phase 3: 遥操作控制器
```bash
# 4. 启动遥操作控制器（核心）
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py
```

### Phase 4: 真实机器人
```bash
# 5. 连接真实机械臂
ros2 launch qyh_robot_bridge robot_bridge.launch.py robot_ip:=192.168.2.200
```

### Phase 5: 监控与录制
```bash
# 6. 启动GUI监控
ros2 launch qyh_teleoperation_gui teleoperation_gui.launch.py

# 7. 启动数据采集（需要时）
ros2 launch qyh_data_collection data_collection.launch.py
```

### 一键启动
```bash
# 创建总启动文件
ros2 launch qyh_teleoperation_bringup full_system.launch.py
```

---

## 📊 开发优先级与时间估算

### 第一阶段: 基础架构 (2-3周)
- ✅ **qyh_dual_arms_description** - 3天
  - 修改URDF模型适配实际安装位置
  - 测试RViz可视化
- ✅ **qyh_dual_arms_moveit_config** - 5天
  - MoveIt Setup Assistant配置
  - IK求解器测试
  - 碰撞检测配置
- ✅ **qyh_teleoperation_msgs** - 2天
  - 消息定义
  - 接口文档

### 第二阶段: 核心功能 (3-4周)
- ✅ **qyh_teleoperation_controller** - 10天
  - 差分IK实现 (3天)
  - 轨迹平滑器 (3天)
  - 安全检查器 (2天)
  - 集成测试 (2天)
- ✅ **qyh_robot_bridge** - 5天
  - 平滑伺服接口 (2天)
  - 轨迹缓冲 (2天)
  - 性能监控 (1天)

### 第三阶段: 接口与工具 (2周)
- ✅ **qyh_vr_interface** - 4天
  - VR姿态接收
  - 坐标变换
- ✅ **qyh_teleoperation_gui** - 4天
  - 监控界面
  - 实时可视化
- ✅ **qyh_data_collection** - 3天
  - 数据录制
  - HDF5导出

### 第四阶段: 测试与优化 (1-2周)
- 真机测试
- 参数调优
- 性能优化
- 文档完善

**总计: 8-10周**

---

## 🎯 关键技术难点与解决方案

### 难点1: 差分IK求解精度与速度
**挑战**: 125Hz高频率下IK求解
**解决**:
- 使用 MoveIt Servo 的优化雅可比求解
- 预计算常用配置的雅可比矩阵
- GPU加速（CUDA，可选）

### 难点2: 轨迹平滑与响应性平衡
**挑战**: 过度平滑导致延迟，不够平滑导致抖动
**解决**:
- 自适应平滑参数
- 基于速度的动态调整
- 可配置的平滑等级

### 难点3: 双臂协调控制
**挑战**: 左右臂同步、避免碰撞
**解决**:
- MoveIt的双臂规划组
- 自碰撞检测
- 对称运动镜像模式

### 难点4: VR抖动过滤
**挑战**: VR手柄抖动导致机械臂不稳定
**解决**:
- 卡尔曼滤波
- 低通滤波
- 死区(Deadzone)设置

---

## 📝 配置文件示例

### 主配置文件: `teleoperation_config.yaml`
```yaml
system:
  control_frequency: 125.0  # Hz
  enable_virtual_arm: true
  enable_real_arm: true
  enable_data_recording: false

arms:
  left:
    base_position: [0.0, 0.09, 0.217]   # X, Y, Z (m)
    base_orientation: [0.0, 0.0, 0.785398]  # Roll, Pitch, Yaw (rad)
    planning_group: "left_arm"
  right:
    base_position: [0.0, -0.09, 0.217]
    base_orientation: [0.0, 0.0, -0.785398]
    planning_group: "right_arm"

differential_ik:
  solver_type: "kdl"  # kdl, tracik, bio_ik
  position_tolerance: 0.001  # m
  orientation_tolerance: 0.01  # rad
  max_iterations: 50
  damping: 0.01

trajectory_smoothing:
  method: "acceleration_limited"  # velocity_limited, acceleration_limited, jerk_limited
  max_joint_velocity: 1.0  # rad/s
  max_joint_acceleration: 0.5  # rad/s²
  max_joint_jerk: 5.0  # rad/s³
  low_pass_filter:
    enable: true
    cutoff_frequency: 10.0  # Hz

safety:
  joint_limit_margin: 0.05  # rad
  velocity_limit_scale: 0.8  # 使用80%的最大速度
  collision_check_distance: 0.02  # m
  singularity_threshold: 0.05

vr_tracking:
  smoothing:
    position_filter_size: 5  # 滑动平均窗口
    orientation_filter_size: 3
  deadzone:
    position: 0.002  # m
    orientation: 0.01  # rad
  scaling:
    position: 1.0
    orientation: 1.0

data_recording:
  output_dir: "/home/jetson/teleoperation_data"
  format: "hdf5"
  recording_frequency: 30.0  # Hz (降采样)
  include_images: false
```

---

## 🚀 快速开始指南

### 1. 创建工作空间
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws/src

# 创建新包
ros2 pkg create qyh_dual_arms_description --build-type ament_cmake
ros2 pkg create qyh_dual_arms_moveit_config --build-type ament_cmake
ros2 pkg create qyh_teleoperation_controller --build-type ament_cmake
ros2 pkg create qyh_robot_bridge --build-type ament_cmake
ros2 pkg create qyh_teleoperation_msgs --build-type ament_cmake
ros2 pkg create qyh_data_collection --build-type ament_cmake
ros2 pkg create qyh_teleoperation_gui --build-type ament_python
ros2 pkg create qyh_teleoperation_bringup --build-type ament_cmake
```

### 2. 修改现有Dual-Arms包
```bash
# 复制Dual-Arms到工作空间
cp -r 资料/temp_packages/Dual-Arms qyh_dual_arms_description

# 修改package.xml和CMakeLists.txt中的包名
# 修改URDF中的基座位置
```

### 3. 构建系统
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build
source install/setup.bash
```

### 4. 测试各模块
```bash
# 测试URDF可视化
ros2 launch qyh_dual_arms_description display.launch.py

# 测试MoveIt配置
ros2 launch qyh_dual_arms_moveit_config demo.launch.py

# 测试遥操作（模拟模式）
ros2 launch qyh_teleoperation_controller teleoperation_controller.launch.py simulation:=true
```

---

## 📚 参考资料

### 学术论文
- **Differential IK**: "A Real-Time Method for 7-DOF Redundant Manipulator Control"
- **Trajectory Smoothing**: "Time-Optimal Path Tracking for Robots"
- **VR Teleoperation**: "Aloha: A Simple Framework for Bimanual Teleoperation"

### 开源项目
- **MoveIt Servo**: https://github.com/ros-planning/moveit2/tree/main/moveit_ros/moveit_servo
- **TracIK**: https://bitbucket.org/traclabs/trac_ik
- **Google RT-X**: https://robotics-transformer-x.github.io/

### ROS2文档
- MoveIt2: https://moveit.picknik.ai/humble/
- ros2_control: https://control.ros.org/humble/
- TF2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/

---

## ✅ 检查清单

在开发过程中，使用此清单跟踪进度：

### Phase 1: 模型与配置
- [ ] URDF模型更新完成
- [ ] MoveIt配置生成
- [ ] IK求解器测试通过
- [ ] RViz可视化正常

### Phase 2: 核心算法
- [ ] 差分IK实现
- [ ] 轨迹平滑器实现
- [ ] 安全检查器实现
- [ ] 单元测试通过

### Phase 3: 系统集成
- [ ] VR接口对接
- [ ] 虚拟臂跟踪
- [ ] 真实臂控制
- [ ] 数据采集

### Phase 4: 测试与优化
- [ ] 模拟环境测试
- [ ] 真机测试
- [ ] 性能优化
- [ ] 文档完善

---

## 🎓 后续扩展

### 短期 (1-3个月)
- 添加力反馈支持
- 实现自动碰撞恢复
- 优化数据集标注工具

### 中期 (3-6个月)
- 集成视觉伺服
- 添加触觉反馈
- 实现多人协作

### 长期 (6-12个月)
- 基于学习的轨迹优化
- 自适应控制参数
- 云端数据管理

---

**文档版本**: v1.0
**最后更新**: 2025-11-25
**维护者**: qyh_jushen_ws team
