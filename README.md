# QYH Jushen ROS2 Workspace

千岩护（QYH）聚神机器人ROS2工作空间。

## 功能包说明

### 核心控制
- **qyh_standard_robot**: 机器人整体控制和导航
- **qyh_standard_robot_msgs**: 标准机器人消息定义
- **qyh_task_engine**: 任务引擎
- **qyh_task_engine_msgs**: 任务引擎消息定义

### 硬件控制
- **qyh_jaka_control**: 节卡机械臂控制
- **qyh_jaka_control_msgs**: 机械臂控制消息定义
- **qyh_gripper_control**: 夹爪控制
- **qyh_gripper_msgs**: 夹爪消息定义
- **qyh_head_motor_control**: 头部电机控制
- **qyh_lift_control**: 升降机构控制
- **qyh_lift_msgs**: 升降机构消息定义
- **qyh_waist_control**: 腰部控制
- **qyh_waist_msgs**: 腰部消息定义

### 遥操作
- **qyh_dual_arm_teleop_python**: 双臂遥操作
- **qyh_vr_bridge**: VR桥接
- **qyh_vr_button_event**: VR按钮事件处理

### 工具
- **qyh_bag_recorder**: ROS bag录制工具
- **qyh_shutdown**: 关机管理

### 传感器
- **OrbbecSDK_ROS2**: Orbbec相机SDK

### 配置
- **qyh_dual_arms_description**: 双臂机器人描述文件
- **bringup**: 启动配置

## 快速开始

### 1. 环境要求
- Ubuntu 22.04
- ROS2 Humble

### 2. 编译

```bash
cd qyh_jushen_ws
colcon build
```

### 3. 运行

```bash
source install/setup.bash
# 启动具体的launch文件
```

## 目录结构

```
qyh_jushen_ws/
├── src/              # 源代码
│   ├── bringup/
│   ├── qyh_*/        # 各功能包
│   └── ...
├── build/            # 编译输出（gitignore）
├── install/          # 安装目录（gitignore）
└── log/              # 日志（gitignore）
```
