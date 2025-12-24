# JAKA双臂机器人控制GUI

## 概述

`jaka_complete_gui` 是JAKA双臂机器人的统一控制界面，提供基础控制、伺服模式、点到点运动和参数配置功能。

**注意**: VR跟随和数据录制功能已移至新的遥操作架构（`qyh_teleoperation_controller`）。

## 功能列表

### 1. 基础控制
- ✅ **连接管理**：显示机器人IP和连接状态
- ✅ **电源控制**：
  - Power On (上电)
  - Power Off (下电)
- ✅ **使能控制**：
  - Enable (使能)
  - Disable (去使能)
- ✅ **错误处理**：
  - Clear Error (清除错误)
  - Motion Abort (急停)

### 2. 点到点运动
- ✅ **关节空间运动 (MoveJ)**
- ✅ **笛卡尔空间直线运动 (MoveL)**
- ✅ **可配置速度和加速度**

### 3. 伺服模式控制
- ✅ **启动/停止伺服模式**
- ✅ **支持关节空间伺服**（JakaDualJointServo）
- ✅ **支持笛卡尔空间伺服**（JakaDualCartesianServo）
- ✅ **125Hz实时控制**
- ✅ **伺服测试功能**（正弦波运动测试）

### 4. 参数配置
- ✅ **碰撞等级设置**
- ✅ **工具偏移设置**
- ✅ **机器人状态查询**

### 5. 状态监控
- ✅ **实时周期时间监控**
- ✅ **发布频率监控**
- ✅ **延迟监控**
- ✅ **控制模式显示**
- ✅ **机器人完整状态显示**
- ✅ **关节位置和笛卡尔位姿显示**

## 界面布局

### 标签页1：基础控制
```
┌─────────────────────────────────────┐
│ ● 连接状态                           │
│   左臂IP: 192.168.1.10  [已连接]    │
│   右臂IP: 192.168.1.11  [已连接]    │
├─────────────────────────────────────┤
│ ● 电源与使能                         │
│   电源状态: [已上电]                 │
│   [上电] [下电]                      │
│   使能状态: [已使能]                 │
│   [使能] [去使能]                    │
├─────────────────────────────────────┤
│ ● 错误处理                           │
│   错误状态: [无错误]                 │
│   [清除错误] [急停]                  │
└─────────────────────────────────────┘
```

### 标签页2: 点到点运动
```
┌─────────────────────────────────────┐
│ ● 关节空间运动 (MoveJ)               │
│   机器人ID: -1 (双臂)                │
│   关节位置: [0,0,0,0,0,0,0...]      │
│   速度: 1.0 rad/s                    │
│   加速度: 0.5 rad/s²                 │
│   [执行MoveJ]                        │
├─────────────────────────────────────┤
│ ● 笛卡尔空间直线运动 (MoveL)         │
│   机器人ID: 0 (左臂)                 │
│   目标位姿: X,Y,Z,qx,qy,qz,qw       │
│   速度: 100.0 mm/s                   │
│   加速度: 50.0 mm/s²                 │
│   [执行MoveL]                        │
└─────────────────────────────────────┘
```

### 标签页3: 伺服模式
```
┌─────────────────────────────────────┐
│ ● 伺服模式控制                       │
│   伺服状态: [已启动]                 │
│   [启动伺服模式] [停止伺服模式]      │
├─────────────────────────────────────┤
│ ● 伺服测试（正弦波运动）             │
│   测试状态: [未启动]                 │
│   [开始测试] [停止测试]              │
├─────────────────────────────────────┤
│ ● 使用说明                           │
│   1. 伺服模式用于接收实时命令        │
│   2. 由 qyh_teleoperation_controller │
│      发送关节/笛卡尔命令              │
│   3. 125Hz高频控制循环               │
└─────────────────────────────────────┘
```

### 标签页4: 参数配置
```
┌─────────────────────────────────────┐
│ ● 碰撞检测                           │
│   碰撞等级: 3 (0-5)                  │
│   [设置碰撞等级]                     │
├─────────────────────────────────────┤
│ ● 工具坐标系                         │
│   X: 0.0  Y: 0.0  Z: 0.15           │
│   [设置工具偏移]                     │
└─────────────────────────────────────┘
```

### 标签页5: 状态监控
```
┌─────────────────────────────────────┐
│ ● 实时系统状态                       │
│   控制模式: joint                    │
│   周期时间: 8.0 ms                   │
│   发布频率: 125.0 Hz                 │
│   延迟: 0.5 ms                       │
└─────────────────────────────────────┘
```

## 使用流程

### 基础启动流程
```bash
# 1. 编译
cd ~/qyh_jushen_ws
colcon build --packages-select qyh_jaka_control_msgs qyh_jaka_control qyh_jaka_control_gui
source install/setup.bash

# 2. 启动GUI
ros2 launch qyh_jaka_control_gui jaka_gui.launch.py

# 或者直接运行
ros2 run qyh_jaka_control_gui jaka_gui
```

### 完整操作流程

#### 场景1：基础运动控制
1. 启动GUI
2. 点击"上电 (Power On)"
3. 点击"使能 (Enable)"
4. 切换到"伺服模式"标签页
5. 点击"启动伺服模式"
6. 通过ROS话题发送控制指令

#### 场景2：VR跟随控制
1. 完成基础启动（上电→使能→启动伺服）
2. 切换到"VR跟随"标签页
3. 首次使用点击"校准VR坐标系"
4. 点击"启用VR跟随"
5. 通过VR控制器控制机械臂

#### 场景3：数据录制（具身智能训练）
1. 完成VR跟随启用
2. 设置"输出路径"（如 `/home/user/datasets/demo01`）
3. 调整"录制频率"（推荐125Hz）
4. 点击"开始录制"
5. 进行VR操作演示
6. 点击"停止录制"
7. 查看保存的CSV文件

#### 场景4：错误处理
- 如果机器人报错：点击"清除错误"
- 如果需要紧急停止：点击"急停 (Abort)"
- 如果需要完全停止：点击"停止伺服模式" → "去使能" → "下电"

## ROS接口

### 发布的话题
- `/jaka/servo/joint_cmd` (JakaDualJointServo) - 关节空间指令
- `/jaka/servo/cartesian_cmd` (JakaDualCartesianServo) - 笛卡尔空间指令

### 调用的服务
**基础控制：**
- `/jaka/robot/power_on` (Trigger) - 上电
- `/jaka/robot/power_off` (Trigger) - 下电
- `/jaka/robot/enable` (Trigger) - 使能
- `/jaka/robot/disable` (Trigger) - 去使能
- `/jaka/robot/clear_error` (Trigger) - 清除错误
- `/jaka/robot/motion_abort` (Trigger) - 急停

**伺服控制：**
- `/jaka/servo/start` (StartServo) - 启动伺服
- `/jaka/servo/stop` (StopServo) - 停止伺服

**VR控制：**
- `/jaka/vr/enable` (EnableVRFollow) - 启用/停止VR跟随
- `/jaka/vr/calibrate` (CalibrateVR) - 校准VR坐标系
- `/jaka/vr/start_recording` (StartRecording) - 开始录制
- `/jaka/vr/stop_recording` (StopRecording) - 停止录制

## 与原jaka_control的对比

| 功能 | 原jaka_control | jaka_complete_gui |
|------|----------------|-------------------|
| 连接管理 | ✅ 命令行 | ✅ GUI显示 |
| 电源控制 | ✅ 服务调用 | ✅ 按钮操作 |
| 使能控制 | ✅ 服务调用 | ✅ 按钮操作 |
| 清除错误 | ✅ 服务调用 | ✅ 按钮操作 |
| 急停 | ✅ 服务调用 | ✅ 按钮操作 |
| 伺服模式 | ❌ 无 | ✅ 完整支持 |
| VR跟随 | ❌ 无 | ✅ 完整支持 |
| 数据录制 | ❌ 无 | ✅ 完整支持 |
| 状态监控 | ❌ 无 | ✅ 实时显示 |
| 用户体验 | 命令行 | 图形界面 |

## 数据格式

录制的CSV文件包含：
- `timestamp` - 时间戳
- `left_vr_x/y/z, left_vr_qx/qy/qz/qw` - 左VR位姿
- `right_vr_x/y/z, right_vr_qx/qy/qz/qw` - 右VR位姿
- `left_joint_0~5, right_joint_0~5` - 双臂关节角度
- `left_joint_vel_0~5, right_joint_vel_0~5` - 双臂关节速度
- `left_cart_x/y/z/rx/ry/rz` - 左臂笛卡尔位姿
- `right_cart_x/y/z/rx/ry/rz` - 右臂笛卡尔位姿

## 配置说明

编辑 `src/qyh_jaka_control/config/robot_config.yaml`：

```yaml
robot:
  left_arm_ip: "192.168.1.10"
  right_arm_ip: "192.168.1.11"
  
servo:
  cycle_time_ms: 8
  control_mode: "joint"  # joint 或 cartesian
  
vr_following:
  enabled: true
  scale_factor: 1.0
  offset_x: 0.0
  offset_y: 0.0
  offset_z: 0.5
  
recording:
  default_frequency: 125.0
  default_path: "/tmp/jaka_recordings"
```

## 故障排查

### GUI无法启动
```bash
# 检查依赖
pip install PyQt5
# 检查ROS环境
source install/setup.bash
```

### 服务调用失败
```bash
# 检查控制节点是否运行
ros2 node list | grep jaka
# 检查服务是否可用
ros2 service list | grep jaka
```

### VR跟随不准确
1. 点击"校准VR坐标系"
2. 调整配置文件中的VR参数
3. 检查TF变换：`ros2 run tf2_tools view_frames`

## 安全注意事项

⚠️ **使用前必读**
1. 确保机器人工作空间无障碍物
2. 首次使用VR跟随时使用低速模式
3. 熟悉急停按钮位置
4. 录制数据前先进行空跑测试
5. 异常情况立即点击"急停"或"停止伺服模式"

## 开发者信息

- 包名：`qyh_jaka_control_gui`
- 可执行文件：`jaka_gui`
- 启动文件：`jaka_gui.launch.py`
- Python包：`qyh_jaka_control_gui.jaka_complete_gui`

## 相关文档

- [JAKA SDK文档](../qyh_jaka_control/README.md)
- [VR跟随配置指南](../qyh_jaka_control/docs/vr_following.md)
- [数据录制格式说明](../qyh_jaka_control/docs/recording_format.md)
