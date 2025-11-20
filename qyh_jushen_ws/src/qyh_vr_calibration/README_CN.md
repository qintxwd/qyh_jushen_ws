# VR标定系统说明

## 概述

统一的标定系统，支持VR用户和机器人的校准数据存储。机器人标定数据独立存储，一次设置后可供所有VR用户使用。

## 消息定义

### CalibrationPose.msg
记录单个标定动作的左右手姿态：
- `sample_index`: 动作索引（0: T-Pose, 1: 双臂前伸, 2: 双臂上举, 3: 双臂下垂）
- `left_hand_pose`: 左手/左臂笛卡尔位姿
- `right_hand_pose`: 右手/右臂笛卡尔位姿

### CalibrationProfile.msg
完整的标定配置文件：
- `username`: 用户标识
- `samples`: CalibrationPose数组（4个标定动作）

## 存储方式

### VR用户标定
路径：`~/qyh_jushen_ws/persistent/vr_calibration/<username>.yaml`

每个VR用户有独立的标定文件，记录该用户执行4个动作时VR控制器的位姿。

### 机器人标定（独立存储）
路径：`~/qyh_jushen_ws/persistent/vr_calibration_robot/robot.yaml`

机器人标定数据单独存储，记录机器人执行4个标准动作时双臂末端的笛卡尔坐标。这些数据是固定的，一次标定后所有VR用户共享使用。

## 标定流程

### 1. 机器人标定（只需一次）
机器人需要执行4个标准动作，记录双臂末端的笛卡尔坐标：
```bash
# 使用SetRobotCalibration服务
ros2 service call /vr_calibration/set_robot_calibration \
  qyh_vr_calibration_msgs/srv/SetRobotCalibration \
  "{sample: {sample_index: 0, left_hand_pose: {...}, right_hand_pose: {...}}}"

# 重复执行sample_index 1, 2, 3
```

查看机器人标定：
```bash
ros2 service call /vr_calibration/get_robot_calibration \
  qyh_vr_calibration_msgs/srv/GetRobotCalibration
```

### 2. VR用户标定（每个用户一次）
VR用户佩戴设备执行相同的4个动作，记录双手控制器的位姿：
```bash
# 使用UpdateActionSample服务，指定用户名
ros2 service call /vr_calibration/update_action_sample \
  qyh_vr_calibration_msgs/srv/UpdateActionSample \
  "{username: 'user1', sample: {sample_index: 0, left_hand_pose: {...}, right_hand_pose: {...}}}"

# 重复执行sample_index 1, 2, 3
```

### 3. 启用VR跟随
使用EnableVRFollow服务，指定VR用户名：
```bash
ros2 service call /jaka/vr/enable \
  qyh_jaka_control_msgs/srv/EnableVRFollow \
  "{enable: true, username: 'user1'}"
```

系统会：
1. 检查VR用户的标定数据是否完整（4个动作）
2. 检查机器人的标定数据是否完整（4个动作）
3. 如果任何一方缺少数据，返回失败并提示缺少的动作索引
4. 计算VR坐标系到机器人坐标系的刚体变换矩阵
5. 实时将VR控制器位姿转换为机器人末端目标位姿

## 坐标变换算法

使用基于4点对应的刚体变换计算：
1. 计算VR点集和机器人点集的质心
2. 将点集去中心化
3. 使用SVD分解计算最优旋转矩阵
4. 计算平移向量

这种方法比简单的偏移计算更精准，能够处理坐标系的旋转和平移。

## 服务接口

### VR用户相关
- `GetProfile`: 获取指定用户的标定配置
- `UpdateActionSample`: 更新单个标定动作
- `ListProfiles`: 列出所有VR用户标定配置
- `DeleteActionSamples`: 删除指定动作或全部动作
- `DeleteProfile`: 删除整个配置文件

### 机器人相关（独立服务）
- `GetRobotCalibration`: 获取机器人标定数据
- `SetRobotCalibration`: 设置机器人单个动作的标定数据

## 默认值

新建的robot.yaml会自动初始化为：
- 位置：(0, 0, 0)
- 姿态：四元数(0, 0, 0, 1)

需要实际标定后才能使用VR跟随功能。

## 错误提示

启用VR跟随时，如果标定数据不完整，系统会明确提示缺少哪些动作：
```
Incomplete calibration data. Missing poses: VR user [0,2], Robot [1]. Required: 0-3 (T-Pose, Forward, Up, Down)
```

这表示VR用户缺少动作0和2，机器人缺少动作1，需要补充标定。
