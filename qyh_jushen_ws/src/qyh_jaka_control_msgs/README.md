# qyh_jaka_control_msgs

该功能包定义了JAKA双臂机器人控制系统所需的自定义消息（Messages）和服务（Services）。

## 消息定义 (Messages)

### JakaDualJointServo.msg
用于双臂关节空间的伺服控制指令。
- `float64[14] positions`: 双臂关节位置数组（左臂7个 + 右臂7个）。
- `bool is_abs`: 是否为绝对位置控制（True=绝对，False=相对）。
- `float64 velocity`: 运动速度。
- `float64 acceleration`: 运动加速度。

### JakaDualCartesianServo.msg
用于双臂笛卡尔空间的伺服控制指令。
- `geometry_msgs/Pose left_pose`: 左臂目标位姿。
- `geometry_msgs/Pose right_pose`: 右臂目标位姿。
- `bool is_abs`: 是否为绝对位姿控制。

### JakaServoStatus.msg
伺服控制回路的实时状态反馈。
- `string mode`: 当前控制模式（如 "joint", "cartesian"）。
- `bool is_abs`: 当前是否为绝对模式。
- `int32 cycle_time_ns`: 控制周期（纳秒）。
- `float64 publish_rate_hz`: 实际发布频率（Hz）。
- `float64 latency_ms`: 控制回路延迟（毫秒）。
- `float64 packet_loss_rate`: 丢包率估计。
- `int32 error_code`: 错误码（0表示正常）。

### VRPose.msg
VR控制器的位姿数据。
- `geometry_msgs/Pose pose`: 控制器的位置和姿态。
- `string controller_id`: 控制器标识符（如 "left", "right"）。
- `float64 battery_level`: 电池电量。

### VRFollowStatus.msg
VR跟随模式的状态信息。
- `string left_arm_status`: 左臂跟随状态。
- `string right_arm_status`: 右臂跟随状态。
- `float64 left_pose_error`: 左臂跟踪误差。
- `float64 right_pose_error`: 右臂跟踪误差。
- `int32 recorded_frames`: 已录制的帧数（如果在录制中）。
- `float64 recording_duration`: 录制时长（秒）。

## 服务定义 (Services)

### 伺服控制类
- **StartServo.srv**: 启动实时伺服控制模式。
    - 请求: 无
    - 响应: `bool success`, `string message`
- **StopServo.srv**: 停止实时伺服控制模式。
    - 请求: 无
    - 响应: `bool success`, `string message`
- **SetFilter.srv**: 配置运动滤波器参数。
    - 请求: 
        - `string filter_type`: 滤波器类型 ("none", "joint_lpf", "joint_nlf")
        - `float64 cutoff_frequency`: 截止频率 (用于LPF)
        - `float64 max_joint_velocity`: 最大关节速度 (用于NLF)
        - `float64 max_joint_acceleration`: 最大关节加速度 (用于NLF)
        - `float64 max_joint_jerk`: 最大关节加加速度 (用于NLF)
    - 响应: `bool success`, `string message`

### VR遥操作类
- **EnableVRFollow.srv**: 开启/关闭VR跟随功能。
    - 请求: `bool enable`
    - 响应: `bool success`, `string message`
- **CalibrateVR.srv**: 校准VR坐标系与机器人基坐标系的变换关系。
    - 请求: 无
    - 响应: `bool success`, `string message`

### 数据录制类
- **StartRecording.srv**: 开始录制机器人状态和VR数据。
    - 请求: `string output_path` (输出目录路径)
    - 响应: `bool success`, `string message`
- **StopRecording.srv**: 停止录制并保存文件。
    - 请求: 无
    - 响应: `bool success`, `int32 total_frames`, `float64 duration`, `string saved_file`, `string message`

### 力控/传感器类 (如有)
- **SetFTConfig.srv**: 配置力矩传感器。
- **ZeroFT.srv**: 力矩传感器清零。
