# VR跟随状态
std_msgs/Header header
bool following_enabled  # VR跟随是否启用
bool recording_enabled  # 是否正在录制数据
string left_arm_status  # "idle", "following", "error"
string right_arm_status  # "idle", "following", "error"
float64 left_pose_error  # 左臂跟随误差 (mm)
float64 right_pose_error  # 右臂跟随误差 (mm)
int32 recorded_frames  # 已录制帧数
float64 recording_duration  # 录制时长 (秒)
