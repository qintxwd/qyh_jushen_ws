# VR设备位姿数据
std_msgs/Header header
string device_type  # "left_controller", "right_controller", "hmd"
geometry_msgs/Pose pose  # VR设备在空间中的位姿
bool button_trigger  # 触发按钮状态
bool button_grip  # 握持按钮状态
