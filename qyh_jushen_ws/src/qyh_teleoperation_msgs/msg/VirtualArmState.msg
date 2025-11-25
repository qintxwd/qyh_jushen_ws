std_msgs/Header header

string arm_id  # "left" or "right"
sensor_msgs/JointState joint_state
geometry_msgs/Pose end_effector_pose
float64[] joint_velocities
bool is_tracking  # 是否正在跟踪VR
