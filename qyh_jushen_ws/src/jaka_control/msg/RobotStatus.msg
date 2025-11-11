std_msgs/Header header

# Robot state
bool powered_on
bool servo_enabled
bool estoped
bool in_error
int32 error_code

# Left arm joint positions
float64[7] left_joint_positions

# Right arm joint positions
float64[7] right_joint_positions

# Left arm Cartesian pose
geometry_msgs/Pose left_cartesian_pose

# Right arm Cartesian pose
geometry_msgs/Pose right_cartesian_pose

# In position status
bool left_in_position
bool right_in_position
