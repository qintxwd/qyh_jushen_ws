cd ~/qyh_jushen_ws/qyh_jushen_ws
source /opt/ros/noetic/setup.bash
colcon build --symlink-install 
# colcon build --symlink-install --packages-select qyh_vr_calibration_msgs qyh_vr_calibration qyh_standard_robot_msgs qyh_jaka_control_msgs qyh_jaka_control qyh_standard_robot qyh_standard_robot_gui qyh_jaka_control_gui 