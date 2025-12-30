 ./start_jaka.sh

cd ~/qyh_jushen_ws/qyh_jushen_ws/
colcon build

source ~/qyh_jushen_ws/qyh_jushen_ws/install/setup.bash 

ros2 run qyh_vr_bridge  vr_bridge_node

<!-- ros2 run qyh_dual_arm_teleop_python qyh_teleop  -->
ros2 launch qyh_dual_arm_teleop_python qyh_dual_teleop_launch.py

~/qyh_jushen_ws/qyh_jushen_ws/StartServo.sh

~/qyh_jushen_ws/qyh_jushen_ws/StopServo.sh