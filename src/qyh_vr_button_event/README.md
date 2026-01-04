我们的 `qyh_vr_button_event` 需要做以下几件事情：

- 订阅由 `vr_bridge` 发布的手柄消息（Joy 消息）。
- 根据手柄消息执行动作：
	- 左手 A/B 控制左夹爪开/合；右手 A/B 控制右夹爪开/合。
	- 左手摇杆前后控制升降电机上升/下降；左右控制腰部下弯/回正（腰部使用绝对角度命令）。
	- 右手摇杆前后控制底盘前进/后退；左右控制底盘转向（发布 `ManualVelocityCommand`）。

默认的 ROS 话题/服务（已在 `config/config.yaml` 和节点默认参数中同步）：

- 左手 Joy: `/vr/left_controller/joy` (sensor_msgs/msg/Joy)
- 右手 Joy: `/vr/right_controller/joy` (sensor_msgs/msg/Joy)
- 左夹爪移动服务: `/left/move_gripper` (qyh_gripper_msgs/srv/MoveGripper)
- 右夹爪移动服务: `/right/move_gripper` (qyh_gripper_msgs/srv/MoveGripper)
- 升降控制服务: `/lift/control` (qyh_lift_msgs/srv/LiftControl)
- 腰部控制服务: `/waist/control` (qyh_waist_msgs/srv/WaistControl)
- 腰部状态话题: `/waist/state` (qyh_waist_msgs/msg/WaistState)
- 手动速度发布话题: `/manual_velocity_cmd` (qyh_standard_robot_msgs/msg/ManualVelocityCommand)

重要行为说明：

- 为了防止 `vr_bridge` 丢帧或抖频导致底层认为“控制中断”，节点会在右摇杆处于非零时保存最近的速度命令并以 `chassis_keepalive_ms`（默认 100 ms）频率补发，确保底盘每 100 ms 收到一次命令以维持控制。超出 `joy_timeout_ms`（默认 500 ms）将发布零速度以停止底盘。

运行方法：

```bash
# 在工作区根目录
colcon build --packages-select qyh_vr_button_event
source install/setup.bash
ros2 launch qyh_vr_button_event qyh_vr_button_event.launch.py
```

如需查看当前系统上的话题/服务名称：

```bash
ros2 topic list
ros2 service list
ros2 topic echo /vr/left_controller/joy --once
```

如果要修改名称或参数，可编辑 `config/config.yaml`，或通过 launch/ROS 参数覆盖。