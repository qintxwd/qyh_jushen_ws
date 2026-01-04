# QYH Standard Robot Driver

This package provides a ROS2 driver for Standard Robot AGV using Modbus TCP communication protocol.

## Features

- Reads robot status via Modbus TCP
- Publishes comprehensive robot status including:
  - System and location status
  - Robot pose and velocity
  - Battery information
  - Mission and task status
  - Emergency stop and obstacle detection status

## Installation

This package requires libmodbus library which is included in the `thirdparty` directory.

Build the package:
```bash
cd /home/qyh/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_standard_robot_msgs qyh_standard_robot
source install/setup.bash
```

## Usage

### Run with default parameters:
```bash
ros2 launch qyh_standard_robot standard_robot.launch.py
```

### Run with custom IP address:
```bash
ros2 launch qyh_standard_robot standard_robot.launch.py modbus_ip:=192.168.1.100
```

### Run the node directly:
```bash
ros2 run qyh_standard_robot standard_robot_node --ros-args -p modbus_ip:=192.168.1.100
```

## Topics

### Published Topics

- `/standard_robot_status` (`qyh_standard_robot_msgs/msg/StandardRobotStatus`): Robot status information

## Parameters

- `modbus_ip` (string, default: "192.168.1.100"): Modbus TCP IP address
- `modbus_port` (int, default: 502): Modbus TCP port
- `slave_id` (int, default: 1): Modbus slave ID
- `publish_rate` (double, default: 10.0): Status publishing rate in Hz

## Communication Protocol

This driver implements the Standard Robot Modbus TCP protocol as documented in the official documentation. It reads:
- Discrete Inputs (10001-10136): Boolean status flags
- Input Registers (30001-30122): Robot state, pose, velocity, battery, and mission information

## Dependencies

- ROS2 Humble
- rclcpp
- geometry_msgs
- std_msgs
- qyh_standard_robot_msgs
- libmodbus (included)
