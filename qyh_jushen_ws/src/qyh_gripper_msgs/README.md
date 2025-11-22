# JODELL EPG Gripper Messages

This package contains message and service definitions for the JODELL EPG series electric parallel gripper.

## Messages

- `GripperState.msg`: Complete gripper state including position, force, and status

## Services

- `ActivateGripper.srv`: Activate gripper after power on
- `MoveGripper.srv`: Move gripper with position, speed, and force control
- `GetGripperState.srv`: Query current gripper state

## Hardware Specifications

- Communication: Modbus RTU via RS-485
- Default baudrate: 115200
- Slave ID: 1 (default)
- Control registers: 0x03E8-0x03EA
- Status registers: 0x07D0-0x07D2
