# Enhanced VR Calibration System - User Guide

## Overview

This enhanced VR calibration system provides **posture-based user calibration** with **hybrid workspace mapping** for precise and smooth teleoperation of dual JAKA Zu7 robotic arms.

**Note**: This is now the standard VR calibration system. The "enhanced" label has been dropped - all nodes use standard names.

### Key Features

✅ **Posture-based Calibration**: Uses 4 natural human poses (~30 seconds)  
✅ **Anisotropic Scaling**: Different scales for vertical/forward/lateral directions  
✅ **Hybrid Mapping**: Blends direct (precise) and incremental (smooth) mapping  
✅ **User Profiles**: Persistent calibration storage per user  
✅ **Workspace Limits**: Safety bounds for robot workspace  
✅ **Multi-layer Filtering**: VR + teleoperation + bridge smoothing  

---

## Architecture

### Components

1. **vr_calibration_node**: Service provider for user calibration
   - Loads fixed robot reference poses from `robot.yaml` at startup
   - Computes scaling, rotation, and offset parameters from 4 user VR postures
   - Stores user calibration to `~/.qyh_robot/users/{username}.yaml`

2. **vr_interface_node**: VR pose processor with hybrid mapping
   - Loads user calibration parameters
   - Applies workspace mapping (direct/incremental/hybrid modes)
   - Filters and publishes target poses

3. **calibration_tool**: Interactive CLI for user calibration
   - Guides user through 4 postures
   - Captures only VR poses (robot poses from `robot.yaml`)
   - Calls calibration service

### Mapping Modes

| Mode | Precision | Smoothness | Use Case |
|------|-----------|------------|----------|
| **Direct** | ⭐⭐⭐ | ⭐ | High-precision tasks, good calibration |
| **Incremental** | ⭐ | ⭐⭐⭐ | Smooth motion, drift-free |
| **Hybrid** (recommended) | ⭐⭐⭐ | ⭐⭐⭐ | Best of both worlds |

---

## Quick Start

### 1. Compile the System

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_vr_calibration_msgs qyh_vr_calibration
source install/setup.bash
```

### 2. Launch Enhanced VR Interface

```bash
# Launch with default settings (hybrid mapping)
ros2 launch qyh_vr_calibration vr_interface.launch.py

# Or specify username to auto-load calibration
ros2 launch qyh_vr_calibration vr_interface.launch.py username:=john_doe

# Or specify mapping mode
ros2 launch qyh_vr_calibration vr_interface.launch.py mapping_mode:=direct
```

### 3. Calibrate a New User

**Important**: Robot reference poses must be configured first in `robot.yaml`!

```bash
# Run calibration tool (interactive)
ros2 run qyh_vr_calibration calibration_tool john_doe
```

Follow the on-screen prompts to perform 4 postures:
1. **Arms down naturally** (手臂自然下垂)
2. **Arms up vertically** (手臂垂直向上)
3. **Arms forward horizontal** (手臂向前伸直)
4. **Arms side horizontal** (双臂侧平举)

**Note**: Only VR controller poses are captured. Robot reference poses are loaded from:
- `~/qyh_jushen_ws/persistent/vr_calibration_robot/robot.yaml`

Calibration takes ~30 seconds total.

---

## Four Calibration Postures

### Posture 1: Arms Down Naturally (手臂自然下垂)
- Stand naturally with arms relaxed at sides
- Controllers pointing down
- **Purpose**: Establish vertical axis baseline

### Posture 2: Arms Up Vertically (手臂垂直向上)
- Raise both arms straight up overhead
- Controllers pointing up
- **Purpose**: Measure vertical workspace extent

### Posture 3: Arms Forward Horizontal (手臂向前伸直)
- Extend both arms forward at shoulder height
- Controllers pointing forward
- **Purpose**: Measure forward reach and establish forward axis

### Posture 4: Arms Side Horizontal (双臂侧平举)
- Extend arms to sides at shoulder height
- Controllers pointing outward
- **Purpose**: Measure lateral workspace span

---

## Calibration Data Format

### Robot Reference Poses (Fixed)

Stored in `~/qyh_jushen_ws/persistent/vr_calibration_robot/robot.yaml`:

```yaml
username: ROBOT
samples:
  - sample_index: 0  # Arms down
    left_hand_pose: {position: {x: 0.4, y: 0.3, z: 0.1}, orientation: {...}}
    right_hand_pose: {position: {x: 0.4, y: -0.3, z: 0.1}, orientation: {...}}
  - sample_index: 1  # Arms up
    left_hand_pose: {position: {x: 0.4, y: 0.3, z: 0.7}, orientation: {...}}
    right_hand_pose: {position: {x: 0.4, y: -0.3, z: 0.7}, orientation: {...}}
  - sample_index: 2  # Arms forward
    left_hand_pose: {position: {x: 0.6, y: 0.2, z: 0.4}, orientation: {...}}
    right_hand_pose: {position: {x: 0.6, y: -0.2, z: 0.4}, orientation: {...}}
  - sample_index: 3  # Arms side
    left_hand_pose: {position: {x: 0.4, y: 0.5, z: 0.4}, orientation: {...}}
    right_hand_pose: {position: {x: 0.4, y: -0.5, z: 0.4}, orientation: {...}}
```

**Important**: Configure these poses to match your robot's actual workspace!

### User Calibration (Per User)

User calibration is stored in `~/.qyh_robot/users/{username}.yaml`:

```yaml
username: john_doe
calibration:
  scaling: [0.856, 0.921, 1.034]  # [vertical, forward, lateral]
  rotation_matrix: [0.998, -0.052, 0.031, ...]  # 3x3 matrix (9 elements)
  vr_offset: [0.0, 0.0, 1.2]  # VR workspace center
  robot_offset: [0.5, 0.0, 0.3]  # Robot workspace center
postures:
  vr: [...]  # 4 VR poses captured during calibration
  robot: [...]  # 4 robot poses captured during calibration
```

---

## Configuration Parameters

Edit `config/vr_interface_params.yaml`:

### Mapping Parameters

```yaml
mapping_mode: 'hybrid'  # 'direct', 'incremental', or 'hybrid'
hybrid_blend_velocity: 0.05  # m/s - velocity threshold
hybrid_min_weight: 0.2  # Min direct weight (max incremental)
hybrid_max_weight: 0.8  # Max direct weight (min incremental)
```

### Workspace Limits

```yaml
workspace_limit_x_min: 0.2  # meters
workspace_limit_x_max: 0.8
workspace_limit_y_min: -0.5
workspace_limit_y_max: 0.5
workspace_limit_z_min: 0.0
workspace_limit_z_max: 0.8
```

### Velocity Limits

```yaml
max_linear_velocity: 0.5  # m/s
max_angular_velocity: 1.0  # rad/s
```

---

## Services

### Calibrate User
```bash
# Only VR poses needed - robot poses loaded from robot.yaml
ros2 service call /vr_calibration/calibrate_user qyh_vr_calibration_msgs/srv/CalibrateUser \
  "{username: 'john_doe', vr_poses: [...]}"
```

### Load User Calibration
```bash
ros2 service call /vr_calibration/load_user qyh_vr_calibration_msgs/srv/LoadUserCalibration \
  "{username: 'john_doe'}"
```

### List Users
```bash
ros2 service call /vr_calibration/list_users qyh_vr_calibration_msgs/srv/ListUsers
```

### Delete User
```bash
ros2 service call /vr_calibration/delete_user qyh_vr_calibration_msgs/srv/DeleteUser \
  "{username: 'john_doe'}"
```

---

## Topics

### Published by VR Interface
- `/vr/left_target_pose` (PoseStamped): Processed left arm target
- `/vr/right_target_pose` (PoseStamped): Processed right arm target
- `/vr/control_status` (Bool): Control enabled status

### Subscribed by VR Interface
- `/vr/left_hand/pose` (PoseStamped): Raw left VR controller pose
- `/vr/right_hand/pose` (PoseStamped): Raw right VR controller pose
- `/vr/buttons` (Joy): VR button inputs
- `/vr/control_enable` (Bool): External enable/disable
- `/vr/load_user` (String): Load user calibration by name

---

## Hybrid Mapping Algorithm

### Position Mapping

```python
# Compute velocity magnitude
velocity = (current_vr_pos - prev_vr_pos) / dt
vel_mag = norm(velocity)

# Adaptive blending weight
if vel_mag < threshold:
    direct_weight = 0.8  # High precision
else:
    direct_weight = 0.2 + 0.6 * (1 - vel_mag/threshold)  # More incremental

# Blend modes
robot_pos = direct_weight * direct_pos + (1 - direct_weight) * incremental_pos
```

### Direct Mapping
1. Center VR position: `vr_centered = vr_pos - vr_offset`
2. Apply rotation: `vr_rotated = R @ vr_centered`
3. Apply anisotropic scaling: `scaled = vr_rotated * [sv, sf, sl]`
4. Translate to robot: `robot_pos = scaled + robot_offset`

### Incremental Mapping
1. Compute VR velocity: `vr_vel = (vr_pos - prev_vr_pos) / dt`
2. Transform velocity: `robot_vel = R @ (vr_vel * scaling)`
3. Limit velocity: `robot_vel = clip(robot_vel, max_vel)`
4. Integrate: `robot_pos = prev_robot_pos + robot_vel * dt`

---

## Troubleshooting

### Calibration fails
- **Check VR connection**: Ensure `/vr/left_hand/pose` and `/vr/right_hand/pose` are publishing
- **Check calibration service**: `ros2 service list | grep calibrate`

### Robot position jumps
- Lower `hybrid_max_weight` to increase smoothing
- Increase `position_smoothing_window`
- Check workspace limits

### Poor precision
- Increase `hybrid_max_weight` for more direct mapping
- Re-calibrate user with careful posture positioning
- Verify robot pose capture during calibration

### Drift in incremental mode
- Hybrid mode prevents drift by blending direct mapping
- Check `hybrid_min_weight` is not too low

---

## Integration with Full System

To use enhanced VR calibration in full teleoperation pipeline:

```bash
# Launch full system with enhanced VR
ros2 launch qyh_teleoperation_bringup full_system.launch.py username:=john_doe
```

The pipeline:
```
VR Controllers → vr_interface_node → teleoperation_controller → 
smooth_servo_bridge → jaka_bridge_node → Real Robots
```

---

## Best Practices

1. **Calibrate Each User**: Different users have different arm lengths and workspace sizes
2. **Use Hybrid Mode**: Best balance of precision and smoothness (default)
3. **Adjust Workspace Limits**: Match your robot's actual reachable workspace
4. **Re-calibrate Periodically**: After VR system changes or robot repositioning
5. **Start with Default Parameters**: Tune only if needed for your specific task

---

## Comparison with Original System

| Feature | Original | Enhanced |
|---------|----------|----------|
| Calibration | Manual 4-point workspace | 4 natural postures |
| Scaling | Isotropic (uniform) | Anisotropic (per-axis) |
| Mapping | Fixed transform | Hybrid (adaptive) |
| User profiles | Single profile | Multi-user with persistence |
| Calibration time | ~2 minutes | ~30 seconds |
| Precision | Medium | High |
| Smoothness | Medium | High |

---

## References

- Original design: `VR_TELEOPERATION_ARCHITECTURE.md`
- Full integration: `TELEOPERATION_INTEGRATION_GUIDE.md`
- Deployment: `JETSON_DEPLOYMENT_CHECKLIST.md`

---

**Questions?** Check the main project documentation or open an issue.
