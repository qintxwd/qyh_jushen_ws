#!/usr/bin/env python3
"""
JAKA Joint Name Adapter Node

Purpose:
- Adapts joint names from JAKA real robot driver to URDF convention
- Subscribes to /joint_states_raw (from jaka_control_node)
- Publishes to /joint_states (for robot_state_publisher)

Joint name mapping:
  JAKA driver format → URDF format
  left_joint1  → l-j1
  left_joint2  → l-j2
  ...
  left_joint7  → l-j7
  right_joint1 → r-j1
  right_joint2 → r-j2
  ...
  right_joint7 → r-j7

Alternative mapping (if JAKA already uses short format):
  l-j1 → l-j1 (pass-through)
  r-j1 → r-j1 (pass-through)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointNameAdapter(Node):
    def __init__(self):
        super().__init__('qyh_jaka_joint_adapter')
        
        # Joint name mapping dictionary
        # Format 1: left_jointN / right_jointN → l-jN / r-jN
        self.name_map_format1 = {
            'left_joint1': 'l-j1',
            'left_joint2': 'l-j2',
            'left_joint3': 'l-j3',
            'left_joint4': 'l-j4',
            'left_joint5': 'l-j5',
            'left_joint6': 'l-j6',
            'left_joint7': 'l-j7',
            'right_joint1': 'r-j1',
            'right_joint2': 'r-j2',
            'right_joint3': 'r-j3',
            'right_joint4': 'r-j4',
            'right_joint5': 'r-j5',
            'right_joint6': 'r-j6',
            'right_joint7': 'r-j7',
        }
        
        # Format 2: l1, l2, ... → l-j1, l-j2, ... (if needed)
        self.name_map_format2 = {
            'l1': 'l-j1', 'l2': 'l-j2', 'l3': 'l-j3', 'l4': 'l-j4',
            'l5': 'l-j5', 'l6': 'l-j6', 'l7': 'l-j7',
            'r1': 'r-j1', 'r2': 'r-j2', 'r3': 'r-j3', 'r4': 'r-j4',
            'r5': 'r-j5', 'r6': 'r-j6', 'r7': 'r-j7',
        }
        
        # Detected mapping format (auto-detect on first message)
        self.detected_format = None
        
        # Subscribe to raw joint states from jaka_control_node
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.joint_states_callback,
            10)
        
        # Publish adapted joint states for robot_state_publisher
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        self.get_logger().info('JAKA Joint Adapter Node started')
        self.get_logger().info('  Subscribing to: /joint_states_raw')
        self.get_logger().info('  Publishing to: /joint_states')
        self.get_logger().info('  Will auto-detect joint name format on first message')
    
    def auto_detect_format(self, joint_names):
        """Auto-detect which format the input uses."""
        if not joint_names:
            return None
        
        first_name = joint_names[0]
        
        # Check if already in URDF format (l-j1 / r-j1)
        if first_name.startswith('l-j') or first_name.startswith('r-j'):
            self.get_logger().info('Detected format: URDF native (l-jN / r-jN) - pass-through mode')
            return 'urdf_native'
        
        # Check if in left_jointN / right_jointN format
        if first_name.startswith('left_joint') or first_name.startswith('right_joint'):
            self.get_logger().info('Detected format: left_jointN / right_jointN → l-jN / r-jN')
            return 'format1'
        
        # Check if in l1/r1 format
        if first_name in ['l1', 'r1'] or (len(first_name) == 2 and first_name[0] in 'lr'):
            self.get_logger().info('Detected format: lN / rN → l-jN / r-jN')
            return 'format2'
        
        self.get_logger().warn(f'Unknown joint name format: {first_name}')
        return None
    
    def convert_joint_names(self, joint_names):
        """Convert joint names based on detected format."""
        if self.detected_format == 'urdf_native':
            # No conversion needed
            return joint_names
        elif self.detected_format == 'format1':
            return [self.name_map_format1.get(name, name) for name in joint_names]
        elif self.detected_format == 'format2':
            return [self.name_map_format2.get(name, name) for name in joint_names]
        else:
            # Fallback: try both mappings
            result = []
            for name in joint_names:
                if name in self.name_map_format1:
                    result.append(self.name_map_format1[name])
                elif name in self.name_map_format2:
                    result.append(self.name_map_format2[name])
                else:
                    result.append(name)  # Pass through unknown names
            return result
    
    def joint_states_callback(self, msg):
        """Convert joint names and republish."""
        # Auto-detect format on first message
        if self.detected_format is None:
            self.detected_format = self.auto_detect_format(msg.name)
            if self.detected_format is None:
                self.get_logger().error('Failed to detect joint name format, using fallback')
        
        # Create new message with converted names
        adapted_msg = JointState()
        adapted_msg.header = msg.header
        
        # Convert joint names
        adapted_msg.name = self.convert_joint_names(msg.name)
        
        # Copy position, velocity, effort as-is
        adapted_msg.position = msg.position
        adapted_msg.velocity = msg.velocity
        adapted_msg.effort = msg.effort
        
        # Publish adapted message
        self.publisher.publish(adapted_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointNameAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
