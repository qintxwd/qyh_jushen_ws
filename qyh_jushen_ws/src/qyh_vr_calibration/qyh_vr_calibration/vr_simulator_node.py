#!/usr/bin/env python3
"""
VR Simulator Node - Publishes simulated VR hand poses for testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
import math


class VRSimulatorNode(Node):
    def __init__(self):
        super().__init__('vr_simulator_node')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 90.0)  # 90Hz typical VR rate
        self.declare_parameter('motion_type', 'circle')  # circle, figure8, static
        self.declare_parameter('motion_amplitude', 0.1)  # meters
        self.declare_parameter('motion_frequency', 0.2)  # Hz
        
        # Get parameters
        self.rate = self.get_parameter('publish_rate').value
        self.motion_type = self.get_parameter('motion_type').value
        self.amplitude = self.get_parameter('motion_amplitude').value
        self.frequency = self.get_parameter('motion_frequency').value
        
        # Publishers
        self.left_pose_pub = self.create_publisher(
            PoseStamped,
            '/vr/left_hand/pose',
            10
        )
        self.right_pose_pub = self.create_publisher(
            PoseStamped,
            '/vr/right_hand/pose',
            10
        )
        self.button_pub = self.create_publisher(
            Joy,
            '/vr/buttons',
            10
        )
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        
        # Initial positions (robot workspace)
        self.left_initial_pos = np.array([0.3, 0.3, 0.3])  # x, y, z
        self.right_initial_pos = np.array([0.3, -0.3, 0.3])
        
        # Time counter
        self.time = 0.0
        
        self.get_logger().info('VR Simulator Node initialized')
        self.get_logger().info(f'  Motion type: {self.motion_type}')
        self.get_logger().info(f'  Amplitude: {self.amplitude} m')
        self.get_logger().info(f'  Frequency: {self.frequency} Hz')
        self.get_logger().info(f'  Publish rate: {self.rate} Hz')
    
    def timer_callback(self):
        """Generate and publish simulated VR poses"""
        self.time += 1.0 / self.rate
        
        # Generate motion offset based on motion type
        offset = self.generate_motion_offset(self.time)
        
        # Left hand pose
        left_pose = self.create_pose_msg(
            self.left_initial_pos + offset,
            [0.0, 0.0, 0.0, 1.0]  # No rotation for now
        )
        self.left_pose_pub.publish(left_pose)
        
        # Right hand pose (mirror motion)
        right_offset = offset.copy()
        right_offset[1] = -right_offset[1]  # Mirror Y axis
        right_pose = self.create_pose_msg(
            self.right_initial_pos + right_offset,
            [0.0, 0.0, 0.0, 1.0]
        )
        self.right_pose_pub.publish(right_pose)
        
        # Publish button state (all released)
        button_msg = Joy()
        button_msg.header.stamp = self.get_clock().now().to_msg()
        button_msg.buttons = [0, 0, 0, 0]  # 4 buttons, all released
        button_msg.axes = [0.0, 0.0, 0.0, 0.0]
        self.button_pub.publish(button_msg)
    
    def generate_motion_offset(self, t):
        """Generate motion offset based on motion type"""
        omega = 2 * math.pi * self.frequency
        
        if self.motion_type == 'circle':
            # Circular motion in XY plane
            x = self.amplitude * math.cos(omega * t)
            y = self.amplitude * math.sin(omega * t)
            z = 0.0
        
        elif self.motion_type == 'figure8':
            # Figure-8 motion
            x = self.amplitude * math.sin(omega * t)
            y = self.amplitude * math.sin(2 * omega * t) / 2
            z = 0.0
        
        elif self.motion_type == 'vertical':
            # Vertical sinusoidal motion
            x = 0.0
            y = 0.0
            z = self.amplitude * math.sin(omega * t)
        
        elif self.motion_type == 'static':
            # No motion
            x = 0.0
            y = 0.0
            z = 0.0
        
        else:
            x = y = z = 0.0
        
        return np.array([x, y, z])
    
    def create_pose_msg(self, position, orientation):
        """Create PoseStamped message"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Directly in robot frame for simplicity
        
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        
        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VRSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
