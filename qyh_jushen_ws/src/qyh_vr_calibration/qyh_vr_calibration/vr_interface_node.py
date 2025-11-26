#!/usr/bin/env python3
"""
VR Interface Node - Enhanced VR pose processing with filtering and coordinate transformation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation


class MovingAverageFilter:
    """Simple moving average filter for smoothing"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value):
        self.buffer.append(value)
        return np.mean(self.buffer, axis=0)
    
    def reset(self):
        self.buffer.clear()


class VRInterfaceNode(Node):
    def __init__(self):
        super().__init__('vr_interface_node')
        
        # Declare parameters
        self.declare_parameter('vr_frame_id', 'vr_world')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('position_smoothing_window', 5)
        self.declare_parameter('orientation_smoothing_window', 3)
        self.declare_parameter('position_deadzone', 0.002)  # 2mm
        self.declare_parameter('orientation_deadzone', 0.01)  # ~0.57 degrees
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('orientation_scale', 1.0)
        self.declare_parameter('control_enabled', True)
        
        # Get parameters
        self.vr_frame = self.get_parameter('vr_frame_id').value
        self.robot_frame = self.get_parameter('robot_frame_id').value
        self.pos_window = self.get_parameter('position_smoothing_window').value
        self.ori_window = self.get_parameter('orientation_smoothing_window').value
        self.pos_deadzone = self.get_parameter('position_deadzone').value
        self.ori_deadzone = self.get_parameter('orientation_deadzone').value
        self.pos_scale = self.get_parameter('position_scale').value
        self.ori_scale = self.get_parameter('orientation_scale').value
        self.control_enabled = self.get_parameter('control_enabled').value
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Filters for left hand
        self.left_pos_filter = MovingAverageFilter(self.pos_window)
        self.left_ori_filter = MovingAverageFilter(self.ori_window)
        self.left_prev_pose = None
        
        # Filters for right hand
        self.right_pos_filter = MovingAverageFilter(self.pos_window)
        self.right_ori_filter = MovingAverageFilter(self.ori_window)
        self.right_prev_pose = None
        
        # Subscribers - VR raw poses
        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/left_hand/pose',
            self.left_pose_callback,
            10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            '/vr/right_hand/pose',
            self.right_pose_callback,
            10
        )
        
        # Subscriber - VR buttons
        self.button_sub = self.create_subscription(
            Joy,
            '/vr/buttons',
            self.button_callback,
            10
        )
        
        # Subscriber - control enable/disable
        self.enable_sub = self.create_subscription(
            Bool,
            '/vr/control_enable',
            self.enable_callback,
            10
        )
        
        # Publishers - processed target poses for teleoperation controller
        self.left_target_pub = self.create_publisher(
            PoseStamped,
            '/vr/left_target_pose',
            10
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped,
            '/vr/right_target_pose',
            10
        )
        
        # Publisher - control status
        self.status_pub = self.create_publisher(
            Bool,
            '/vr/control_status',
            10
        )
        
        self.get_logger().info('VR Interface Node initialized')
        self.get_logger().info(f'  VR frame: {self.vr_frame}')
        self.get_logger().info(f'  Robot frame: {self.robot_frame}')
        self.get_logger().info(f'  Position smoothing: {self.pos_window}')
        self.get_logger().info(f'  Orientation smoothing: {self.ori_window}')
        self.get_logger().info(f'  Position deadzone: {self.pos_deadzone} m')
        self.get_logger().info(f'  Orientation deadzone: {self.ori_deadzone} rad')
        self.get_logger().info(f'  Control enabled: {self.control_enabled}')
    
    def left_pose_callback(self, msg):
        """Process left hand VR pose"""
        if not self.control_enabled:
            return
        
        processed_pose = self.process_pose(
            msg,
            self.left_pos_filter,
            self.left_ori_filter,
            self.left_prev_pose
        )
        
        if processed_pose:
            self.left_prev_pose = processed_pose
            self.left_target_pub.publish(processed_pose)
    
    def right_pose_callback(self, msg):
        """Process right hand VR pose"""
        if not self.control_enabled:
            return
        
        processed_pose = self.process_pose(
            msg,
            self.right_pos_filter,
            self.right_ori_filter,
            self.right_prev_pose
        )
        
        if processed_pose:
            self.right_prev_pose = processed_pose
            self.right_target_pub.publish(processed_pose)
    
    def process_pose(self, pose_msg, pos_filter, ori_filter, prev_pose):
        """
        Process VR pose: coordinate transform, filtering, deadzone, scaling
        """
        try:
            # Transform from VR frame to robot frame
            if pose_msg.header.frame_id != self.robot_frame:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    pose_msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_msg, transform
                )
            else:
                pose_transformed = pose_msg
            
            # Extract position and orientation
            pos = np.array([
                pose_transformed.pose.position.x,
                pose_transformed.pose.position.y,
                pose_transformed.pose.position.z
            ])
            
            quat = np.array([
                pose_transformed.pose.orientation.x,
                pose_transformed.pose.orientation.y,
                pose_transformed.pose.orientation.z,
                pose_transformed.pose.orientation.w
            ])
            
            # Apply smoothing filters
            pos_filtered = pos_filter.update(pos)
            quat_filtered = ori_filter.update(quat)
            
            # Normalize quaternion after filtering
            quat_filtered = quat_filtered / np.linalg.norm(quat_filtered)
            
            # Apply deadzone (compare with previous pose)
            if prev_pose is not None:
                prev_pos = np.array([
                    prev_pose.pose.position.x,
                    prev_pose.pose.position.y,
                    prev_pose.pose.position.z
                ])
                
                pos_diff = np.linalg.norm(pos_filtered - prev_pos)
                
                if pos_diff < self.pos_deadzone:
                    # Use previous position if within deadzone
                    pos_filtered = prev_pos
                
                # Orientation deadzone (angle difference)
                prev_quat = np.array([
                    prev_pose.pose.orientation.x,
                    prev_pose.pose.orientation.y,
                    prev_pose.pose.orientation.z,
                    prev_pose.pose.orientation.w
                ])
                
                # Compute angle between quaternions
                dot_product = np.abs(np.dot(quat_filtered, prev_quat))
                dot_product = np.clip(dot_product, -1.0, 1.0)
                angle_diff = 2 * np.arccos(dot_product)
                
                if angle_diff < self.ori_deadzone:
                    # Use previous orientation if within deadzone
                    quat_filtered = prev_quat
            
            # Apply scaling
            if self.pos_scale != 1.0 or self.ori_scale != 1.0:
                if prev_pose is not None:
                    # Scale relative to previous pose
                    prev_pos = np.array([
                        prev_pose.pose.position.x,
                        prev_pose.pose.position.y,
                        prev_pose.pose.position.z
                    ])
                    pos_delta = pos_filtered - prev_pos
                    pos_filtered = prev_pos + pos_delta * self.pos_scale
                    
                    # Orientation scaling (slerp)
                    if self.ori_scale != 1.0:
                        prev_quat = np.array([
                            prev_pose.pose.orientation.x,
                            prev_pose.pose.orientation.y,
                            prev_pose.pose.orientation.z,
                            prev_pose.pose.orientation.w
                        ])
                        # SLERP between previous and current
                        quat_filtered = self.slerp(prev_quat, quat_filtered, self.ori_scale)
            
            # Create output message
            output_msg = PoseStamped()
            output_msg.header.stamp = self.get_clock().now().to_msg()
            output_msg.header.frame_id = self.robot_frame
            
            output_msg.pose.position.x = float(pos_filtered[0])
            output_msg.pose.position.y = float(pos_filtered[1])
            output_msg.pose.position.z = float(pos_filtered[2])
            
            output_msg.pose.orientation.x = float(quat_filtered[0])
            output_msg.pose.orientation.y = float(quat_filtered[1])
            output_msg.pose.orientation.z = float(quat_filtered[2])
            output_msg.pose.orientation.w = float(quat_filtered[3])
            
            return output_msg
            
        except Exception as e:
            self.get_logger().warn(f'Failed to process pose: {e}', throttle_duration_sec=1.0)
            return None
    
    def slerp(self, q1, q2, t):
        """Spherical linear interpolation between two quaternions"""
        dot = np.dot(q1, q2)
        
        # Ensure shortest path
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # Clamp dot product
        dot = np.clip(dot, -1.0, 1.0)
        
        theta = np.arccos(dot)
        
        if theta < 1e-6:
            return q2
        
        sin_theta = np.sin(theta)
        w1 = np.sin((1.0 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2
    
    def button_callback(self, msg):
        """Process VR button inputs"""
        # Button mapping (customize based on your VR controller)
        # Example: Button 0 = Trigger, Button 1 = Grip
        
        if len(msg.buttons) > 0:
            # Button 0: Toggle control enable/disable
            if msg.buttons[0] == 1:
                self.control_enabled = not self.control_enabled
                self.get_logger().info(f'Control {"enabled" if self.control_enabled else "disabled"}')
                
                # Publish status
                status_msg = Bool()
                status_msg.data = self.control_enabled
                self.status_pub.publish(status_msg)
                
                # Reset filters when disabled
                if not self.control_enabled:
                    self.left_pos_filter.reset()
                    self.left_ori_filter.reset()
                    self.right_pos_filter.reset()
                    self.right_ori_filter.reset()
                    self.left_prev_pose = None
                    self.right_prev_pose = None
    
    def enable_callback(self, msg):
        """External control enable/disable"""
        self.control_enabled = msg.data
        self.get_logger().info(f'Control externally {"enabled" if self.control_enabled else "disabled"}')
        
        if not self.control_enabled:
            # Reset filters
            self.left_pos_filter.reset()
            self.left_ori_filter.reset()
            self.right_pos_filter.reset()
            self.right_ori_filter.reset()
            self.left_prev_pose = None
            self.right_prev_pose = None


def main(args=None):
    rclpy.init(args=args)
    node = VRInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
