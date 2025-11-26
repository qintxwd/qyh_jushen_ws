#!/usr/bin/env python3
"""
Enhanced VR Interface Node - Hybrid mapping with user calibration

Supports three mapping modes:
1. Direct mapping: VR pose -> Robot pose (high precision)
2. Incremental mapping: VR velocity -> Robot velocity (smooth, no drift accumulation)
3. Hybrid mapping: Blend direct and incremental based on velocity (recommended)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from qyh_vr_calibration_msgs.srv import LoadUserCalibration
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation


class MovingAverageFilter:
    """Simple moving average filter"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value):
        self.buffer.append(value)
        return np.mean(self.buffer, axis=0)
    
    def reset(self):
        self.buffer.clear()


class VRInterfaceNodeEnhanced(Node):
    def __init__(self):
        super().__init__('vr_interface_node_enhanced')
        
        # Declare parameters
        self._declare_parameters()
        self._load_parameters()
        
        # User calibration data
        self.user_loaded = False
        self.scaling = np.array([1.0, 1.0, 1.0])  # [vertical, forward, lateral]
        self.rotation_matrix = np.eye(3)
        self.vr_offset = np.zeros(3)
        self.robot_offset = np.zeros(3)
        
        # State tracking for incremental mapping
        self.left_prev_vr_pos = None
        self.left_prev_robot_pos = None
        self.left_prev_vr_ori = None
        self.left_prev_robot_ori = None
        
        self.right_prev_vr_pos = None
        self.right_prev_robot_pos = None
        self.right_prev_vr_ori = None
        self.right_prev_robot_ori = None
        
        # Filters
        self.left_pos_filter = MovingAverageFilter(self.pos_window)
        self.left_ori_filter = MovingAverageFilter(self.ori_window)
        
        self.right_pos_filter = MovingAverageFilter(self.pos_window)
        self.right_ori_filter = MovingAverageFilter(self.ori_window)
        
        # Subscribers
        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/vr/left_hand/pose', self.left_pose_callback, 10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/vr/right_hand/pose', self.right_pose_callback, 10
        )
        self.button_sub = self.create_subscription(
            Joy, '/vr/buttons', self.button_callback, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, '/vr/control_enable', self.enable_callback, 10
        )
        self.user_sub = self.create_subscription(
            String, '/vr/load_user', self.user_callback, 10
        )
        
        # Publishers
        self.left_target_pub = self.create_publisher(
            PoseStamped, '/vr/left_target_pose', 10
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped, '/vr/right_target_pose', 10
        )
        self.status_pub = self.create_publisher(
            Bool, '/vr/control_status', 10
        )
        
        # Service client for loading user calibration
        self.load_user_client = self.create_client(
            LoadUserCalibration,
            '/vr_calibration/load_user'
        )
        
        self.get_logger().info('Enhanced VR Interface Node initialized')
        self.get_logger().info(f'  Mapping mode: {self.mapping_mode}')
        self.get_logger().info(f'  Position smoothing: {self.pos_window}')
        self.get_logger().info(f'  Orientation smoothing: {self.ori_window}')
        self.get_logger().info(f'  Control enabled: {self.control_enabled}')
        
        # Auto-load default user if specified
        if self.default_user:
            self.get_logger().info(f'  Auto-loading user: {self.default_user}')
            self._load_user_calibration(self.default_user)
    
    def _declare_parameters(self):
        """Declare all ROS parameters"""
        # Smoothing
        self.declare_parameter('position_smoothing_window', 5)
        self.declare_parameter('orientation_smoothing_window', 3)
        self.declare_parameter('position_deadzone', 0.002)  # 2mm
        self.declare_parameter('orientation_deadzone', 0.01)  # 0.57 degrees
        
        # Mapping mode
        self.declare_parameter('mapping_mode', 'hybrid')  # 'direct', 'incremental', 'hybrid'
        self.declare_parameter('hybrid_blend_velocity', 0.05)  # m/s, velocity threshold for blending
        self.declare_parameter('hybrid_min_weight', 0.2)  # Minimum direct weight in hybrid mode
        self.declare_parameter('hybrid_max_weight', 0.8)  # Maximum direct weight in hybrid mode
        
        # Workspace limits (robot workspace in meters)
        self.declare_parameter('workspace_limit_x_min', 0.2)
        self.declare_parameter('workspace_limit_x_max', 0.8)
        self.declare_parameter('workspace_limit_y_min', -0.5)
        self.declare_parameter('workspace_limit_y_max', 0.5)
        self.declare_parameter('workspace_limit_z_min', 0.0)
        self.declare_parameter('workspace_limit_z_max', 0.8)
        
        # Velocity limiting for incremental mode
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        
        # Control state
        self.declare_parameter('control_enabled', True)
        self.declare_parameter('default_user', '')  # Auto-load user on startup
    
    def _load_parameters(self):
        """Load all ROS parameters"""
        self.pos_window = self.get_parameter('position_smoothing_window').value
        self.ori_window = self.get_parameter('orientation_smoothing_window').value
        self.pos_deadzone = self.get_parameter('position_deadzone').value
        self.ori_deadzone = self.get_parameter('orientation_deadzone').value
        
        self.mapping_mode = self.get_parameter('mapping_mode').value
        self.hybrid_blend_vel = self.get_parameter('hybrid_blend_velocity').value
        self.hybrid_min_weight = self.get_parameter('hybrid_min_weight').value
        self.hybrid_max_weight = self.get_parameter('hybrid_max_weight').value
        
        self.ws_x_min = self.get_parameter('workspace_limit_x_min').value
        self.ws_x_max = self.get_parameter('workspace_limit_x_max').value
        self.ws_y_min = self.get_parameter('workspace_limit_y_min').value
        self.ws_y_max = self.get_parameter('workspace_limit_y_max').value
        self.ws_z_min = self.get_parameter('workspace_limit_z_min').value
        self.ws_z_max = self.get_parameter('workspace_limit_z_max').value
        
        self.max_lin_vel = self.get_parameter('max_linear_velocity').value
        self.max_ang_vel = self.get_parameter('max_angular_velocity').value
        
        self.control_enabled = self.get_parameter('control_enabled').value
        self.default_user = self.get_parameter('default_user').value
        
        self.last_time = self.get_clock().now()
    
    def _load_user_calibration(self, username):
        """Load user calibration asynchronously"""
        if not self.load_user_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Calibration service not available')
            return
        
        request = LoadUserCalibration.Request()
        request.username = username
        
        future = self.load_user_client.call_async(request)
        future.add_done_callback(lambda f: self._load_user_callback(f, username))
    
    def _load_user_callback(self, future, username):
        """Handle user calibration load response"""
        try:
            response = future.result()
            if response.success and response.found:
                self.scaling = np.array(response.scaling)
                self.rotation_matrix = np.array(response.rotation_matrix).reshape(3, 3)
                self.vr_offset = np.array(response.vr_offset)
                self.robot_offset = np.array(response.robot_offset)
                self.user_loaded = True
                
                self.get_logger().info(f'Loaded calibration for user: {username}')
                self.get_logger().info(f'  Scaling: {self.scaling}')
                self.get_logger().info(f'  VR offset: {self.vr_offset}')
                self.get_logger().info(f'  Robot offset: {self.robot_offset}')
            else:
                self.get_logger().warn(f'Failed to load calibration: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error loading calibration: {e}')
    
    def user_callback(self, msg):
        """Load user calibration from topic"""
        username = msg.data
        self.get_logger().info(f'Loading user: {username}')
        self._load_user_calibration(username)
    
    def left_pose_callback(self, msg):
        """Process left hand VR pose"""
        if not self.control_enabled:
            return
        
        processed_pose = self.process_pose(
            msg,
            self.left_pos_filter,
            self.left_ori_filter,
            self.left_prev_vr_pos,
            self.left_prev_robot_pos,
            self.left_prev_vr_ori,
            self.left_prev_robot_ori,
            'left'
        )
        
        if processed_pose:
            # Update previous states
            pos = np.array([
                processed_pose.pose.position.x,
                processed_pose.pose.position.y,
                processed_pose.pose.position.z
            ])
            quat = np.array([
                processed_pose.pose.orientation.x,
                processed_pose.pose.orientation.y,
                processed_pose.pose.orientation.z,
                processed_pose.pose.orientation.w
            ])
            
            # Store VR pose (before mapping)
            vr_pos, vr_quat = self._extract_pose_arrays(msg)
            self.left_prev_vr_pos = vr_pos
            self.left_prev_vr_ori = vr_quat
            
            # Store robot pose (after mapping)
            self.left_prev_robot_pos = pos
            self.left_prev_robot_ori = quat
            
            self.left_target_pub.publish(processed_pose)
    
    def right_pose_callback(self, msg):
        """Process right hand VR pose"""
        if not self.control_enabled:
            return
        
        processed_pose = self.process_pose(
            msg,
            self.right_pos_filter,
            self.right_ori_filter,
            self.right_prev_vr_pos,
            self.right_prev_robot_pos,
            self.right_prev_vr_ori,
            self.right_prev_robot_ori,
            'right'
        )
        
        if processed_pose:
            # Update previous states
            pos = np.array([
                processed_pose.pose.position.x,
                processed_pose.pose.position.y,
                processed_pose.pose.position.z
            ])
            quat = np.array([
                processed_pose.pose.orientation.x,
                processed_pose.pose.orientation.y,
                processed_pose.pose.orientation.z,
                processed_pose.pose.orientation.w
            ])
            
            # Store VR pose
            vr_pos, vr_quat = self._extract_pose_arrays(msg)
            self.right_prev_vr_pos = vr_pos
            self.right_prev_vr_ori = vr_quat
            
            # Store robot pose
            self.right_prev_robot_pos = pos
            self.right_prev_robot_ori = quat
            
            self.right_target_pub.publish(processed_pose)
    
    def _extract_pose_arrays(self, pose_msg):
        """Extract position and orientation as numpy arrays"""
        pos = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z
        ])
        quat = np.array([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        ])
        return pos, quat
    
    def process_pose(self, pose_msg, pos_filter, ori_filter,
                     prev_vr_pos, prev_robot_pos, prev_vr_ori, prev_robot_ori,
                     hand='left'):
        """
        Process VR pose with hybrid mapping
        """
        try:
            # Extract VR pose
            vr_pos, vr_quat = self._extract_pose_arrays(pose_msg)
            
            # Apply smoothing filter
            vr_pos_filtered = pos_filter.update(vr_pos)
            vr_quat_filtered = ori_filter.update(vr_quat)
            vr_quat_filtered = vr_quat_filtered / np.linalg.norm(vr_quat_filtered)
            
            # Apply deadzone
            if prev_vr_pos is not None:
                pos_diff = np.linalg.norm(vr_pos_filtered - prev_vr_pos)
                if pos_diff < self.pos_deadzone:
                    vr_pos_filtered = prev_vr_pos
                
                dot_product = np.abs(np.dot(vr_quat_filtered, prev_vr_ori))
                dot_product = np.clip(dot_product, -1.0, 1.0)
                angle_diff = 2 * np.arccos(dot_product)
                if angle_diff < self.ori_deadzone:
                    vr_quat_filtered = prev_vr_ori
            
            # Apply workspace mapping based on mode
            if self.mapping_mode == 'direct':
                robot_pos, robot_quat = self._direct_mapping(vr_pos_filtered, vr_quat_filtered)
            
            elif self.mapping_mode == 'incremental':
                robot_pos, robot_quat = self._incremental_mapping(
                    vr_pos_filtered, vr_quat_filtered,
                    prev_vr_pos, prev_robot_pos,
                    prev_vr_ori, prev_robot_ori
                )
            
            elif self.mapping_mode == 'hybrid':
                robot_pos, robot_quat = self._hybrid_mapping(
                    vr_pos_filtered, vr_quat_filtered,
                    prev_vr_pos, prev_robot_pos,
                    prev_vr_ori, prev_robot_ori
                )
            
            else:
                self.get_logger().warn(f'Unknown mapping mode: {self.mapping_mode}', 
                                      throttle_duration_sec=5.0)
                robot_pos, robot_quat = self._direct_mapping(vr_pos_filtered, vr_quat_filtered)
            
            # Apply workspace limits
            robot_pos = self._apply_workspace_limits(robot_pos)
            
            # Create output message
            output_msg = PoseStamped()
            output_msg.header.stamp = self.get_clock().now().to_msg()
            output_msg.header.frame_id = 'base_link'
            
            output_msg.pose.position.x = float(robot_pos[0])
            output_msg.pose.position.y = float(robot_pos[1])
            output_msg.pose.position.z = float(robot_pos[2])
            
            output_msg.pose.orientation.x = float(robot_quat[0])
            output_msg.pose.orientation.y = float(robot_quat[1])
            output_msg.pose.orientation.z = float(robot_quat[2])
            output_msg.pose.orientation.w = float(robot_quat[3])
            
            return output_msg
            
        except Exception as e:
            self.get_logger().warn(f'Failed to process pose: {e}', throttle_duration_sec=1.0)
            return None
    
    def _direct_mapping(self, vr_pos, vr_quat):
        """
        Direct mapping: VR pose -> Robot pose
        High precision, but sensitive to calibration errors
        """
        if not self.user_loaded:
            # No calibration loaded, pass through
            return vr_pos.copy(), vr_quat.copy()
        
        # Center VR position
        vr_centered = vr_pos - self.vr_offset
        
        # Apply rotation
        vr_rotated = self.rotation_matrix @ vr_centered
        
        # Apply anisotropic scaling
        # Decompose into vertical, forward, lateral components
        # This is simplified - assumes axes are aligned after rotation
        scaled = vr_rotated * self.scaling
        
        # Translate to robot workspace
        robot_pos = scaled + self.robot_offset
        
        # Orientation: apply same rotation
        vr_rot_scipy = Rotation.from_quat(vr_quat)
        calib_rot = Rotation.from_matrix(self.rotation_matrix)
        robot_rot = calib_rot * vr_rot_scipy
        robot_quat = robot_rot.as_quat()
        
        return robot_pos, robot_quat
    
    def _incremental_mapping(self, vr_pos, vr_quat, 
                            prev_vr_pos, prev_robot_pos,
                            prev_vr_ori, prev_robot_ori):
        """
        Incremental mapping: VR velocity -> Robot velocity
        Smooth, no drift, but less precise absolute positioning
        """
        if prev_vr_pos is None or prev_robot_pos is None:
            # First frame, use direct mapping
            return self._direct_mapping(vr_pos, vr_quat)
        
        # Compute VR velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt < 1e-6:
            return prev_robot_pos.copy(), prev_robot_ori.copy()
        
        vr_velocity = (vr_pos - prev_vr_pos) / dt
        
        # Apply scaling and rotation to velocity
        if self.user_loaded:
            vr_vel_rotated = self.rotation_matrix @ vr_velocity
            scaled_velocity = vr_vel_rotated * self.scaling
        else:
            scaled_velocity = vr_velocity
        
        # Limit velocity
        vel_mag = np.linalg.norm(scaled_velocity)
        if vel_mag > self.max_lin_vel:
            scaled_velocity = scaled_velocity / vel_mag * self.max_lin_vel
        
        # Update robot position incrementally
        robot_pos = prev_robot_pos + scaled_velocity * dt
        
        # Orientation: compute angular velocity and integrate
        # Simplified: use SLERP for smooth interpolation
        vr_rot_prev = Rotation.from_quat(prev_vr_ori)
        vr_rot_curr = Rotation.from_quat(vr_quat)
        vr_rot_delta = vr_rot_curr * vr_rot_prev.inv()
        
        robot_rot_prev = Rotation.from_quat(prev_robot_ori)
        robot_rot_curr = robot_rot_prev * vr_rot_delta
        robot_quat = robot_rot_curr.as_quat()
        
        return robot_pos, robot_quat
    
    def _hybrid_mapping(self, vr_pos, vr_quat,
                       prev_vr_pos, prev_robot_pos,
                       prev_vr_ori, prev_robot_ori):
        """
        Hybrid mapping: Blend direct and incremental based on velocity
        Low velocity: More direct (precise)
        High velocity: More incremental (smooth)
        """
        if prev_vr_pos is None or prev_robot_pos is None:
            return self._direct_mapping(vr_pos, vr_quat)
        
        # Compute VR velocity
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 1e-6:
            return prev_robot_pos.copy(), prev_robot_ori.copy()
        
        vr_velocity = (vr_pos - prev_vr_pos) / dt
        vel_mag = np.linalg.norm(vr_velocity)
        
        # Compute blend weight (direct weight)
        # Low velocity -> high weight (more direct)
        # High velocity -> low weight (more incremental)
        if vel_mag < self.hybrid_blend_vel:
            direct_weight = self.hybrid_max_weight
        else:
            # Linear interpolation
            ratio = min((vel_mag - self.hybrid_blend_vel) / self.hybrid_blend_vel, 1.0)
            direct_weight = self.hybrid_max_weight - ratio * (self.hybrid_max_weight - self.hybrid_min_weight)
        
        incremental_weight = 1.0 - direct_weight
        
        # Compute both mappings
        direct_pos, direct_quat = self._direct_mapping(vr_pos, vr_quat)
        incr_pos, incr_quat = self._incremental_mapping(
            vr_pos, vr_quat, prev_vr_pos, prev_robot_pos, prev_vr_ori, prev_robot_ori
        )
        
        # Blend positions
        robot_pos = direct_weight * direct_pos + incremental_weight * incr_pos
        
        # Blend orientations (SLERP)
        robot_quat = self._slerp(direct_quat, incr_quat, incremental_weight)
        
        return robot_pos, robot_quat
    
    def _slerp(self, q1, q2, t):
        """Spherical linear interpolation"""
        dot = np.dot(q1, q2)
        
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        dot = np.clip(dot, -1.0, 1.0)
        theta = np.arccos(dot)
        
        if theta < 1e-6:
            return q2
        
        sin_theta = np.sin(theta)
        w1 = np.sin((1.0 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2
    
    def _apply_workspace_limits(self, pos):
        """Clamp position to workspace limits"""
        pos_clamped = pos.copy()
        pos_clamped[0] = np.clip(pos[0], self.ws_x_min, self.ws_x_max)
        pos_clamped[1] = np.clip(pos[1], self.ws_y_min, self.ws_y_max)
        pos_clamped[2] = np.clip(pos[2], self.ws_z_min, self.ws_z_max)
        return pos_clamped
    
    def button_callback(self, msg):
        """Process VR button inputs"""
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.control_enabled = not self.control_enabled
            self.get_logger().info(f'Control {"enabled" if self.control_enabled else "disabled"}')
            
            status_msg = Bool()
            status_msg.data = self.control_enabled
            self.status_pub.publish(status_msg)
            
            if not self.control_enabled:
                self._reset_state()
    
    def enable_callback(self, msg):
        """External control enable/disable"""
        self.control_enabled = msg.data
        self.get_logger().info(f'Control externally {"enabled" if self.control_enabled else "disabled"}')
        
        if not self.control_enabled:
            self._reset_state()
    
    def _reset_state(self):
        """Reset all state variables"""
        self.left_pos_filter.reset()
        self.left_ori_filter.reset()
        self.right_pos_filter.reset()
        self.right_ori_filter.reset()
        
        self.left_prev_vr_pos = None
        self.left_prev_robot_pos = None
        self.left_prev_vr_ori = None
        self.left_prev_robot_ori = None
        
        self.right_prev_vr_pos = None
        self.right_prev_robot_pos = None
        self.right_prev_vr_ori = None
        self.right_prev_robot_ori = None


def main(args=None):
    rclpy.init(args=args)
    node = VRInterfaceNodeEnhanced()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
