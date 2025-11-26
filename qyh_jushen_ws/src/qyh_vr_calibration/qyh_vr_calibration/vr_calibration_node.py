#!/usr/bin/env python3
"""
Enhanced VR Calibration Node - Posture-based calibration with hybrid mapping

This node provides user-specific calibration using 4 natural human postures:
1. Arms down naturally (手臂自然下垂)
2. Arms up vertically (手臂垂直向上)
3. Arms forward horizontal (手臂向前伸直)
4. Arms side horizontal (双臂侧平举)

Computes anisotropic scaling, rotation matrix, and workspace mapping parameters.
"""

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from qyh_vr_calibration_msgs.msg import CalibrationPose
from qyh_vr_calibration_msgs.srv import (
    CalibrateUser,
    LoadUserCalibration,
    DeleteUser,
    ListUsers
)


class VRCalibrationNodeEnhanced(Node):
    def __init__(self):
        super().__init__('vr_calibration_node_enhanced')
        
        # Define storage path: ~/.qyh_robot/users/
        self.storage_dir = os.path.join(
            os.path.expanduser("~"),
            ".qyh_robot",
            "users"
        )
        if not os.path.exists(self.storage_dir):
            os.makedirs(self.storage_dir)
            self.get_logger().info(f"Created user storage: {self.storage_dir}")
        else:
            self.get_logger().info(f"Using user storage: {self.storage_dir}")
        
        # Define robot calibration storage path
        self.robot_storage_dir = os.path.join(
            os.path.expanduser("~"),
            "qyh_jushen_ws",
            "persistent",
            "vr_calibration_robot"
        )
        self.robot_file_path = os.path.join(self.robot_storage_dir, "robot.yaml")
        
        # Load robot reference poses at startup
        self.robot_poses = self._load_robot_poses()
        if self.robot_poses is None:
            self.get_logger().error(f"Failed to load robot.yaml from {self.robot_file_path}")
            self.get_logger().error("Robot reference poses are required for calibration!")
        else:
            self.get_logger().info(f"Loaded {len(self.robot_poses)} robot reference poses")
        
        # Create services for user calibration
        self.srv_calibrate = self.create_service(
            CalibrateUser,
            'vr_calibration/calibrate_user',
            self.calibrate_user_callback
        )
        self.srv_load = self.create_service(
            LoadUserCalibration,
            'vr_calibration/load_user',
            self.load_user_callback
        )
        self.srv_delete = self.create_service(
            DeleteUser,
            'vr_calibration/delete_user',
            self.delete_user_callback
        )
        self.srv_list = self.create_service(
            ListUsers,
            'vr_calibration/list_users',
            self.list_users_callback
        )
        
        self.get_logger().info("Enhanced VR Calibration Node started")
        self.get_logger().info("Services:")
        self.get_logger().info("  - /vr_calibration/calibrate_user")
        self.get_logger().info("  - /vr_calibration/load_user")
        self.get_logger().info("  - /vr_calibration/delete_user")
        self.get_logger().info("  - /vr_calibration/list_users")
    
    def _get_user_file(self, username):
        """Get user calibration file path"""
        safe_username = "".join([c for c in username if c.isalnum() or c in ('-', '_')])
        return os.path.join(self.storage_dir, f"{safe_username}.yaml")
    
    def _load_robot_poses(self):
        """Load fixed robot reference poses from robot.yaml"""
        if not os.path.exists(self.robot_file_path):
            return None
        
        try:
            with open(self.robot_file_path, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'samples' not in data or len(data['samples']) < 4:
                self.get_logger().error("robot.yaml must contain at least 4 samples")
                return None
            
            # Convert to CalibrationPose-like structure
            robot_poses = []
            for sample in data['samples'][:4]:  # Take first 4
                pose = type('obj', (object,), {
                    'left_hand_pose': type('obj', (object,), {
                        'position': type('obj', (object,), sample['left_hand_pose']['position'])(),
                        'orientation': type('obj', (object,), sample['left_hand_pose']['orientation'])()
                    })(),
                    'right_hand_pose': type('obj', (object,), {
                        'position': type('obj', (object,), sample['right_hand_pose']['position'])(),
                        'orientation': type('obj', (object,), sample['right_hand_pose']['orientation'])()
                    })()
                })()
                robot_poses.append(pose)
            
            return robot_poses
            
        except Exception as e:
            self.get_logger().error(f"Error loading robot.yaml: {e}")
            return None
    
    def _pose_to_array(self, pose):
        """Convert Pose message to numpy arrays"""
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        orientation = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        return position, orientation
    
    def _compute_calibration_params(self, vr_poses, robot_poses):
        """
        Compute calibration parameters from 4 postures
        
        Posture indices:
        0: Arms down naturally (手臂自然下垂)
        1: Arms up vertically (手臂垂直向上)
        2: Arms forward horizontal (手臂向前伸直)
        3: Arms side horizontal (双臂侧平举)
        
        Returns:
            scaling: [vertical_scale, forward_scale, lateral_scale]
            rotation_matrix: 3x3 rotation matrix
            vr_offset: VR workspace center
            robot_offset: Robot workspace center
        """
        
        # Extract left hand positions from each posture
        vr_positions = []
        robot_positions = []
        
        for i in range(4):
            vr_pos, _ = self._pose_to_array(vr_poses[i].left_hand_pose)
            robot_pos, _ = self._pose_to_array(robot_poses[i].left_hand_pose)
            vr_positions.append(vr_pos)
            robot_positions.append(robot_pos)
        
        vr_positions = np.array(vr_positions)  # Shape: (4, 3)
        robot_positions = np.array(robot_positions)  # Shape: (4, 3)
        
        # Compute workspace centers (average of all 4 postures)
        vr_offset = np.mean(vr_positions, axis=0)
        robot_offset = np.mean(robot_positions, axis=0)
        
        # Center the data
        vr_centered = vr_positions - vr_offset
        robot_centered = robot_positions - robot_offset
        
        # Compute direction vectors for each axis
        # Vertical: up (posture 1) - down (posture 0)
        vr_vertical = vr_centered[1] - vr_centered[0]
        robot_vertical = robot_centered[1] - robot_centered[0]
        
        # Forward: forward (posture 2) - center
        vr_forward = vr_centered[2]
        robot_forward = robot_centered[2]
        
        # Lateral: side (posture 3) - center
        vr_lateral = vr_centered[3]
        robot_lateral = robot_centered[3]
        
        # Compute anisotropic scaling factors
        vertical_scale = np.linalg.norm(robot_vertical) / (np.linalg.norm(vr_vertical) + 1e-6)
        forward_scale = np.linalg.norm(robot_forward) / (np.linalg.norm(vr_forward) + 1e-6)
        lateral_scale = np.linalg.norm(robot_lateral) / (np.linalg.norm(vr_lateral) + 1e-6)
        
        scaling = np.array([vertical_scale, forward_scale, lateral_scale])
        
        # Compute rotation matrix using SVD (Kabsch algorithm)
        # This finds the optimal rotation to align VR axes to robot axes
        
        # Normalize direction vectors to form basis
        vr_basis = np.column_stack([
            vr_vertical / (np.linalg.norm(vr_vertical) + 1e-6),
            vr_forward / (np.linalg.norm(vr_forward) + 1e-6),
            vr_lateral / (np.linalg.norm(vr_lateral) + 1e-6)
        ])
        
        robot_basis = np.column_stack([
            robot_vertical / (np.linalg.norm(robot_vertical) + 1e-6),
            robot_forward / (np.linalg.norm(robot_forward) + 1e-6),
            robot_lateral / (np.linalg.norm(robot_lateral) + 1e-6)
        ])
        
        # Compute rotation: R = robot_basis @ vr_basis.T
        # This transforms from VR coordinate frame to robot coordinate frame
        H = vr_basis.T @ robot_basis
        U, _, Vt = np.linalg.svd(H)
        rotation_matrix = Vt.T @ U.T
        
        # Ensure proper rotation (det(R) = 1, not -1)
        if np.linalg.det(rotation_matrix) < 0:
            Vt[-1, :] *= -1
            rotation_matrix = Vt.T @ U.T
        
        self.get_logger().info(f"Calibration computed:")
        self.get_logger().info(f"  Vertical scale: {vertical_scale:.3f}")
        self.get_logger().info(f"  Forward scale: {forward_scale:.3f}")
        self.get_logger().info(f"  Lateral scale: {lateral_scale:.3f}")
        self.get_logger().info(f"  VR offset: {vr_offset}")
        self.get_logger().info(f"  Robot offset: {robot_offset}")
        
        return scaling, rotation_matrix, vr_offset, robot_offset
    
    def calibrate_user_callback(self, request, response):
        """
        Calibrate a user with 4 natural postures
        Robot poses are loaded from robot.yaml (fixed reference)
        """
        username = request.username
        
        try:
            # Validate robot poses loaded
            if self.robot_poses is None:
                response.success = False
                response.message = "Robot reference poses not loaded. Check robot.yaml"
                return response
            
            # Validate input
            if len(request.vr_poses) != 4:
                response.success = False
                response.message = "Must provide exactly 4 VR poses"
                return response
            
            # Compute calibration parameters using user VR poses and fixed robot poses
            scaling, rotation_matrix, vr_offset, robot_offset = self._compute_calibration_params(
                request.vr_poses,
                self.robot_poses  # Use loaded robot poses
            )
            
            # Prepare data for saving
            calibration_data = {
                'username': username,
                'calibration': {
                    'scaling': scaling.tolist(),
                    'rotation_matrix': rotation_matrix.flatten().tolist(),  # 9 elements
                    'vr_offset': vr_offset.tolist(),
                    'robot_offset': robot_offset.tolist()
                },
                'postures': {
                    'vr': [self._pose_to_dict(p.left_hand_pose) for p in request.vr_poses],
                    'robot': [self._pose_to_dict(p.left_hand_pose) for p in self.robot_poses]
                }
            }
            
            # Save to file
            user_file = self._get_user_file(username)
            with open(user_file, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
            
            self.get_logger().info(f"Calibration saved for user: {username}")
            
            # Return computed parameters in response
            response.success = True
            response.message = f"Calibration completed for {username}"
            response.scaling = scaling.tolist()
            response.rotation_matrix = rotation_matrix.flatten().tolist()
            
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            response.success = False
            response.message = f"Calibration error: {str(e)}"
        
        return response
    
    def load_user_callback(self, request, response):
        """
        Load calibration parameters for a user
        """
        username = request.username
        user_file = self._get_user_file(username)
        
        if not os.path.exists(user_file):
            response.success = False
            response.found = False
            response.message = f"No calibration found for user: {username}"
            return response
        
        try:
            with open(user_file, 'r') as f:
                data = yaml.safe_load(f)
            
            calib = data['calibration']
            
            response.success = True
            response.found = True
            response.message = f"Calibration loaded for {username}"
            response.scaling = calib['scaling']
            response.rotation_matrix = calib['rotation_matrix']
            response.vr_offset = calib['vr_offset']
            response.robot_offset = calib['robot_offset']
            
            self.get_logger().info(f"Calibration loaded for: {username}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            response.success = False
            response.found = False
            response.message = f"Load error: {str(e)}"
        
        return response
    
    def delete_user_callback(self, request, response):
        """
        Delete user calibration
        """
        username = request.username
        user_file = self._get_user_file(username)
        
        if not os.path.exists(user_file):
            response.success = False
            response.message = f"No calibration found for user: {username}"
            return response
        
        try:
            os.remove(user_file)
            response.success = True
            response.message = f"Calibration deleted for {username}"
            self.get_logger().info(f"Calibration deleted: {username}")
        except Exception as e:
            response.success = False
            response.message = f"Delete error: {str(e)}"
        
        return response
    
    def list_users_callback(self, request, response):
        """
        List all calibrated users
        """
        try:
            yaml_files = [f for f in os.listdir(self.storage_dir) if f.endswith('.yaml')]
            usernames = [os.path.splitext(f)[0] for f in yaml_files]
            
            response.usernames = usernames
            response.message = f"Found {len(usernames)} calibrated users"
            
            self.get_logger().info(f"Listed {len(usernames)} users")
            
        except Exception as e:
            response.usernames = []
            response.message = f"Error listing users: {str(e)}"
        
        return response
    
    def _pose_to_dict(self, pose):
        """Convert Pose message to dict"""
        return {
            'position': {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z)
            },
            'orientation': {
                'x': float(pose.orientation.x),
                'y': float(pose.orientation.y),
                'z': float(pose.orientation.z),
                'w': float(pose.orientation.w)
            }
        }


def main(args=None):
    rclpy.init(args=args)
    node = VRCalibrationNodeEnhanced()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
