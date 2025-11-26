#!/usr/bin/env python3
"""
User Calibration Tool - Interactive CLI for 4-posture calibration

Guides user through 4 natural postures:
1. Arms down naturally (手臂自然下垂)
2. Arms up vertically (手臂垂直向上)
3. Arms forward horizontal (手臂向前伸直)
4. Arms side horizontal (双臂侧平举)

Robot reference poses are loaded from robot.yaml (fixed)
Only captures user VR poses

Completion time: ~30 seconds
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from qyh_vr_calibration_msgs.msg import CalibrationPose
from qyh_vr_calibration_msgs.srv import CalibrateUser
import time
import sys


class CalibrationTool(Node):
    def __init__(self):
        super().__init__('calibration_tool')
        
        # Subscribers for VR hand poses
        self.left_vr_pose = None
        self.right_vr_pose = None
        
        self.left_vr_sub = self.create_subscription(
            PoseStamped,
            '/vr/left_hand/pose',
            lambda msg: setattr(self, 'left_vr_pose', msg),
            10
        )
        self.right_vr_sub = self.create_subscription(
            PoseStamped,
            '/vr/right_hand/pose',
            lambda msg: setattr(self, 'right_vr_pose', msg),
            10
        )
        
        # Service client for calibration
        self.calibrate_client = self.create_client(
            CalibrateUser,
            '/vr_calibration/calibrate_user'
        )
        
        self.get_logger().info('Calibration Tool initialized')
    
    def wait_for_vr_data(self, timeout=5.0):
        """Wait for VR data to be available"""
        start_time = time.time()
        while (self.left_vr_pose is None or self.right_vr_pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                return False
        return True
    
    def capture_current_poses(self):
        """Capture current VR poses only"""
        rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.left_vr_pose is None or self.right_vr_pose is None:
            return None
        
        calib_pose = CalibrationPose()
        calib_pose.left_hand_pose = self.left_vr_pose.pose
        calib_pose.right_hand_pose = self.right_vr_pose.pose
        
        return calib_pose
    
    def run_calibration(self, username):
        """Run interactive calibration workflow"""
        
        print("\n" + "="*60)
        print("  VR Teleoperation User Calibration")
        print("="*60)
        print(f"\nCalibrating user: {username}")
        print("\nYou will be guided through 4 natural postures.")
        print("Robot reference poses are loaded from robot.yaml (fixed).")
        print("Each posture takes ~7 seconds.")
        print("\nPostures:")
        print("  1. Arms down naturally (手臂自然下垂)")
        print("  2. Arms up vertically (手臂垂直向上)")
        print("  3. Arms forward horizontal (手臂向前伸直)")
        print("  4. Arms side horizontal (双臂侧平举)")
        print("\n" + "="*60)
        
        # Wait for VR connection
        print("\n[1/5] Waiting for VR controllers...")
        if not self.wait_for_vr_data(timeout=10.0):
            print("❌ ERROR: VR controllers not detected. Please check VR system.")
            return False
        print("✓ VR controllers connected")
        
        # Capture 4 postures
        posture_names = [
            "Arms down naturally (手臂自然下垂)",
            "Arms up vertically (手臂垂直向上)",
            "Arms forward horizontal (手臂向前伸直)",
            "Arms side horizontal (双臂侧平举)"
        ]
        
        vr_poses = []
        
        for i, posture_name in enumerate(posture_names):
            print(f"\n[{i+2}/5] Posture {i+1}: {posture_name}")
            print("      Position yourself...")
            
            # Countdown
            for countdown in range(3, 0, -1):
                print(f"      {countdown}...", end='', flush=True)
                time.sleep(1.0)
            print(" Capturing!")
            
            # Capture VR pose only
            vr_pose = self.capture_current_poses()
            if vr_pose is None:
                print("❌ ERROR: Failed to capture VR pose")
                return False
            
            vr_pose.sample_index = i
            vr_poses.append(vr_pose)
            
            print("      ✓ Captured!")
            time.sleep(0.5)
        
        # Send calibration request
        print(f"\n[6/5] Computing calibration parameters...")
        print("       (Using robot reference poses from robot.yaml)")
        
        if not self.calibrate_client.wait_for_service(timeout_sec=3.0):
            print("❌ ERROR: Calibration service not available")
            return False
        
        request = CalibrateUser.Request()
        request.username = username
        request.vr_poses = vr_poses
        # Robot poses loaded from robot.yaml by calibration service
        
        future = self.calibrate_client.call_async(request)
        
        # Wait for response
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                break
        
        try:
            response = future.result()
            if response.success:
                print("\n✓ Calibration completed successfully!")
                print(f"\nCalibration parameters:")
                print(f"  Vertical scale:  {response.scaling[0]:.3f}")
                print(f"  Forward scale:   {response.scaling[1]:.3f}")
                print(f"  Lateral scale:   {response.scaling[2]:.3f}")
                print(f"\nCalibration saved to: ~/.qyh_robot/users/{username}.yaml")
                print("\n" + "="*60)
                return True
            else:
                print(f"\n❌ ERROR: {response.message}")
                return False
        except Exception as e:
            print(f"\n❌ ERROR: {e}")
            return False


def main():
    rclpy.init()
    
    tool = CalibrationTool()
    
    # Get username from command line
    if len(sys.argv) < 2:
        print("\nUsage: ros2 run qyh_vr_calibration calibration_tool <username>")
        print("Example: ros2 run qyh_vr_calibration calibration_tool john_doe")
        sys.exit(1)
    
    username = sys.argv[1]
    
    try:
        success = tool.run_calibration(username)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user")
        sys.exit(1)
    finally:
        tool.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
