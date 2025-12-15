#!/usr/bin/env python3
"""
Print robot joint states and tool poses at specified frequency
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import numpy as np
import math


class PrintRobotInfo(Node):
    def __init__(self):
        super().__init__('print_robot_info')
        
        # Declare parameters with explicit types
        self.declare_parameter('frequency', 5.0)
        self.declare_parameter(
            'left_arm_joints', 
            Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'right_arm_joints', 
            Parameter.Type.STRING_ARRAY,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter('left_tool_frame', 'lt')
        self.declare_parameter('right_tool_frame', 'rt')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        self.frequency = self.get_parameter('frequency').value
        self.left_arm_joints = self.get_parameter('left_arm_joints').value
        self.right_arm_joints = self.get_parameter('right_arm_joints').value
        self.left_tool_frame = self.get_parameter('left_tool_frame').value
        self.right_tool_frame = self.get_parameter('right_tool_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Joint state storage
        self.current_joint_states = {}
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Create timer for printing
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.print_info)
        
        self.get_logger().info(f'Starting robot info printer at {self.frequency} Hz')
        
    def joint_state_callback(self, msg):
        """Store latest joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_tool_pose(self, tool_frame):
        """Get tool pose relative to base frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                tool_frame,
                rclpy.time.Time()
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Extract and convert rotation
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            
            return x, y, z, roll, pitch, yaw
            
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform for {tool_frame}: {str(e)}')
            return None
    
    def print_info(self):
        """Print joint states and tool poses"""
        print('\n' + '='*80)
        print(f'Robot State at {self.get_clock().now().to_msg().sec}s')
        print('='*80)
        
        # Print left arm joint states
        print('\n【Left Arm Joint Angles (rad)】')
        for joint_name in self.left_arm_joints:
            if joint_name in self.current_joint_states:
                angle_rad = self.current_joint_states[joint_name]
                angle_deg = math.degrees(angle_rad)
                print(f'  {joint_name:20s}: {angle_rad:8.4f} rad ({angle_deg:8.2f}°)')
            else:
                print(f'  {joint_name:20s}: N/A')
        
        # Print right arm joint states
        print('\n【Right Arm Joint Angles (rad)】')
        for joint_name in self.right_arm_joints:
            if joint_name in self.current_joint_states:
                angle_rad = self.current_joint_states[joint_name]
                angle_deg = math.degrees(angle_rad)
                print(f'  {joint_name:20s}: {angle_rad:8.4f} rad ({angle_deg:8.2f}°)')
            else:
                print(f'  {joint_name:20s}: N/A')
        
        # Print left tool pose
        print('\n【Left Tool Pose in base_link】')
        left_pose = self.get_tool_pose(self.left_tool_frame)
        if left_pose:
            x, y, z, roll, pitch, yaw = left_pose
            print(f'  Position (XYZ): [{x:8.4f}, {y:8.4f}, {z:8.4f}] m')
            print(f'  Rotation (RPY): [{roll:8.4f}, {pitch:8.4f}, {yaw:8.4f}] rad')
            print(f'                  [{math.degrees(roll):8.2f}°, {math.degrees(pitch):8.2f}°, {math.degrees(yaw):8.2f}°]')
        else:
            print('  N/A')
        
        # Print right tool pose
        print('\n【Right Tool Pose in base_link】')
        right_pose = self.get_tool_pose(self.right_tool_frame)
        if right_pose:
            x, y, z, roll, pitch, yaw = right_pose
            print(f'  Position (XYZ): [{x:8.4f}, {y:8.4f}, {z:8.4f}] m')
            print(f'  Rotation (RPY): [{roll:8.4f}, {pitch:8.4f}, {yaw:8.4f}] rad')
            print(f'                  [{math.degrees(roll):8.2f}°, {math.degrees(pitch):8.2f}°, {math.degrees(yaw):8.2f}°]')
        else:
            print('  N/A')
        
        print('='*80)


def main(args=None):
    rclpy.init(args=args)
    node = PrintRobotInfo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
