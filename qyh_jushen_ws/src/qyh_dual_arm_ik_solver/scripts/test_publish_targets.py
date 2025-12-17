#!/usr/bin/env python3
"""
手动测试IK求解器 - 发布测试目标位姿
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

# left joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
# right joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
# left pos = -0.626130, 989.737160, 219.885132, 1.572874, -0.000000, -3.141593
# right pos = -0.679880, -989.449941, 217.950931, 1.574990, 0.000000, 0.000000

class IKTestPublisher(Node):
    def __init__(self):
        super().__init__('ik_test_publisher')
        
        # 发布测试目标
        self.left_pub = self.create_publisher(
            PoseStamped, '/teleop/left_hand/target', 10)
        self.right_pub = self.create_publisher(
            PoseStamped, '/teleop/right_hand/target', 10)
        
        # 定时发布（10Hz）
        self.timer = self.create_timer(0.1, self.publish_test_poses)
        self.t = 0.0
        
        self.get_logger().info("📤 开始发布测试位姿...")
        self.get_logger().info("左臂: 零位 + Z轴正弦运动")
        self.get_logger().info("右臂: 零位 + Z轴正弦运动（相位相反）")
    
    def publish_test_poses(self):
        self.t += 0.1
        
        # 左臂测试位姿（已知：全0关节对应的正解位姿）
        # left joint = 0,0,0,0,0,0,0
        # left pos = -0.626130, 989.737160, 219.885132, 1.572874, -0.000000, -3.141593
        left_pose = PoseStamped()
        left_pose.header.stamp = self.get_clock().now().to_msg()
        left_pose.header.frame_id = 'base_link_left'
        left_pose.pose.position.x = -0.626130 / 1000.0
        left_pose.pose.position.y = 989.737160 / 1000.0
        left_pose.pose.position.z = 219.885132 / 1000.0
        
        left_roll = 1.572874
        left_pitch = 0.0
        left_yaw = -3.141593
        # 将 RPY 转为四元数
        cr = math.cos(left_roll * 0.5)
        sr = math.sin(left_roll * 0.5)
        cp = math.cos(left_pitch * 0.5)
        sp = math.sin(left_pitch * 0.5)
        cy = math.cos(left_yaw * 0.5)
        sy = math.sin(left_yaw * 0.5)

        left_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        left_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        left_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        left_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        
        # 右臂测试位姿（已知：全0关节对应的正解位姿）
        # right joint = 0,0,0,0,0,0,0
        # right pos = -0.679880, -989.449941, 217.950931, 1.574990, 0.000000, 0.000000
        right_pose = PoseStamped()
        right_pose.header.stamp = self.get_clock().now().to_msg()
        right_pose.header.frame_id = 'base_link_right'
        right_pose.pose.position.x = -0.679880 / 1000.0
        right_pose.pose.position.y = -989.449941 / 1000.0
        right_pose.pose.position.z = 217.950931 / 1000.0
        
        right_roll = 1.574990
        right_pitch = 0.0
        right_yaw = 0.0
        # 将 RPY 转为四元数
        cr = math.cos(right_roll * 0.5)
        sr = math.sin(right_roll * 0.5)
        cp = math.cos(right_pitch * 0.5)
        sp = math.sin(right_pitch * 0.5)
        cy = math.cos(right_yaw * 0.5)
        sy = math.sin(right_yaw * 0.5)

        right_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        right_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        right_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        right_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        
        self.left_pub.publish(left_pose)
        self.right_pub.publish(right_pose)
        
        if int(self.t * 10) % 10 == 0:
            self.get_logger().info(f'⏱️  t={self.t:.1f}s, 左Z={left_pose.pose.position.z:.4f}m')

def main():
    rclpy.init()
    node = IKTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
