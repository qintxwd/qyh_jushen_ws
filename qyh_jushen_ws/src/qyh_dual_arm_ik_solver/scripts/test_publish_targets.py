#!/usr/bin/env python3
"""
手动测试IK求解器 - 发布测试目标位姿
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

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
        
        # 左臂测试位姿（基于实测零位）
        left_pose = PoseStamped()
        left_pose.header.stamp = self.get_clock().now().to_msg()
        left_pose.header.frame_id = 'base_link_left'
        left_pose.pose.position.x = 0.0
        left_pose.pose.position.y = 0.9927  # 992.7mm
        left_pose.pose.position.z = 0.220 + 0.01 * math.sin(self.t)  # ±10mm
        left_pose.pose.orientation.x = 0.0
        left_pose.pose.orientation.y = 0.0
        left_pose.pose.orientation.z = 0.0
        left_pose.pose.orientation.w = 1.0
        
        # 右臂测试位姿（镜像对称）
        right_pose = PoseStamped()
        right_pose.header.stamp = self.get_clock().now().to_msg()
        right_pose.header.frame_id = 'base_link_right'
        right_pose.pose.position.x = 0.0
        right_pose.pose.position.y = -0.9927
        right_pose.pose.position.z = 0.220 + 0.01 * math.sin(self.t + math.pi)
        right_pose.pose.orientation.x = 0.0
        right_pose.pose.orientation.y = 0.0
        right_pose.pose.orientation.z = 0.0
        right_pose.pose.orientation.w = 1.0
        
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
