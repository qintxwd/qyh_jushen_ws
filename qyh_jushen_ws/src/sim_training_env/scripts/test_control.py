#!/usr/bin/env python3
"""
简单的控制测试脚本
测试能否向机械臂发送轨迹命令
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ControlTester(Node):
    def __init__(self):
        super().__init__('control_tester')
        
        # 创建发布者
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Control Tester initialized')
        self.get_logger().info('Waiting 2 seconds before sending command...')
        
        # 延迟发送测试命令
        self.timer = self.create_timer(2.0, self.send_test_trajectory)
        self.command_sent = False
    
    def send_test_trajectory(self):
        if self.command_sent:
            return
        
        self.get_logger().info('Sending test trajectory to right arm...')
        
        # 创建轨迹消息
        traj = JointTrajectory()
        traj.joint_names = ['r-j1', 'r-j2', 'r-j3', 'r-j4', 'r-j5', 'r-j6', 'r-j7']
        
        # 第一个点：当前位置（全零）
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        
        # 第二个点：稍微移动
        point2 = JointTrajectoryPoint()
        point2.positions = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)
        
        # 第三个点：回到原位
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=4, nanosec=0)
        
        traj.points = [point1, point2, point3]
        
        # 发布
        self.right_arm_pub.publish(traj)
        self.get_logger().info('✓ Test trajectory sent! Watch the right arm move.')
        self.command_sent = True
        
        # 5秒后关闭
        self.create_timer(6.0, self.shutdown)
    
    def shutdown(self):
        self.get_logger().info('Test complete. Shutting down.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
