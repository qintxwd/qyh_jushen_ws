#!/usr/bin/env python3
"""
手动测试IK求解器 - 发布测试目标位姿
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

# 第一套，机械臂完全伸直，所有关节角度为0时的正解位姿（单位mm和rad）：
# left joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
# right joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
# left pos = -0.626130, 989.737160, 219.885132, 1.572874, -0.000000, -3.141593
# right pos = -0.679880, -989.449941, 217.950931, 1.574990, 0.000000, 0.000000

# 第二套，机械臂类似人将双手向前合并的样子
# left pos = 608.320432, 323.753614, 215.913915, 90.447975, -0.399704, 120.914392
# right pos = 574.599277, -318.357499, 216.269154, 90.406400, 0.482981, 45.002262
# left joint = 0.004677, -1.030041, 0.003351, -1.398358, -0.001902, 1.397188, 0.000262
# right joint = -0.001571, -1.134639, -0.005952, -1.395653, 0.004590, -1.744875, -0.000279
presets = [
    {
        'left_pos': [-0.626130, 989.737160, 219.885132, 1.572874, 0.0, -3.141593],
        'right_pos': [-0.679880, -989.449941, 217.950931, 1.574990, 0.0, 0.0],
        'left_joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'right_joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    {
        'left_pos': [608.320432, 323.753614, 215.913915, 1.579447975, -0.006976, 2.110914392],
        'right_pos': [574.599277, -318.357499, 216.269154, 1.579406400, 0.008426, 0.785002262],
        'left_joint': [0.004677, -1.030041, 0.003351, -1.398358, -0.001902, 1.397188, 0.000262],
        'right_joint': [-0.001571, -1.134639, -0.005952, -1.395653, 0.004590, -1.744875, -0.000279],
    },
]
class IKTestPublisher(Node):
    def __init__(self):
        super().__init__('ik_test_publisher')
        
        # 发布测试目标
        self.left_pub = self.create_publisher(
            PoseStamped, '/teleop/left_hand/target', 10)
        self.right_pub = self.create_publisher(
            PoseStamped, '/teleop/right_hand/target', 10)
        
        # 使用第几套预设
        self.preset_index = 1
        
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
        left_pose.pose.position.x = presets[self.preset_index]['left_pos'][0] / 1000.0
        left_pose.pose.position.y = presets[self.preset_index]['left_pos'][1] / 1000.0 +  self.preset_index * 0.02 * math.sin(self.t + math.pi)
        left_pose.pose.position.z = presets[self.preset_index]['left_pos'][2] / 1000.0 + self.preset_index * 0.02 * math.sin(self.t)
        
        left_roll = presets[self.preset_index]['left_pos'][3]
        left_pitch = presets[self.preset_index]['left_pos'][4]
        left_yaw = presets[self.preset_index]['left_pos'][5]
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
        right_pose.pose.position.x = presets[self.preset_index]['right_pos'][0] / 1000.0
        right_pose.pose.position.y = presets[self.preset_index]['right_pos'][1] / 1000.0 + self.preset_index * 0.02 * math.sin(self.t + math.pi)
        right_pose.pose.position.z = presets[self.preset_index]['right_pos'][2] / 1000.0 + self.preset_index * 0.02 * math.sin(self.t)
        
        right_roll = presets[self.preset_index]['right_pos'][3]
        right_pitch = presets[self.preset_index]['right_pos'][4]
        right_yaw = presets[self.preset_index]['right_pos'][5]
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
