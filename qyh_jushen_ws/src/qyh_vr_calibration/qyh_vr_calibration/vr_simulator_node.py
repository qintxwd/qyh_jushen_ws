#!/usr/bin/env python3
"""
VR Simulator Node - 模拟VR手柄数据用于测试

功能:
1. 发布模拟的VR手柄位姿和按键
2. 支持键盘控制位置移动
3. 支持鼠标/滑块控制grip值

键盘控制:
- W/S: 前后 (Y轴)
- A/D: 左右 (X轴)  
- Q/E: 上下 (Z轴)
- Space: 切换grip开关 (0.0 <-> 1.0)
- Tab: 切换控制左/右手
- R: 重置位置
- 1-9: 设置速度 (1=慢, 9=快)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
import sys
import threading
import time

# 尝试导入keyboard库，如果没有则使用简单的stdin
try:
    import keyboard
    HAS_KEYBOARD = True
except ImportError:
    HAS_KEYBOARD = False


class VRSimulatorNode(Node):
    """VR模拟器节点"""
    
    def __init__(self):
        super().__init__('vr_simulator_node')
        
        # 参数
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('initial_left_pos', [0.3, 0.3, 0.5])
        self.declare_parameter('initial_right_pos', [0.3, -0.3, 0.5])
        self.declare_parameter('move_speed', 0.005)  # m per key press
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.move_speed = self.get_parameter('move_speed').value
        
        # 左手状态
        initial_left = self.get_parameter('initial_left_pos').value
        self.left_pos = np.array(initial_left, dtype=np.float64)
        self.left_ori = np.array([0.0, 0.0, 0.0, 1.0])  # 单位四元数
        self.left_grip = 0.0
        
        # 右手状态
        initial_right = self.get_parameter('initial_right_pos').value
        self.right_pos = np.array(initial_right, dtype=np.float64)
        self.right_ori = np.array([0.0, 0.0, 0.0, 1.0])
        self.right_grip = 0.0
        
        # 当前控制的手 ('left' or 'right')
        self.active_hand = 'left'
        
        # 保存初始位置用于重置
        self.initial_left_pos = self.left_pos.copy()
        self.initial_right_pos = self.right_pos.copy()
        
        # === 发布者 ===
        self.left_pose_pub = self.create_publisher(
            PoseStamped, '/vr/left_hand/pose', 10)
        self.right_pose_pub = self.create_publisher(
            PoseStamped, '/vr/right_hand/pose', 10)
        self.left_joy_pub = self.create_publisher(
            Joy, '/vr/left_hand/joy', 10)
        self.right_joy_pub = self.create_publisher(
            Joy, '/vr/right_hand/joy', 10)
        
        # 状态显示
        self.status_pub = self.create_publisher(
            String, '/vr_simulator/status', 10)
        
        # 发布定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_loop)
        
        # 状态显示定时器
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        # 键盘输入线程
        self.running = True
        if HAS_KEYBOARD:
            self.setup_keyboard_hooks()
        else:
            self.input_thread = threading.Thread(target=self.stdin_input_loop)
            self.input_thread.daemon = True
            self.input_thread.start()
        
        self.get_logger().info('=== VR Simulator Node Started ===')
        self.get_logger().info('Controls:')
        self.get_logger().info('  W/S: Forward/Back (Y)')
        self.get_logger().info('  A/D: Left/Right (X)')
        self.get_logger().info('  Q/E: Up/Down (Z)')
        self.get_logger().info('  Space: Toggle Grip')
        self.get_logger().info('  Tab: Switch Left/Right hand')
        self.get_logger().info('  R: Reset position')
        self.get_logger().info(f'Active hand: {self.active_hand.upper()}')
    
    def setup_keyboard_hooks(self):
        """设置键盘钩子（需要keyboard库）"""
        keyboard.on_press_key('w', lambda _: self.move_active(0, 1, 0))
        keyboard.on_press_key('s', lambda _: self.move_active(0, -1, 0))
        keyboard.on_press_key('a', lambda _: self.move_active(-1, 0, 0))
        keyboard.on_press_key('d', lambda _: self.move_active(1, 0, 0))
        keyboard.on_press_key('q', lambda _: self.move_active(0, 0, 1))
        keyboard.on_press_key('e', lambda _: self.move_active(0, 0, -1))
        keyboard.on_press_key('space', lambda _: self.toggle_grip())
        keyboard.on_press_key('tab', lambda _: self.switch_hand())
        keyboard.on_press_key('r', lambda _: self.reset_position())
        
        # 数字键设置速度
        for i in range(1, 10):
            keyboard.on_press_key(str(i), lambda _, n=i: self.set_speed(n))
    
    def stdin_input_loop(self):
        """标准输入循环（备用方案）"""
        self.get_logger().info('Using stdin input (press keys then Enter)')
        while self.running:
            try:
                line = sys.stdin.readline().strip().lower()
                for char in line:
                    if char == 'w':
                        self.move_active(0, 1, 0)
                    elif char == 's':
                        self.move_active(0, -1, 0)
                    elif char == 'a':
                        self.move_active(-1, 0, 0)
                    elif char == 'd':
                        self.move_active(1, 0, 0)
                    elif char == 'q':
                        self.move_active(0, 0, 1)
                    elif char == 'e':
                        self.move_active(0, 0, -1)
                    elif char == ' ':
                        self.toggle_grip()
                    elif char == '\t':
                        self.switch_hand()
                    elif char == 'r':
                        self.reset_position()
                    elif char.isdigit() and char != '0':
                        self.set_speed(int(char))
            except Exception:
                break
    
    def move_active(self, dx: float, dy: float, dz: float):
        """移动当前激活的手"""
        delta = np.array([dx, dy, dz]) * self.move_speed
        if self.active_hand == 'left':
            self.left_pos += delta
        else:
            self.right_pos += delta
    
    def toggle_grip(self):
        """切换grip值"""
        if self.active_hand == 'left':
            self.left_grip = 0.0 if self.left_grip > 0.5 else 1.0
            self.get_logger().info(f'Left grip: {self.left_grip}')
        else:
            self.right_grip = 0.0 if self.right_grip > 0.5 else 1.0
            self.get_logger().info(f'Right grip: {self.right_grip}')
    
    def switch_hand(self):
        """切换控制的手"""
        self.active_hand = 'right' if self.active_hand == 'left' else 'left'
        self.get_logger().info(f'Switched to: {self.active_hand.upper()} hand')
    
    def reset_position(self):
        """重置位置"""
        if self.active_hand == 'left':
            self.left_pos = self.initial_left_pos.copy()
        else:
            self.right_pos = self.initial_right_pos.copy()
        self.get_logger().info(f'Reset {self.active_hand} hand position')
    
    def set_speed(self, level: int):
        """设置移动速度 (1-9)"""
        self.move_speed = 0.001 * level  # 1mm to 9mm
        self.get_logger().info(f'Move speed: {self.move_speed*1000:.1f} mm/step')
    
    def publish_loop(self):
        """发布VR数据"""
        now = self.get_clock().now().to_msg()
        
        # 发布左手位姿
        left_pose = PoseStamped()
        left_pose.header.stamp = now
        left_pose.header.frame_id = 'world'
        left_pose.pose.position.x = float(self.left_pos[0])
        left_pose.pose.position.y = float(self.left_pos[1])
        left_pose.pose.position.z = float(self.left_pos[2])
        left_pose.pose.orientation.x = float(self.left_ori[0])
        left_pose.pose.orientation.y = float(self.left_ori[1])
        left_pose.pose.orientation.z = float(self.left_ori[2])
        left_pose.pose.orientation.w = float(self.left_ori[3])
        self.left_pose_pub.publish(left_pose)
        
        # 发布左手按键
        left_joy = Joy()
        left_joy.header.stamp = now
        left_joy.axes = [0.0, 0.0, 0.0, float(self.left_grip)]  # grip在axes[3]
        left_joy.buttons = []
        self.left_joy_pub.publish(left_joy)
        
        # 发布右手位姿
        right_pose = PoseStamped()
        right_pose.header.stamp = now
        right_pose.header.frame_id = 'world'
        right_pose.pose.position.x = float(self.right_pos[0])
        right_pose.pose.position.y = float(self.right_pos[1])
        right_pose.pose.position.z = float(self.right_pos[2])
        right_pose.pose.orientation.x = float(self.right_ori[0])
        right_pose.pose.orientation.y = float(self.right_ori[1])
        right_pose.pose.orientation.z = float(self.right_ori[2])
        right_pose.pose.orientation.w = float(self.right_ori[3])
        self.right_pose_pub.publish(right_pose)
        
        # 发布右手按键
        right_joy = Joy()
        right_joy.header.stamp = now
        right_joy.axes = [0.0, 0.0, 0.0, float(self.right_grip)]
        right_joy.buttons = []
        self.right_joy_pub.publish(right_joy)
    
    def publish_status(self):
        """发布状态信息"""
        status = (
            f"Active: {self.active_hand.upper()} | "
            f"L: [{self.left_pos[0]:.3f}, {self.left_pos[1]:.3f}, {self.left_pos[2]:.3f}] "
            f"grip={self.left_grip:.1f} | "
            f"R: [{self.right_pos[0]:.3f}, {self.right_pos[1]:.3f}, {self.right_pos[2]:.3f}] "
            f"grip={self.right_grip:.1f}"
        )
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        self.running = False
        if HAS_KEYBOARD:
            keyboard.unhook_all()
        super().destroy_node()


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
