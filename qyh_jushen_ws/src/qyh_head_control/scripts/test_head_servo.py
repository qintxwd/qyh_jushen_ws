#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
头部舵机测试脚本
测试各种控制模式
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
import time
import sys


class HeadServoTest(Node):
    """头部舵机测试节点"""
    
    def __init__(self):
        super().__init__('head_servo_test')
        
        # 创建发布器
        self.pan_pos_pub = self.create_publisher(Float64, 'head/pan_position', 10)
        self.tilt_pos_pub = self.create_publisher(Float64, 'head/tilt_position', 10)
        self.pan_norm_pub = self.create_publisher(Float64, 'head/pan_normalized', 10)
        self.tilt_norm_pub = self.create_publisher(Float64, 'head/tilt_normalized', 10)
        self.target_pub = self.create_publisher(Point, 'head/target_point', 10)
        
        time.sleep(1)  # 等待连接建立
        
        self.get_logger().info('头部舵机测试节点已启动')
    
    def test_position_control(self):
        """测试位置控制 (0-1000)"""
        self.get_logger().info('=== 测试1: 位置控制 (0-1000) ===')
        
        positions = [500, 700, 500, 300, 500]  # 中-右-中-左-中
        
        for pos in positions:
            self.get_logger().info(f'Pan移动到位置: {pos}')
            msg = Float64()
            msg.data = float(pos)
            self.pan_pos_pub.publish(msg)
            time.sleep(1.5)
        
        positions = [500, 700, 500, 300, 500]  # 中-上-中-下-中
        
        for pos in positions:
            self.get_logger().info(f'Tilt移动到位置: {pos}')
            msg = Float64()
            msg.data = float(pos)
            self.tilt_pos_pub.publish(msg)
            time.sleep(1.5)
    
    def test_normalized_control(self):
        """测试归一化控制 (-1.0 到 1.0)"""
        self.get_logger().info('=== 测试2: 归一化控制 (-1.0 到 1.0) ===')
        
        values = [0.0, 0.5, 0.0, -0.5, 0.0]  # 中-右-中-左-中
        
        for val in values:
            self.get_logger().info(f'Pan归一化值: {val}')
            msg = Float64()
            msg.data = val
            self.pan_norm_pub.publish(msg)
            time.sleep(1.5)
        
        values = [0.0, 0.5, 0.0, -0.5, 0.0]  # 中-上-中-下-中
        
        for val in values:
            self.get_logger().info(f'Tilt归一化值: {val}')
            msg = Float64()
            msg.data = val
            self.tilt_norm_pub.publish(msg)
            time.sleep(1.5)
    
    def test_target_tracking(self):
        """测试目标跟踪模式"""
        self.get_logger().info('=== 测试3: 目标跟踪模式 ===')
        
        # 模拟目标在屏幕上的位置
        targets = [
            (0.0, 0.0),    # 中心
            (0.5, 0.0),    # 右侧
            (0.5, 0.5),    # 右上
            (0.0, 0.5),    # 正上
            (-0.5, 0.5),   # 左上
            (-0.5, 0.0),   # 左侧
            (-0.5, -0.5),  # 左下
            (0.0, -0.5),   # 正下
            (0.5, -0.5),   # 右下
            (0.0, 0.0),    # 回中心
        ]
        
        for x, y in targets:
            self.get_logger().info(f'目标位置: x={x:.2f}, y={y:.2f}')
            msg = Point()
            msg.x = x
            msg.y = y
            msg.z = 0.0
            self.target_pub.publish(msg)
            time.sleep(1.0)
    
    def test_scan_pattern(self):
        """测试扫描模式"""
        self.get_logger().info('=== 测试4: 扫描模式 ===')
        
        # 水平扫描
        self.get_logger().info('水平扫描...')
        for i in range(5):
            for pos in range(300, 701, 100):
                msg = Float64()
                msg.data = float(pos)
                self.pan_pos_pub.publish(msg)
                time.sleep(0.3)
            for pos in range(700, 299, -100):
                msg = Float64()
                msg.data = float(pos)
                self.pan_pos_pub.publish(msg)
                time.sleep(0.3)
        
        # 归中
        msg = Float64()
        msg.data = 500.0
        self.pan_pos_pub.publish(msg)
        time.sleep(1.0)
        
        # 垂直扫描
        self.get_logger().info('垂直扫描...')
        for i in range(5):
            for pos in range(300, 701, 100):
                msg = Float64()
                msg.data = float(pos)
                self.tilt_pos_pub.publish(msg)
                time.sleep(0.3)
            for pos in range(700, 299, -100):
                msg = Float64()
                msg.data = float(pos)
                self.tilt_pos_pub.publish(msg)
                time.sleep(0.3)
        
        # 归中
        msg = Float64()
        msg.data = 500.0
        self.tilt_pos_pub.publish(msg)
        time.sleep(1.0)
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('=================================')
        self.get_logger().info('      头部舵机测试开始')
        self.get_logger().info('=================================')
        
        try:
            self.test_position_control()
            time.sleep(1)
            
            self.test_normalized_control()
            time.sleep(1)
            
            self.test_target_tracking()
            time.sleep(1)
            
            self.test_scan_pattern()
            
            self.get_logger().info('=================================')
            self.get_logger().info('      所有测试完成！')
            self.get_logger().info('=================================')
            
        except KeyboardInterrupt:
            self.get_logger().info('测试被中断')


def main(args=None):
    rclpy.init(args=args)
    
    test_node = HeadServoTest()
    
    print("\n头部舵机测试菜单:")
    print("1 - 位置控制测试")
    print("2 - 归一化控制测试")
    print("3 - 目标跟踪测试")
    print("4 - 扫描模式测试")
    print("5 - 运行所有测试")
    print("q - 退出")
    
    try:
        choice = input("\n请选择测试项目 (1-5 或 q): ").strip()
        
        if choice == '1':
            test_node.test_position_control()
        elif choice == '2':
            test_node.test_normalized_control()
        elif choice == '3':
            test_node.test_target_tracking()
        elif choice == '4':
            test_node.test_scan_pattern()
        elif choice == '5':
            test_node.run_all_tests()
        elif choice.lower() == 'q':
            print("退出测试")
        else:
            print("无效选择")
    
    except KeyboardInterrupt:
        print("\n测试被中断")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
