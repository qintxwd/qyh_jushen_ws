#!/usr/bin/env python3
"""
JAKA机器人伺服控制示例

演示如何使用伺服模式进行实时轨迹控制
伺服模式适用于需要高频率、低延迟的运动控制场景
"""

import rclpy
from rclpy.node import Node
from jaka_control.srv import (
    EnableRobot, ServoMoveEnable, ServoJ, ServoP, SetServoFilter
)
from geometry_msgs.msg import Pose
import math
import time


class JakaServoExample(Node):
    """JAKA机器人伺服控制示例类"""
    
    def __init__(self):
        super().__init__('jaka_servo_example')
        
        # 创建服务客户端
        self.enable_client = self.create_client(
            EnableRobot, '/jaka_robot_node/enable_robot')
        self.servo_enable_client = self.create_client(
            ServoMoveEnable, '/jaka_robot_node/servo_move_enable')
        self.servo_j_client = self.create_client(
            ServoJ, '/jaka_robot_node/servo_j')
        self.servo_p_client = self.create_client(
            ServoP, '/jaka_robot_node/servo_p')
        self.filter_client = self.create_client(
            SetServoFilter, '/jaka_robot_node/set_servo_filter')
        
        self.wait_for_services()
    
    def wait_for_services(self):
        """等待所有服务可用"""
        services = [
            self.enable_client,
            self.servo_enable_client,
            self.servo_j_client,
            self.servo_p_client,
            self.filter_client
        ]
        
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'等待服务 {service.srv_name}...')
    
    def enable_servo_mode(self, robot_id=0):
        """
        使能伺服模式
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT, -1=DUAL
        """
        request = ServoMoveEnable.Request()
        request.enable = True
        request.robot_id = robot_id
        
        future = self.servo_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        self.get_logger().info(f'伺服使能结果: {result.message}')
        return result.success
    
    def disable_servo_mode(self, robot_id=0):
        """
        关闭伺服模式
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT, -1=DUAL
        """
        request = ServoMoveEnable.Request()
        request.enable = False
        request.robot_id = robot_id
        
        future = self.servo_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        self.get_logger().info(f'伺服关闭结果: {result.message}')
        return result.success
    
    def set_filter_none(self):
        """设置不使用滤波器"""
        request = SetServoFilter.Request()
        request.filter_type = 0
        
        future = self.filter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        self.get_logger().info(f'滤波器设置: {result.message}')
        return result.success
    
    def set_filter_lpf(self, cutoff_freq=125.0):
        """
        设置低通滤波器
        
        Args:
            cutoff_freq: 截止频率(Hz)
        """
        request = SetServoFilter.Request()
        request.filter_type = 1
        request.cutoff_freq = cutoff_freq
        
        future = self.filter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        self.get_logger().info(f'LPF滤波器设置: {result.message}')
        return result.success
    
    def servo_j(self, robot_id, joint_positions, move_mode=False):
        """
        伺服关节运动
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT, -1=DUAL
            joint_positions: 14个关节角度(弧度) [左臂7个 + 右臂7个]
            move_mode: False=ABS, True=INCR
        """
        request = ServoJ.Request()
        request.robot_id = robot_id
        request.joint_positions = joint_positions
        request.move_mode = move_mode
        
        future = self.servo_j_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        return result.success
    
    def servo_p(self, robot_id, target_pose, move_mode=False):
        """
        伺服笛卡尔空间运动
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT
            target_pose: 目标位姿
            move_mode: False=ABS, True=INCR
        """
        request = ServoP.Request()
        request.robot_id = robot_id
        request.target_pose = target_pose
        request.move_mode = move_mode
        
        future = self.servo_p_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        
        return result.success


def example_servo_joint_sine_wave():
    """
    示例1: 伺服关节运动 - 正弦波轨迹
    
    演示在伺服模式下控制关节按正弦波运动
    """
    rclpy.init()
    client = JakaServoExample()
    
    try:
        # 1. 设置滤波器
        client.set_filter_lpf(125.0)
        time.sleep(0.5)
        
        # 2. 使能伺服模式
        if not client.enable_servo_mode(0):  # 左臂
            client.get_logger().error('伺服使能失败')
            return
        
        time.sleep(1.0)
        
        # 3. 初始位置 (14个关节: 左臂7个 + 右臂7个)
        initial_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 右臂
        client.servo_j(0, initial_pos)
        time.sleep(2.0)
        
        # 4. 执行正弦波运动(关节1)
        client.get_logger().info('开始正弦波轨迹...')
        amplitude = 0.3  # 振幅(弧度)
        frequency = 0.5  # 频率(Hz)
        duration = 10.0  # 持续时间(秒)
        control_rate = 100  # 控制频率(Hz)
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            t = time.time() - start_time
            
            # 计算正弦波位置
            angle = amplitude * math.sin(2 * math.pi * frequency * t)
            
            # 目标关节位置
            target = initial_pos.copy()
            target[0] = angle
            
            # 发送伺服指令
            client.servo_j(0, target, move_mode=False)
            
            # 控制频率
            time.sleep(1.0 / control_rate)
        
        client.get_logger().info('正弦波轨迹完成')
        
        # 5. 关闭伺服模式
        time.sleep(1.0)
        client.disable_servo_mode(0)
    
    finally:
        rclpy.shutdown()


def example_servo_cartesian_circle():
    """
    示例2: 伺服笛卡尔运动 - 圆形轨迹
    
    演示在伺服模式下控制末端执行器走圆形轨迹
    """
    rclpy.init()
    client = JakaServoExample()
    
    try:
        # 1. 设置不使用滤波器(实时性更好)
        client.set_filter_none()
        time.sleep(0.5)
        
        # 2. 使能伺服模式
        if not client.enable_servo_mode(0):  # 左臂
            client.get_logger().error('伺服使能失败')
            return
        
        time.sleep(1.0)
        
        # 3. 中心位置
        center_pose = Pose()
        center_pose.position.x = 0.5  # 米
        center_pose.position.y = 0.0
        center_pose.position.z = 0.3
        center_pose.orientation.w = 1.0
        
        # 移动到中心位置
        client.servo_p(0, center_pose)
        time.sleep(2.0)
        
        # 4. 执行圆形轨迹
        client.get_logger().info('开始圆形轨迹...')
        radius = 0.05  # 圆半径(米)
        angular_velocity = 0.5  # 角速度(rad/s)
        duration = 2 * math.pi / angular_velocity  # 一圈时间
        control_rate = 100  # 控制频率(Hz)
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration * 2:  # 走两圈
            t = time.time() - start_time
            angle = angular_velocity * t
            
            # 计算圆形轨迹位置
            target_pose = Pose()
            target_pose.position.x = center_pose.position.x + \
                radius * math.cos(angle)
            target_pose.position.y = center_pose.position.y + \
                radius * math.sin(angle)
            target_pose.position.z = center_pose.position.z
            target_pose.orientation.w = 1.0
            
            # 发送伺服指令
            client.servo_p(0, target_pose, move_mode=False)
            
            # 控制频率
            time.sleep(1.0 / control_rate)
        
        client.get_logger().info('圆形轨迹完成')
        
        # 5. 关闭伺服模式
        time.sleep(1.0)
        client.disable_servo_mode(0)
    
    finally:
        rclpy.shutdown()


def example_servo_incremental():
    """
    示例3: 伺服增量运动
    
    演示使用增量模式进行伺服控制
    """
    rclpy.init()
    client = JakaServoExample()
    
    try:
        # 1. 使能伺服模式
        client.set_filter_lpf(125.0)
        time.sleep(0.5)
        
        if not client.enable_servo_mode(0):
            client.get_logger().error('伺服使能失败')
            return
        
        time.sleep(1.0)
        
        # 2. 增量运动 (14个关节)
        client.get_logger().info('开始增量运动...')
        increment = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂第1关节每次增加0.01弧度
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # 右臂不变
        
        for i in range(100):
            # 使用增量模式
            client.servo_j(0, increment, move_mode=True)
            time.sleep(0.01)  # 100Hz
        
        client.get_logger().info('增量运动完成')
        
        # 3. 关闭伺服模式
        time.sleep(1.0)
        client.disable_servo_mode(0)
    
    finally:
        rclpy.shutdown()


def main():
    """主函数"""
    print("JAKA机器人伺服控制示例")
    print("=" * 50)
    print("伺服模式特点:")
    print("  - 高频率控制(通常100-1000Hz)")
    print("  - 低延迟响应")
    print("  - 适合实时轨迹跟踪")
    print("  - 需要周期性发送指令")
    print("=" * 50)
    print("1. 正弦波关节运动")
    print("2. 圆形笛卡尔轨迹")
    print("3. 增量运动示例")
    print("=" * 50)
    
    choice = input("请选择示例 (1-3): ")
    
    if choice == '1':
        example_servo_joint_sine_wave()
    elif choice == '2':
        example_servo_cartesian_circle()
    elif choice == '3':
        example_servo_incremental()
    else:
        print("无效选择")


if __name__ == '__main__':
    main()
