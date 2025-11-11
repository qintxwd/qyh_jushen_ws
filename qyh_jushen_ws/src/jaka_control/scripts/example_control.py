#!/usr/bin/env python3
"""
JAKA机器人控制示例脚本

演示如何使用jaka_control包控制机器人进行基本操作
"""

import rclpy
from rclpy.node import Node
from jaka_control.srv import (
    PowerOn, EnableRobot, DisableRobot, 
    MoveJ, MoveL, ClearError, MotionAbort,
    SetCollisionLevel
)
from geometry_msgs.msg import Pose
import time


class JakaControlExample(Node):
    """JAKA机器人控制示例类"""
    
    def __init__(self):
        super().__init__('jaka_control_example')
        
        # 创建服务客户端
        self.power_on_client = self.create_client(PowerOn, '/jaka_robot_node/power_on')
        self.enable_client = self.create_client(EnableRobot, '/jaka_robot_node/enable_robot')
        self.disable_client = self.create_client(DisableRobot, '/jaka_robot_node/disable_robot')
        self.move_j_client = self.create_client(MoveJ, '/jaka_robot_node/move_j')
        self.move_l_client = self.create_client(MoveL, '/jaka_robot_node/move_l')
        self.clear_error_client = self.create_client(ClearError, '/jaka_robot_node/clear_error')
        self.abort_client = self.create_client(MotionAbort, '/jaka_robot_node/motion_abort')
        self.collision_client = self.create_client(SetCollisionLevel, '/jaka_robot_node/set_collision_level')
        
        # 等待服务可用
        self.get_logger().info('等待服务可用...')
        self.wait_for_services()
        self.get_logger().info('所有服务已就绪')
    
    def wait_for_services(self):
        """等待所有服务可用"""
        services = [
            self.power_on_client,
            self.enable_client,
            self.disable_client,
            self.move_j_client,
            self.move_l_client,
            self.clear_error_client,
            self.abort_client,
            self.collision_client
        ]
        
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'等待服务 {service.srv_name}...')
    
    def power_on(self):
        """上电"""
        request = PowerOn.Request()
        future = self.power_on_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'上电结果: {result.message}')
        return result.success
    
    def enable_robot(self):
        """使能机器人"""
        request = EnableRobot.Request()
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'使能结果: {result.message}')
        return result.success
    
    def disable_robot(self):
        """下使能机器人"""
        request = DisableRobot.Request()
        future = self.disable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'下使能结果: {result.message}')
        return result.success
    
    def clear_error(self):
        """清除错误"""
        request = ClearError.Request()
        future = self.clear_error_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'清除错误结果: {result.message}')
        return result.success
    
    def set_collision_level(self, robot_id, level):
        """设置碰撞等级"""
        request = SetCollisionLevel.Request()
        request.robot_id = robot_id
        request.level = level
        future = self.collision_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'设置碰撞等级结果: {result.message}')
        return result.success
    
    def move_j(self, robot_id, joint_positions, velocity=0.5, acceleration=1.0, 
               move_mode=False, is_block=True):
        """
        关节空间运动
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT, -1=DUAL
            joint_positions: 14个关节角度(弧度) [左臂7个 + 右臂7个]
            velocity: 速度(rad/s)
            acceleration: 加速度(rad/s^2)
            move_mode: False=ABS(绝对), True=INCR(相对)
            is_block: 是否阻塞等待
        """
        request = MoveJ.Request()
        request.robot_id = robot_id
        request.joint_positions = joint_positions
        request.velocity = velocity
        request.acceleration = acceleration
        request.move_mode = move_mode
        request.is_block = is_block
        
        future = self.move_j_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'关节运动结果: {result.message}')
        return result.success
    
    def move_l(self, robot_id, target_pose, velocity=100.0, acceleration=200.0,
               move_mode=False, is_block=True):
        """
        笛卡尔空间直线运动
        
        Args:
            robot_id: 0=LEFT, 1=RIGHT, -1=DUAL
            target_pose: 目标位姿(Pose)
            velocity: 速度(mm/s)
            acceleration: 加速度(mm/s^2)
            move_mode: False=ABS(绝对), True=INCR(相对)
            is_block: 是否阻塞等待
        """
        request = MoveL.Request()
        request.robot_id = robot_id
        request.target_pose = target_pose
        request.velocity = velocity
        request.acceleration = acceleration
        request.move_mode = move_mode
        request.is_block = is_block
        
        future = self.move_l_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'直线运动结果: {result.message}')
        return result.success
    
    def abort_motion(self):
        """终止运动"""
        request = MotionAbort.Request()
        future = self.abort_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f'终止运动结果: {result.message}')
        return result.success


def example_basic_control():
    """示例1: 基本控制流程"""
    rclpy.init()
    client = JakaControlExample()
    
    try:
        # 1. 清除错误
        client.clear_error()
        time.sleep(0.5)
        
        # 2. 上电
        if client.power_on():
            time.sleep(2)  # 等待上电完成
            
            # 3. 使能
            if client.enable_robot():
                time.sleep(1)
                
                # 4. 设置碰撞等级
                client.set_collision_level(0, 2)  # 左臂
                client.set_collision_level(1, 2)  # 右臂
                
                client.get_logger().info('机器人已准备就绪')
            else:
                client.get_logger().error('使能失败')
        else:
            client.get_logger().error('上电失败')
    
    finally:
        rclpy.shutdown()


def example_joint_motion():
    """示例2: 关节空间运动"""
    rclpy.init()
    client = JakaControlExample()
    
    try:
        # 假设机器人已经上电并使能
        
        # 左臂运动到指定关节位置，右臂保持零位
        # 格式: [左臂7个关节 + 右臂7个关节]
        target_joints = [0.0, 0.5, 0.0, -1.57, 0.0, -0.6, 0.0,  # 左臂
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # 右臂保持零位
        client.get_logger().info('执行关节运动...')
        client.move_j(0, target_joints, velocity=0.3, acceleration=0.5)
        
        # 相对运动: 在当前位置基础上增加
        increment = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂第1关节增加0.1
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 右臂不变
        client.get_logger().info('执行相对运动...')
        client.move_j(0, increment, move_mode=True, velocity=0.2)
    
    finally:
        rclpy.shutdown()


def example_cartesian_motion():
    """示例3: 笛卡尔空间运动"""
    rclpy.init()
    client = JakaControlExample()
    
    try:
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position.x = 0.5  # 米
        target_pose.position.y = 0.0
        target_pose.position.z = 0.3
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        
        # 执行直线运动
        client.get_logger().info('执行笛卡尔空间运动...')
        client.move_l(0, target_pose, velocity=100.0, acceleration=200.0)
        
        # 相对运动: z轴上升50mm
        increment_pose = Pose()
        increment_pose.position.x = 0.0
        increment_pose.position.y = 0.0
        increment_pose.position.z = 0.05  # 米
        increment_pose.orientation.w = 1.0
        
        client.get_logger().info('执行相对运动(上升50mm)...')
        client.move_l(0, increment_pose, move_mode=True, velocity=50.0)
    
    finally:
        rclpy.shutdown()


def example_dual_arm():
    """示例4: 双臂同步运动"""
    rclpy.init()
    client = JakaControlExample()
    
    try:
        # 双臂同步运动到相同的关节位置
        # 注意: robot_id = -1 表示DUAL(双臂)，需要提供14个关节值
        target_joints = [0.0, 0.3, 0.0, -1.2, 0.0, -0.5, 0.0,  # 左臂
                         0.0, 0.3, 0.0, -1.2, 0.0, -0.5, 0.0]   # 右臂(对称运动)
        client.get_logger().info('双臂同步运动...')
        client.move_j(-1, target_joints, velocity=0.3)
    
    finally:
        rclpy.shutdown()


def main():
    """主函数"""
    print("JAKA机器人控制示例")
    print("=" * 50)
    print("1. 基本控制流程")
    print("2. 关节空间运动")
    print("3. 笛卡尔空间运动")
    print("4. 双臂同步运动")
    print("=" * 50)
    
    choice = input("请选择示例 (1-4): ")
    
    if choice == '1':
        example_basic_control()
    elif choice == '2':
        example_joint_motion()
    elif choice == '3':
        example_cartesian_motion()
    elif choice == '4':
        example_dual_arm()
    else:
        print("无效选择")


if __name__ == '__main__':
    main()
