#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HTD-85H 头部舵机控制节点 (ROS2)
支持双自由度头部运动控制 (pan/tilt)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import sys
import os

# 添加 SDK 路径
# current_dir = os.path.dirname(os.path.abspath(__file__))
# sdk_path = os.path.join(current_dir, 'sdk')
# if sdk_path not in sys.path:
#     sys.path.insert(0, sdk_path)

try:
    from qyh_head_control.sdk.hiwonder_servo_controller import HiwonderServoController
except ImportError:
    try:
        # Fallback for local testing or if structure is different
        from sdk.hiwonder_servo_controller import HiwonderServoController
    except ImportError as e:
        print(f"错误：无法导入SDK库: {e}")
        sys.exit(1)


class HeadServoController(Node):
    """头部舵机控制器节点"""
    
    def __init__(self):
        super().__init__('head_servo_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('pan_servo_id', 1)
        self.declare_parameter('tilt_servo_id', 2)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('default_duration', 100)
        
        # Pan舵机范围参数
        self.declare_parameter('pan_min_position', 100)
        self.declare_parameter('pan_max_position', 900)
        self.declare_parameter('pan_center_position', 500)
        
        # Tilt舵机范围参数
        self.declare_parameter('tilt_min_position', 200)
        self.declare_parameter('tilt_max_position', 800)
        self.declare_parameter('tilt_center_position', 500)
        
        # 安全参数
        self.declare_parameter('voltage_min', 6000)
        self.declare_parameter('voltage_max', 12000)
        self.declare_parameter('temp_max', 65)
        self.declare_parameter('enable_safety_check', True)
        
        # 获取参数
        self.port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.pan_servo_id = self.get_parameter('pan_servo_id').value
        self.tilt_servo_id = self.get_parameter('tilt_servo_id').value
        self.update_rate = self.get_parameter('update_rate').value
        self.default_duration = self.get_parameter('default_duration').value
        
        self.pan_min = self.get_parameter('pan_min_position').value
        self.pan_max = self.get_parameter('pan_max_position').value
        self.pan_center = self.get_parameter('pan_center_position').value
        
        self.tilt_min = self.get_parameter('tilt_min_position').value
        self.tilt_max = self.get_parameter('tilt_max_position').value
        self.tilt_center = self.get_parameter('tilt_center_position').value
        
        self.voltage_min = self.get_parameter('voltage_min').value
        self.voltage_max = self.get_parameter('voltage_max').value
        self.temp_max = self.get_parameter('temp_max').value
        self.enable_safety = self.get_parameter('enable_safety_check').value
        
        # 初始化舵机控制器
        try:
            self.servo_control = HiwonderServoController(self.port, self.baudrate)
            self.get_logger().info(f'舵机控制器初始化成功: {self.port}@{self.baudrate}')
        except Exception as e:
            self.get_logger().error(f'舵机控制器初始化失败: {e}')
            sys.exit(1)
        
        # 订阅话题
        self.pan_position_sub = self.create_subscription(
            Float64,
            'head/pan_position',
            self.pan_position_callback,
            10
        )
        
        self.tilt_position_sub = self.create_subscription(
            Float64,
            'head/tilt_position',
            self.tilt_position_callback,
            10
        )
        
        # 订阅归一化位置 (-1.0 到 1.0)
        self.pan_normalized_sub = self.create_subscription(
            Float64,
            'head/pan_normalized',
            self.pan_normalized_callback,
            10
        )
        
        self.tilt_normalized_sub = self.create_subscription(
            Float64,
            'head/tilt_normalized',
            self.tilt_normalized_callback,
            10
        )
        
        # 订阅目标点 (用于视觉跟踪)
        self.target_point_sub = self.create_subscription(
            Point,
            'head/target_point',
            self.target_point_callback,
            10
        )
        
        # 发布话题
        self.joint_state_pub = self.create_publisher(JointState, 'head/joint_states', 10)
        
        # 当前状态
        self.current_pan = self.pan_center
        self.current_tilt = self.tilt_center
        
        # 创建定时器发布状态
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_joint_states)
        
        # 启动时归中
        self.reset_to_center()
        
        self.get_logger().info('头部舵机控制节点已启动')
        self.get_logger().info(f'Pan舵机 ID={self.pan_servo_id}, 范围=[{self.pan_min}, {self.pan_max}]')
        self.get_logger().info(f'Tilt舵机 ID={self.tilt_servo_id}, 范围=[{self.tilt_min}, {self.tilt_max}]')
    
    def reset_to_center(self):
        """归中位"""
        try:
            self.servo_control.set_servo_position(self.pan_servo_id, self.pan_center, 500)
            self.servo_control.set_servo_position(self.tilt_servo_id, self.tilt_center, 500)
            self.current_pan = self.pan_center
            self.current_tilt = self.tilt_center
            self.get_logger().info('头部已归中')
        except Exception as e:
            self.get_logger().error(f'归中失败: {e}')
    
    def check_safety(self, servo_id):
        """安全检查"""
        if not self.enable_safety:
            return True
        
        try:
            # 检查电压
            voltage = self.servo_control.get_servo_vin(servo_id)
            if voltage < self.voltage_min or voltage > self.voltage_max:
                self.get_logger().warn(f'舵机{servo_id}电压异常: {voltage}mV')
                return False
            
            # 检查温度
            temp = self.servo_control.get_servo_temp(servo_id)
            if temp > self.temp_max:
                self.get_logger().error(f'舵机{servo_id}温度过高: {temp}℃')
                return False
            
            return True
        except Exception as e:
            self.get_logger().warn(f'安全检查失败: {e}')
            return True  # 检查失败不阻止运动
    
    def pan_position_callback(self, msg):
        """Pan位置控制 (0-1000)"""
        position = int(msg.data)
        position = max(self.pan_min, min(self.pan_max, position))
        
        if self.check_safety(self.pan_servo_id):
            try:
                self.servo_control.set_servo_position(
                    self.pan_servo_id,
                    position,
                    self.default_duration
                )
                self.current_pan = position
            except Exception as e:
                self.get_logger().error(f'Pan舵机控制错误: {e}')
    
    def tilt_position_callback(self, msg):
        """Tilt位置控制 (0-1000)"""
        position = int(msg.data)
        position = max(self.tilt_min, min(self.tilt_max, position))
        
        if self.check_safety(self.tilt_servo_id):
            try:
                self.servo_control.set_servo_position(
                    self.tilt_servo_id,
                    position,
                    self.default_duration
                )
                self.current_tilt = position
            except Exception as e:
                self.get_logger().error(f'Tilt舵机控制错误: {e}')
    
    def pan_normalized_callback(self, msg):
        """Pan归一化控制 (-1.0 到 1.0)"""
        normalized = max(-1.0, min(1.0, msg.data))
        position = int(self.pan_center + normalized * (self.pan_max - self.pan_center))
        position = max(self.pan_min, min(self.pan_max, position))
        
        if self.check_safety(self.pan_servo_id):
            try:
                self.servo_control.set_servo_position(
                    self.pan_servo_id,
                    position,
                    self.default_duration
                )
                self.current_pan = position
            except Exception as e:
                self.get_logger().error(f'Pan舵机控制错误: {e}')
    
    def tilt_normalized_callback(self, msg):
        """Tilt归一化控制 (-1.0 到 1.0)"""
        normalized = max(-1.0, min(1.0, msg.data))
        position = int(self.tilt_center + normalized * (self.tilt_max - self.tilt_center))
        position = max(self.tilt_min, min(self.tilt_max, position))
        
        if self.check_safety(self.tilt_servo_id):
            try:
                self.servo_control.set_servo_position(
                    self.tilt_servo_id,
                    position,
                    self.default_duration
                )
                self.current_tilt = position
            except Exception as e:
                self.get_logger().error(f'Tilt舵机控制错误: {e}')
    
    def target_point_callback(self, msg):
        """
        目标点跟踪控制
        msg.x: 水平方向 (-1.0=左, 1.0=右)
        msg.y: 垂直方向 (-1.0=下, 1.0=上)
        msg.z: 未使用
        """
        # 转换为舵机位置
        pan_pos = int(self.pan_center - msg.x * (self.pan_max - self.pan_center))
        tilt_pos = int(self.tilt_center + msg.y * (self.tilt_max - self.tilt_center))
        
        pan_pos = max(self.pan_min, min(self.pan_max, pan_pos))
        tilt_pos = max(self.tilt_min, min(self.tilt_max, tilt_pos))
        
        if self.check_safety(self.pan_servo_id) and self.check_safety(self.tilt_servo_id):
            try:
                self.servo_control.set_servo_position(self.pan_servo_id, pan_pos, self.default_duration)
                self.servo_control.set_servo_position(self.tilt_servo_id, tilt_pos, self.default_duration)
                self.current_pan = pan_pos
                self.current_tilt = tilt_pos
            except Exception as e:
                self.get_logger().error(f'目标跟踪控制错误: {e}')
    
    def publish_joint_states(self):
        """发布关节状态"""
        try:
            # 读取实际位置
            pan_pos = self.servo_control.get_servo_position(self.pan_servo_id)
            tilt_pos = self.servo_control.get_servo_position(self.tilt_servo_id)
            
            if pan_pos is not None and tilt_pos is not None:
                # 转换为弧度 (0-1000 映射到 -π/2 到 π/2)
                pan_angle = (pan_pos - self.pan_center) / (self.pan_max - self.pan_center) * 1.5708
                tilt_angle = (tilt_pos - self.tilt_center) / (self.tilt_max - self.tilt_center) * 1.5708
                
                # 构建消息
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['head_pan_joint', 'head_tilt_joint']
                joint_state.position = [pan_angle, tilt_angle]
                
                self.joint_state_pub.publish(joint_state)
        except Exception as e:
            self.get_logger().warn(f'读取关节状态失败: {e}', throttle_duration_sec=5.0)
    
    def shutdown(self):
        """关闭时归中"""
        self.get_logger().info('节点关闭，头部归中...')
        try:
            self.servo_control.set_servo_position(self.pan_servo_id, self.pan_center, 500)
            self.servo_control.set_servo_position(self.tilt_servo_id, self.tilt_center, 500)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HeadServoController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点异常: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
