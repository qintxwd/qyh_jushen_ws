#!/usr/bin/env python3
"""
增强版数据录制节点
- 监听录制触发信号
- 同步录制RGB、深度、关节状态
- 自动保存为结构化数据
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import yaml
import os
from datetime import datetime
from pathlib import Path


class DataRecorderEnhanced(Node):
    def __init__(self):
        super().__init__('data_recorder_enhanced')
        
        # 参数
        self.declare_parameter('save_dir', os.path.expanduser('~/demo_recordings'))
        self.declare_parameter('task_name', 'pick_place_upright')
        
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.task_name = self.get_parameter('task_name').get_parameter_value().string_value
        
        # 确保保存目录存在
        Path(self.save_dir).mkdir(parents=True, exist_ok=True)
        
        # 状态
        self.demo_count = self._get_next_demo_number()
        self.frame_count = 0
        self.current_demo_dir = None
        self.bridge = CvBridge()
        self.is_recording = False
        
        # 数据缓冲
        self.rgb_buffer = []
        self.depth_buffer = []
        self.joint_states_buffer = []
        
        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 订阅者
        self.rgb_sub = self.create_subscription(
            Image,
            '/simulation/camera/image_raw',
            self.rgb_callback,
            qos_profile
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/simulation/camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # 录制触发订阅
        self.record_trigger_sub = self.create_subscription(
            Bool,
            '/recording_trigger',
            self.record_trigger_callback,
            10
        )
        
        self.get_logger().info(f'Data Recorder Enhanced initialized')
        self.get_logger().info(f'Save directory: {self.save_dir}')
        self.get_logger().info(f'Next demo number: {self.demo_count}')
    
    def _get_next_demo_number(self):
        """获取下一个演示编号"""
        if not os.path.exists(self.save_dir):
            return 1
        
        existing_demos = [d for d in os.listdir(self.save_dir) if d.startswith('demo_')]
        if not existing_demos:
            return 1
        
        numbers = []
        for demo_dir in existing_demos:
            try:
                num = int(demo_dir.split('_')[1])
                numbers.append(num)
            except:
                pass
        
        return max(numbers) + 1 if numbers else 1
    
    def record_trigger_callback(self, msg):
        """处理录制触发信号"""
        if msg.data and not self.is_recording:
            self.start_recording()
        elif not msg.data and self.is_recording:
            self.stop_recording()
    
    def rgb_callback(self, msg):
        if not self.is_recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.rgb_buffer.append({'timestamp': timestamp, 'frame': cv_image})
        except Exception as e:
            self.get_logger().error(f'RGB error: {e}')
    
    def depth_callback(self, msg):
        if not self.is_recording:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.depth_buffer.append({'timestamp': timestamp, 'frame': cv_image})
        except Exception as e:
            self.get_logger().error(f'Depth error: {e}')
    
    def joint_states_callback(self, msg):
        if not self.is_recording:
            return
        
        joint_data = {
            'timestamp': self.get_clock().now().seconds_nanoseconds()[0] +                         self.get_clock().now().seconds_nanoseconds()[1] * 1e-9,
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'efforts': list(msg.effort) if msg.effort else []
        }
        self.joint_states_buffer.append(joint_data)
    
    def start_recording(self):
        """开始录制"""
        if self.is_recording:
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        demo_name = f'demo_{self.demo_count:04d}_{timestamp}'
        self.current_demo_dir = os.path.join(self.save_dir, demo_name)
        
        Path(self.current_demo_dir).mkdir(parents=True, exist_ok=True)
        Path(os.path.join(self.current_demo_dir, 'rgb')).mkdir(exist_ok=True)
        Path(os.path.join(self.current_demo_dir, 'depth')).mkdir(exist_ok=True)
        
        self.rgb_buffer.clear()
        self.depth_buffer.clear()
        self.joint_states_buffer.clear()
        self.frame_count = 0
        
        self.is_recording = True
        self.get_logger().info(f'🔴 Recording started: {demo_name}')
    
    def stop_recording(self, success=True, comment=''):
        """停止录制并保存"""
        if not self.is_recording:
            return
        
        self.is_recording = False
        self.get_logger().info('⏹️  Saving data...')
        
        # 保存图像
        for i, rgb_data in enumerate(self.rgb_buffer):
            path = os.path.join(self.current_demo_dir, 'rgb', f'frame_{i:05d}.jpg')
            cv2.imwrite(path, rgb_data['frame'])
        
        for i, depth_data in enumerate(self.depth_buffer):
            path = os.path.join(self.current_demo_dir, 'depth', f'frame_{i:05d}.png')
            depth_img = depth_data['frame']
            if depth_img.dtype == np.float32:
                depth_img = (depth_img * 1000).astype(np.uint16)
            cv2.imwrite(path, depth_img)
        
        self.frame_count = len(self.rgb_buffer)
        
        # 保存关节状态
        with open(os.path.join(self.current_demo_dir, 'joint_states.json'), 'w') as f:
            json.dump(self.joint_states_buffer, f, indent=2)
        
        # 保存元数据
        metadata = {
            'task_name': self.task_name,
            'demo_number': self.demo_count,
            'success': success,
            'frame_count': self.frame_count,
            'joint_state_count': len(self.joint_states_buffer),
            'duration': (self.rgb_buffer[-1]['timestamp'] - self.rgb_buffer[0]['timestamp']) 
                       if len(self.rgb_buffer) > 1 else 0
        }
        
        with open(os.path.join(self.current_demo_dir, 'meta.yaml'), 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False)
        
        self.get_logger().info(f'✓ Saved: {self.current_demo_dir}')
        self.demo_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderEnhanced()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.is_recording:
            node.stop_recording(success=False, comment='Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
