#!/usr/bin/env python3
"""
VR数据接收并发布到base_vr坐标系

接收PICO 4 VR的UDP数据，将手柄位置直接发布为base_vr坐标系下的子frame
TF树: base_vr -> vr_left
      base_vr -> vr_right

VR手柄数据直接对应base_vr坐标系（X右 Y上 Z后）
"""

import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import threading
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml
from pathlib import Path


class VRToBaseVRNode(Node):
    """接收VR数据并发布到base_vr坐标系下"""
    
    # UDP数据包结构 (与C++代码一致)
    # 总大小: 8 + 28 + 53 + 53 + 8 = 150 bytes
    PACKET_FORMAT = '<q'  # timestamp (int64)
    PACKET_FORMAT += '3f 4f'  # head: position(3 float), orientation(4 float)
    PACKET_FORMAT += 'B 3f 4f 2f f f'  # left: active(uint8), pos, ori, joy, trigger, grip
    PACKET_FORMAT += 'B 3f 4f 2f f f'  # right: active(uint8), pos, ori, joy, trigger, grip
    PACKET_FORMAT += '2I'  # buttons_bitmask, touches_bitmask (2 uint32)
    
    PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
    
    def __init__(self):
        super().__init__('vr_to_base_vr_node')
        
        # 参数
        self.declare_parameter('udp_port', 9999)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('grip_offset_deg', 35.0)  # 握持补偿角度(pitch)
        self.declare_parameter('flip_y_axis', True)  # 是否翻转Y轴180度
        
        # 位置映射参数: P' = scale * P + offset
        # 左手映射参数
        self.declare_parameter('left_scale_x', 1.0)
        self.declare_parameter('left_scale_y', 1.0)
        self.declare_parameter('left_scale_z', 1.0)
        self.declare_parameter('left_offset_x', 0.0)
        self.declare_parameter('left_offset_y', 0.0)
        self.declare_parameter('left_offset_z', 0.0)
        
        # 右手映射参数
        self.declare_parameter('right_scale_x', 1.0)
        self.declare_parameter('right_scale_y', 1.0)
        self.declare_parameter('right_scale_z', 1.0)
        self.declare_parameter('right_offset_x', 0.0)
        self.declare_parameter('right_offset_y', 0.0)
        self.declare_parameter('right_offset_z', 0.0)
        
        self.udp_port = self.get_parameter('udp_port').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.grip_offset_deg = self.get_parameter('grip_offset_deg').value
        self.flip_y_axis = self.get_parameter('flip_y_axis').value
        
        # 尝试从标定文件加载映射参数
        config_file = Path(__file__).parent.parent / 'config' / 'vr_mapping_calibration.yaml'
        if config_file.exists():
            self.get_logger().info(f'加载标定文件: {config_file}')
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            calib = config.get('vr_mapping_calibration', {})
            left_cfg = calib.get('left', {})
            right_cfg = calib.get('right', {})
            
            # 从文件加载，覆盖参数
            if left_cfg:
                self.declare_parameter('left_scale_x', left_cfg['scale']['x'])
                self.declare_parameter('left_scale_y', left_cfg['scale']['y'])
                self.declare_parameter('left_scale_z', left_cfg['scale']['z'])
                self.declare_parameter('left_offset_x', left_cfg['offset']['x'])
                self.declare_parameter('left_offset_y', left_cfg['offset']['y'])
                self.declare_parameter('left_offset_z', left_cfg['offset']['z'])
            
            if right_cfg:
                self.declare_parameter('right_scale_x', right_cfg['scale']['x'])
                self.declare_parameter('right_scale_y', right_cfg['scale']['y'])
                self.declare_parameter('right_scale_z', right_cfg['scale']['z'])
                self.declare_parameter('right_offset_x', right_cfg['offset']['x'])
                self.declare_parameter('right_offset_y', right_cfg['offset']['y'])
                self.declare_parameter('right_offset_z', right_cfg['offset']['z'])
            
            self.get_logger().info('✓ 标定参数已加载')
        else:
            self.get_logger().warn(f'未找到标定文件: {config_file}')
            self.get_logger().warn('使用默认映射参数 (scale=1.0, offset=0.0)')
            self.get_logger().warn('运行 calibrate_vr_mapping.py 进行标定')
        
        # 加载位置映射参数
        self.left_scale = np.array([
            self.get_parameter('left_scale_x').value,
            self.get_parameter('left_scale_y').value,
            self.get_parameter('left_scale_z').value
        ])
        self.left_offset = np.array([
            self.get_parameter('left_offset_x').value,
            self.get_parameter('left_offset_y').value,
            self.get_parameter('left_offset_z').value
        ])
        
        self.right_scale = np.array([
            self.get_parameter('right_scale_x').value,
            self.get_parameter('right_scale_y').value,
            self.get_parameter('right_scale_z').value
        ])
        self.right_offset = np.array([
            self.get_parameter('right_offset_x').value,
            self.get_parameter('right_offset_y').value,
            self.get_parameter('right_offset_z').value
        ])
        
        # 预计算旋转补偿（明确顺序：先握持补偿，后翻转）
        # 工具坐标系固定变换
        # "先绕x轴旋转35°，然后绕着旋转后的y轴旋转180°"
        # 父坐标系: vr_left/vr_right (原始数据)
        # 子坐标系: vr_left_tool/vr_right_tool
        # 使用参数 grip_offset_deg 作为 X 轴旋转角度
        
        # 左手: -35度 (根据实际测试调整)
        self.left_tool_rot = R.from_euler('xy', [self.grip_offset_deg, 180.0], degrees=True)
        self.left_tool_quat = self.left_tool_rot.as_quat()

        # 右手: +35度
        self.right_tool_rot = R.from_euler('xy', [self.grip_offset_deg, 180.0], degrees=True)
        self.right_tool_quat = self.right_tool_rot.as_quat()
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 可选：发布PoseStamped（便于调试）
        self.left_pose_pub = self.create_publisher(
            PoseStamped, '/vr_base_vr/left_pose', 10)
        self.right_pose_pub = self.create_publisher(
            PoseStamped, '/vr_base_vr/right_pose', 10)
        
        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(0.1)  # 100ms超时
        
        # 最新的VR数据
        self.latest_data = None
        self.data_lock = threading.Lock()
        
        # 启动接收线程
        self.running = True
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()
        
        # 定时发布TF
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_callback)
        
        self.get_logger().info(f'VR to base_vr node started')
        self.get_logger().info(f'Listening on UDP port {self.udp_port}')
        self.get_logger().info(f'Flip Y-axis: {self.flip_y_axis} (180 deg)')
        self.get_logger().info(f'Grip offset: {self.grip_offset_deg:.1f} deg (pitch)')
        total_angle = (180.0 + self.grip_offset_deg) if self.flip_y_axis else self.grip_offset_deg
        self.get_logger().info(f'Total rotation: {total_angle:.1f} deg around Y-axis')
        self.get_logger().info(f'Expected packet size: {self.PACKET_SIZE} bytes')
        self.get_logger().info(f'Left mapping: scale={self.left_scale}, offset={self.left_offset}')
        self.get_logger().info(f'Right mapping: scale={self.right_scale}, offset={self.right_offset}')
    
    def _receive_loop(self):
        """UDP接收线程"""
        self.get_logger().info('UDP receive thread started')
        
        while self.running and rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(4096)
                
                if len(data) == self.PACKET_SIZE:
                    parsed_data = self._parse_packet(data)
                    if parsed_data:
                        with self.data_lock:
                            self.latest_data = parsed_data
                        self.get_logger().info(
                            f"UDP Recv: Left={parsed_data['left']['active']} Right={parsed_data['right']['active']}", 
                            throttle_duration_sec=2.0)
                elif len(data) > 0:
                    self.get_logger().warn(
                        f'Received packet size {len(data)}, expected {self.PACKET_SIZE}',
                        throttle_duration_sec=5.0)
                        
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP receive error: {e}')
    
    def _parse_packet(self, data):
        """解析UDP数据包"""
        try:
            unpacked = struct.unpack(self.PACKET_FORMAT, data)
            idx = 0
            
            # Timestamp
            timestamp = unpacked[idx]
            idx += 1
            
            # Head (跳过，我们不需要)
            idx += 3 + 4  # position + orientation
            
            # Left controller
            left_active = unpacked[idx] != 0
            idx += 1
            left_pos = unpacked[idx:idx+3]
            idx += 3
            left_ori = unpacked[idx:idx+4]  # x, y, z, w
            idx += 4
            left_joy = unpacked[idx:idx+2]
            idx += 2
            left_trigger = unpacked[idx]
            idx += 1
            left_grip = unpacked[idx]
            idx += 1
            
            # Right controller
            right_active = unpacked[idx] != 0
            idx += 1
            right_pos = unpacked[idx:idx+3]
            idx += 3
            right_ori = unpacked[idx:idx+4]  # x, y, z, w
            idx += 4
            right_joy = unpacked[idx:idx+2]
            idx += 2
            right_trigger = unpacked[idx]
            idx += 1
            right_grip = unpacked[idx]
            idx += 1
            
            # Buttons (我们不需要)
            
            return {
                'timestamp': timestamp,
                'left': {
                    'active': left_active,
                    'position': left_pos,
                    'orientation': left_ori,
                },
                'right': {
                    'active': right_active,
                    'position': right_pos,
                    'orientation': right_ori,
                }
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse packet: {e}')
            return None
    
    def _publish_callback(self):
        """定时发布TF"""
        with self.data_lock:
            if self.latest_data is None:
                self.get_logger().warn("Waiting for first VR packet...", throttle_duration_sec=2.0)
                return
            data = self.latest_data
        
        now = self.get_clock().now().to_msg()
        transforms = []
        
        # 发布左手柄: base_vr -> vr_left
        if data['left']['active']:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_vr'
            t.child_f经过映射变换后作为base_vr坐标系下的位置
            # 公式: P' = scale * P + offset
            vr_pos = np.array([
                data['left']['position'][0],
                data['left']['position'][1],
                data['left']['position'][2]
            ])
            mapped_pos = self.left_scale * vr_pos + self.left_offset
            
            t.transform.translation.x = float(mapped_pos[0])
            t.transform.translation.y = float(mapped_pos[1])
            t.transform.translation.z = float(mapped_pos[0])
            t.transform.translation.y = float(data['left']['position'][1])
            t.transform.translation.z = float(data['left']['position'][2])
            
            # 四元数 (x, y, z, w) - 原始数据
            vr_quat = R.from_quat([
                data['left']['orientation'][0],
                data['left']['orientation'][1],
                data['left']['orientation'][2],
                data['left']['orientation'][3]
            ])
            q = vr_quat.as_quat()
            
            # Debug Log
            r_euler = vr_quat.as_euler('xyz', degrees=True)
            self.get_logger().info(
                f"[LEFT] RawPos:({data['left']['position'][0]:.2f}, {data['left']['position'][1]:.2f}, {data['left']['position'][2]:.2f}) "
                f"Euler:{r_euler}", throttle_duration_sec=0.5)

            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            
            transforms.append(t)

            # 发布 vr_left -> vr_left_tool
            t_tool = TransformStamped()
            t_tool.header.stamp = now
            t_tool.header.frame_id = 'vr_left'
            t_tool.child_frame_id = 'vr_left_tool'
            t_tool.transform.rotation.x = float(self.left_tool_quat[0])
            t_tool.transform.rotation.y = float(self.left_tool_quat[1])
            t_tool.transform.rotation.z = float(self.left_tool_quat[2])
            t_tool.transform.rotation.w = float(self.left_tool_quat[3])
            transforms.append(t_tool)
            
            # 可选：发布PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            self.left_pose_pub.publish(pose_msg)
        
        # 发布右手柄: base经过映射变换后作为base_vr坐标系下的位置
            # 公式: P' = scale * P + offset
            vr_pos = np.array([
                data['right']['position'][0],
                data['right']['position'][1],
                data['right']['position'][2]
            ])
            mapped_pos = self.right_scale * vr_pos + self.right_offset
            
            t.transform.translation.x = float(mapped_pos[0])
            t.transform.translation.y = float(mapped_pos[1])
            t.transform.translation.z = float(mapped_pos
            t.header.frame_id = 'base_vr'
            t.child_frame_id = 'vr_right'

            # VR手柄的位置直接作为base_vr坐标系下的位置
            t.transform.translation.x = float(data['right']['position'][0])
            t.transform.translation.y = float(data['right']['position'][1])
            t.transform.translation.z = float(data['right']['position'][2])

            # 四元数 (x, y, z, w) - 原始数据
            vr_quat = R.from_quat([
                data['right']['orientation'][0],
                data['right']['orientation'][1],
                data['right']['orientation'][2],
                data['right']['orientation'][3]
            ])
            q = vr_quat.as_quat()
            
            # Debug Log
            r_euler = vr_quat.as_euler('xyz', degrees=True)
            self.get_logger().info(
                f"[RIGHT] RawPos:({data['right']['position'][0]:.2f}, {data['right']['position'][1]:.2f}, {data['right']['position'][2]:.2f}) "
                f"Euler:{r_euler}", throttle_duration_sec=0.5)

            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            
            transforms.append(t)

            # 发布 vr_right -> vr_right_tool
            t_tool = TransformStamped()
            t_tool.header.stamp = now
            t_tool.header.frame_id = 'vr_right'
            t_tool.child_frame_id = 'vr_right_tool'
            t_tool.transform.rotation.x = float(self.right_tool_quat[0])
            t_tool.transform.rotation.y = float(self.right_tool_quat[1])
            t_tool.transform.rotation.z = float(self.right_tool_quat[2])
            t_tool.transform.rotation.w = float(self.right_tool_quat[3])
            transforms.append(t_tool)
            
            # 可选：发布PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            self.right_pose_pub.publish(pose_msg)
        
        # 批量发布所有TF
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
    
    def destroy_node(self):
        """清理资源"""
        self.running = False
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        if hasattr(self, 'sock'):
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRToBaseVRNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
