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
        
        self.udp_port = self.get_parameter('udp_port').as_int()
        self.publish_rate = self.get_parameter('publish_rate').as_double()
        
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
        self.get_logger().info(f'Expected packet size: {self.PACKET_SIZE} bytes')
    
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
                return
            data = self.latest_data
        
        now = self.get_clock().now().to_msg()
        transforms = []
        
        # 发布左手柄: base_vr -> vr_left
        if data['left']['active']:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_vr'
            t.child_frame_id = 'vr_left'
            
            # VR手柄的位置直接作为base_vr坐标系下的位置
            # VR: X右 Y上 Z后，正好就是base_vr的定义
            t.transform.translation.x = float(data['left']['position'][0])
            t.transform.translation.y = float(data['left']['position'][1])
            t.transform.translation.z = float(data['left']['position'][2])
            
            # 四元数 (x, y, z, w)
            t.transform.rotation.x = float(data['left']['orientation'][0])
            t.transform.rotation.y = float(data['left']['orientation'][1])
            t.transform.rotation.z = float(data['left']['orientation'][2])
            t.transform.rotation.w = float(data['left']['orientation'][3])
            
            transforms.append(t)
            
            # 可选：发布PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = t.header
            pose_msg.pose.position.x = t.transform.translation.x
            pose_msg.pose.position.y = t.transform.translation.y
            pose_msg.pose.position.z = t.transform.translation.z
            pose_msg.pose.orientation = t.transform.rotation
            self.left_pose_pub.publish(pose_msg)
        
        # 发布右手柄: base_vr -> vr_right
        if data['right']['active']:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_vr'
            t.child_frame_id = 'vr_right'
            
            # VR手柄的位置直接作为base_vr坐标系下的位置
            t.transform.translation.x = float(data['right']['position'][0])
            t.transform.translation.y = float(data['right']['position'][1])
            t.transform.translation.z = float(data['right']['position'][2])
            
            # 四元数 (x, y, z, w)
            t.transform.rotation.x = float(data['right']['orientation'][0])
            t.transform.rotation.y = float(data['right']['orientation'][1])
            t.transform.rotation.z = float(data['right']['orientation'][2])
            t.transform.rotation.w = float(data['right']['orientation'][3])
            
            transforms.append(t)
            
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
