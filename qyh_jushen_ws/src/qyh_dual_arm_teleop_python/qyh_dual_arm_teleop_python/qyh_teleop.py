#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========================================================================================
# vr-hand teleop (renamed from qyh_teleop.py)
# ========================================================================================
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
from std_msgs.msg import String


class QyhTeleopNode(Node):
    def __init__(self):
        super().__init__('qyh_teleop')
        
        # 获取参数：arm_side: 'left' 或 'right'
        self.declare_parameter('arm_side', 'left')
        self.arm_side = self.get_parameter('arm_side').get_parameter_value().string_value
        self.get_logger().info(f"[{self.arm_side}]Arm side set to: {self.arm_side}")
        
        # 获取参数，arm_in_base_link_transform: xyz rpy
        self.declare_parameter('arm_in_base_link_transform', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        arm_transform_list = self.get_parameter('arm_in_base_link_transform').get_parameter_value().double_array_value
        self.get_logger().info(f"[{self.arm_side}]Arm in base_link transform: {arm_transform_list}")
        
        # publishers
        self.servo_pub = self.create_publisher(JointState, f'/teleop/{self.arm_side}/servo_p', 10)

        # subscribers
        self.pose_sub = self.create_subscription(PoseStamped, f'/vr/{self.arm_side}_controller/pose', self.vr_pose_callback, 10)
        self.joy_sub = self.create_subscription(Joy, f'/vr/{self.arm_side}_controller/joy', self.joy_callback, 10)
        self.tcp_sub = self.create_subscription(JointState, f'/{self.arm_side}_arm/tcp_pose', self.tcp_pose_callback, 10)

        # clutch thresholds
        self.grip_engage = 0.6
        self.grip_release = 0.3
        
        # state machine
        self.clutch_state = 'IDLE'
        # current values
        self.grip_value = 0.0
        self.cur_tcp_position = [0.0, 0.0, 0.0]
        self.cur_tcp_rpy = [0.0, 0.0, 0.0]
        self.tcp_ready = False
        
        # 变换矩阵
        self.T_base_link_2_arm = self.make_transform_matrix(
            translation=arm_transform_list[0:3],
            rpy=arm_transform_list[3:6]
        )

        self.T_vr_human_align = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.radians(35), 0]
        )
        
        # 右臂镜像修正（语义层，不是物理安装）
        self.T_vr_mirror = np.eye(4)

        # if self.arm_side == 'right':
        #     self.T_vr_mirror[1, 1] = -1.0   # Y 轴镜像
        #     self.get_logger().info("[right] Enable VR semantic mirror (Y axis)")

        # 增量缓存
        self.vr_start_rotm = None
        self.vr_start_position = None
        self.arm_start_rpy = None
        self.arm_start_position = None

        self.get_logger().info(f'[{self.arm_side}]qyh_teleop node started')

    def make_transform_matrix(self, translation, rpy):
        T = np.eye(4)
        T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
        T[:3, 3] = translation
        return T

    def invert_transform(self, T):
        R_inv = T[:3, :3].T
        t_inv = -R_inv @ T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        return T_inv

    def joy_callback(self, msg):
        if hasattr(msg, 'axes') and len(msg.axes) > 3:
            self.grip_value = float(msg.axes[3])

    def tcp_pose_callback(self, msg):
        mm_to_m = 0.001
        try:
            if len(msg.position) >= 10:
                self.cur_tcp_position = [
                    msg.position[0] * mm_to_m,
                    msg.position[1] * mm_to_m,
                    msg.position[2] * mm_to_m
                ]
                
                self.cur_tcp_rpy = [msg.position[7], msg.position[8], msg.position[9]]
                if not self.tcp_ready:
                    self.get_logger().info(f"[{self.arm_side}]TCP ready: pos={[round(x*1000,1) for x in self.cur_tcp_position]}mm")
                self.tcp_ready = True
        except Exception:
            pass

    def print_pose(self, T, label="Pose"):
        pos = T[:3, 3]
        rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        self.get_logger().info(f"[{self.arm_side}]{label} Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        self.get_logger().info(f"[{self.arm_side}]{label} Orientation (rpy in deg): roll={rpy[0]:.1f}, pitch={rpy[1]:.1f}, yaw={rpy[2]:.1f}")
        
    def vr_pose_callback(self, msg):
        grip = self.grip_value
        pressed = grip > self.grip_engage
        released = grip < self.grip_release
                
        if not self.tcp_ready:
            if pressed:
                self.get_logger().warn(f"[{self.arm_side}]TCP pose not ready, ignore clutch")
            return
        
        T_vr_now_base = np.eye(4)
        T_vr_now_base[:3, :3] = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]).as_matrix()
        T_vr_now_base[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        T_vr_now_base = T_vr_now_base @ self.invert_transform(self.T_vr_human_align)
                
        state = self.clutch_state

        if state == 'IDLE':
            if pressed:
                self.clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            try:
                T_vr_in_arm = (
                    self.T_vr_mirror @
                    self.invert_transform(self.T_base_link_2_arm) @
                    T_vr_now_base
                )
                self.vr_start_rotm = T_vr_in_arm[:3, :3].copy()
                self.vr_start_position = T_vr_in_arm[:3, 3].copy()
                self.arm_start_rpy = np.array(self.cur_tcp_rpy).copy()
                self.arm_start_position = np.array(self.cur_tcp_position).copy()

                self.clutch_state = 'TRACKING'
                # 打印T_vr_now_base
                self.print_pose(T_vr_now_base, label="ENGAGING T_vr_now_base")
                self.get_logger().info(
                    f"[{self.arm_side}]▶ TRACKING开始 | VR起点:{[round(x*100,1) for x in self.vr_start_position]}cm | "
                    f"ARM起点:{[round(x*100,1) for x in self.arm_start_position]}cm")
                return
            except Exception as e:
                self.get_logger().error(f"[{self.arm_side}]Error in ENGAGING state: {e}")
                self.clutch_state = 'IDLE'
                return

        if state == 'TRACKING':
            if released:
                self.clutch_state = 'IDLE'
                self.get_logger().info(f"[{self.arm_side}]▶ TRACKING结束 | 已释放离合器，回到IDLE状态")
                return
        
        if self.clutch_state != 'TRACKING':
            return

        if (self.vr_start_rotm is None or self.vr_start_position is None or
            self.arm_start_rpy is None or self.arm_start_position is None):
            return

        if np.isnan(T_vr_now_base).any():
            self.get_logger().error(f"[{self.arm_side}]T_vr_now_base contains NaN")
            return

        try:
            T_vr_current_in_arm = (
                self.T_vr_mirror @
                self.invert_transform(self.T_base_link_2_arm) @
                T_vr_now_base
            )
            vr_current_rotm = T_vr_current_in_arm[:3, :3]
            vr_current_pos = T_vr_current_in_arm[:3, 3]
            delta_rotm_vr = vr_current_rotm @ np.linalg.inv(self.vr_start_rotm)
            arm_start_rotm = R.from_euler('xyz', self.arm_start_rpy).as_matrix()
            target_rotm = delta_rotm_vr @ arm_start_rotm
            target_rpy = R.from_matrix(target_rotm).as_euler('xyz', degrees=False)
            delta_pos = vr_current_pos - self.vr_start_position
            target_position = self.arm_start_position + delta_pos
            out_pose = [
                target_position[0],
                target_position[1],
                target_position[2],
                target_rpy[0],
                target_rpy[1],
                target_rpy[2]
            ]
            delta_rpy_deg = R.from_matrix(delta_rotm_vr).as_euler('xyz', degrees=True)
            self.print_pose(T_vr_now_base, label="TRACKING T_vr_now_base")
            self.get_logger().info(
                f"[{self.arm_side}]Δpos(前左上)=[{delta_pos[0]*100:.1f},{delta_pos[1]*100:.1f},{delta_pos[2]*100:.1f}]cm "
                f"[{self.arm_side}]Δrpy=[{delta_rpy_deg[0]:.1f},{delta_rpy_deg[1]:.1f},{delta_rpy_deg[2]:.1f}]°")

            self.publish_servo_p_command(out_pose)

        except Exception as e:
            self.get_logger().error(f"[{self.arm_side}]Error in rotation matrix calculation: {e}")
            return

    def publish_servo_p_command(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [
            pose[0] * 1000.0,
            pose[1] * 1000.0,
            pose[2] * 1000.0,
            pose[3],
            pose[4],
            pose[5]
        ]

        self.get_logger().info(
            f"[{self.arm_side}]→ servo_p: [{msg.position[0]:.0f},{msg.position[1]:.0f},{msg.position[2]:.0f}]mm "
            f"[{self.arm_side}]→ servo_p: [{np.degrees(msg.position[3]):.1f},{np.degrees(msg.position[4]):.1f},{np.degrees(msg.position[5]):.1f}]°")

        self.servo_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QyhTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
