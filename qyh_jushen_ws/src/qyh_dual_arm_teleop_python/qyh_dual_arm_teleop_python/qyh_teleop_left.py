#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========================================================================================
# Left-hand teleop (renamed from qyh_teleop.py)
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
        super().__init__('qyh_teleop_left')

        # publishers
        self.left_servo_pub = self.create_publisher(JointState, '/teleop/left/servo_p', 10)

        # subscribers
        self.left_pose_sub = self.create_subscription(PoseStamped, '/vr/left_controller/pose', self.vr_left_pose_callback, 10)
        self.left_joy_sub = self.create_subscription(Joy, '/vr/left_controller/joy', self.left_joy_callback, 10)
        self.left_tcp_sub = self.create_subscription(JointState, '/left_arm/tcp_pose', self.left_tcp_pose_callback, 10)

        # clutch thresholds
        self.grip_engage = 0.6
        self.grip_release = 0.3
        
        # state machine
        self.left_clutch_state = 'IDLE'
        # current values
        self.left_grip_value_ = 0.0
        self.cur_tcp_position = [0.0, 0.0, 0.0]
        self.cur_tcp_rotm = np.eye(3)
        self.cur_tcp_rpy = [0.0, 0.0, 0.0]
        self.tcp_ready = False
        
        # base_link -> base_link_left 变换 (包含-30° yaw 和 xyz 偏移)
        # 从 URDF: <joint name="base_to_left" origin xyz="-0.0004 0.08522 0.0030" rpy="0 0 -0.5236" />
        self.T_base_base_left = self.make_transform_matrix(
            translation=[-0.0004, 0.08522, 0.0030],
            rpy=[0, 0, -math.pi / 6]
        )

        # lt -> forward_lt
        self.T_lt_forward = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.pi / 2, -math.pi / 2]
        )

        # VR->arm alignment matrix (vector mapping)
        self.R_vr_to_arm = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0]
        ])

        # Pico手柄 → 人体手势坐标系
        self.T_vr_human_align = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.radians(35), 0]
        )

        # 增量缓存
        self.vr_start_rotm = None
        self.vr_start_position = None
        self.arm_start_rpy = None
        self.arm_start_position = None

        self.get_logger().info('qyh_teleop_left node started')

        # 调试
        self.debug_coordinate_alignment()
        self.debug_rpy_incremental()

    def debug_coordinate_alignment(self):
        self.get_logger().info("=== Coordinate Alignment Debug ===")
        vr_x_axis = np.array([1, 0, 0])
        arm_x_from_vr_x = self.R_vr_to_arm @ vr_x_axis
        self.get_logger().info(f"VR x轴(前) -> 机械臂坐标: {arm_x_from_vr_x} (期望: [0,0,1] 机械臂z轴)")
        vr_y_axis = np.array([0, 1, 0])
        arm_y_from_vr_y = self.R_vr_to_arm @ vr_y_axis
        self.get_logger().info(f"VR y轴(左) -> 机械臂坐标: {arm_y_from_vr_y} (期望: [1,0,0] 机械臂x轴)")
        vr_z_axis = np.array([0, 0, 1])
        arm_z_from_vr_z = self.R_vr_to_arm @ vr_z_axis
        self.get_logger().info(f"VR z轴(上) -> 机械臂坐标: {arm_z_from_vr_z} (期望: [0,1,0] 机械臂y轴)")
        self.get_logger().info("=== Coordinate Debug Complete ===")

    def debug_rpy_incremental(self):
        self.get_logger().info("=== RPY Incremental Debug ===")
        vr_start_rpy = np.array([0, 0, 0])
        vr_current_rpy = np.array([np.radians(30), 0, 0])
        delta_rpy_raw = vr_current_rpy - vr_start_rpy
        self.get_logger().info(f"VR roll+30° -> delta_rpy_raw: {np.degrees(delta_rpy_raw)}°")
        delta_rpy_aligned = self.R_vr_to_arm @ delta_rpy_raw
        self.get_logger().info(f"坐标系对齐后: delta_rpy_aligned: {np.degrees(delta_rpy_aligned)}°")
        arm_start_rpy = np.array([np.radians(10), np.radians(20), np.radians(30)])
        target_rpy = arm_start_rpy + delta_rpy_aligned
        self.get_logger().info(f"机械臂起始RPY: {np.degrees(arm_start_rpy)}°")
        self.get_logger().info(f"目标RPY: {np.degrees(target_rpy)}°")
        self.get_logger().info("=== RPY Debug Complete ===")

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

    def rotation_matrix_z(self, angle_deg):
        angle_rad = math.radians(angle_deg)
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

    def choose_rpy_closest(self, rpy_prev, rotm):
        prev = np.asarray(rpy_prev, dtype=float)
        Rmat = np.asarray(rotm, dtype=float)
        EPS = 1e-6
        z_axis = Rmat[:, 2]
        if abs(z_axis[2] - 1.0) < EPS:
            return np.array([0.0, 0.0, prev[2]])
        if abs(z_axis[2] + 1.0) < EPS:
            return np.array([math.pi, 0.0, prev[2]])
        base = R.from_matrix(Rmat).as_euler('xyz', degrees=False)
        candidates = [base, np.array([base[0] + math.pi, math.pi - base[1], base[2] + math.pi])]
        TWO_PI = 2.0 * math.pi
        expanded = []
        for c0 in candidates:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        c = c0 + np.array([dx * TWO_PI, dy * TWO_PI, dz * TWO_PI])
                        for k in range(3):
                            while c[k] - prev[k] > math.pi:
                                c[k] -= TWO_PI
                            while c[k] - prev[k] < -math.pi:
                                c[k] += TWO_PI
                        expanded.append(c)
        best = expanded[0]
        best_dist = np.sum((best - prev) ** 2)
        for c in expanded:
            d = np.sum((c - prev) ** 2)
            if d < best_dist:
                best = c
                best_dist = d
        best[2] = math.atan2(math.sin(best[2]), math.cos(best[2]))
        return best

    def left_joy_callback(self, msg):
        if hasattr(msg, 'axes') and len(msg.axes) > 3:
            self.left_grip_value_ = float(msg.axes[3])

    def left_tcp_pose_callback(self, msg):
        mm_to_m = 0.001
        try:
            if len(msg.position) >= 10:
                self.cur_tcp_position = [
                    msg.position[0] * mm_to_m,
                    msg.position[1] * mm_to_m,
                    msg.position[2] * mm_to_m
                ]
                q = [
                    msg.position[4],
                    msg.position[5],
                    msg.position[6],
                    msg.position[3]
                ]
                self.cur_tcp_rotm = R.from_quat(q).as_matrix()
                self.cur_tcp_rpy = [msg.position[7], msg.position[8], msg.position[9]]
                if not self.tcp_ready:
                    self.get_logger().info(f"TCP ready: pos={[round(x*1000,1) for x in self.cur_tcp_position]}mm")
                self.tcp_ready = True
        except Exception:
            pass

    def print_pose(self, T, label="Pose"):
        pos = T[:3, 3]
        rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
        self.get_logger().info(f"{label} Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
        self.get_logger().info(f"{label} Orientation (rpy in deg): roll={rpy[0]:.1f}, pitch={rpy[1]:.1f}, yaw={rpy[2]:.1f}")
        
    def vr_left_pose_callback(self, msg):
        grip = self.left_grip_value_
        pressed = grip > self.grip_engage
        released = grip < self.grip_release
                
        if not self.tcp_ready:
            self.get_logger().warn("TCP pose not ready, ignore clutch")
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
                
        state = self.left_clutch_state

        if state == 'IDLE':
            if pressed:
                self.left_clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            try:
                T_vr_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_now_base
                self.vr_start_rotm = T_vr_in_left[:3, :3].copy()
                self.vr_start_position = T_vr_in_left[:3, 3].copy()
                self.arm_start_rpy = np.array(self.cur_tcp_rpy).copy()
                self.arm_start_position = np.array(self.cur_tcp_position).copy()

                self.left_clutch_state = 'TRACKING'
                self.get_logger().info(
                    f"▶ TRACKING开始 | VR起点:{[round(x*100,1) for x in self.vr_start_position]}cm | "
                    f"ARM起点:{[round(x*100,1) for x in self.arm_start_position]}cm")
                return
            except Exception as e:
                self.get_logger().error(f"Error in ENGAGING state: {e}")
                self.left_clutch_state = 'IDLE'
                return

        if state == 'TRACKING':
            if released:
                self.left_clutch_state = 'IDLE'
                self.get_logger().info('[LEFT] Clutch RELEASED')
                return
        
        if self.left_clutch_state != 'TRACKING':
            return

        if (self.vr_start_rotm is None or self.vr_start_position is None or
            self.arm_start_rpy is None or self.arm_start_position is None):
            return

        if np.isnan(T_vr_now_base).any():
            self.get_logger().error("T_vr_now_base contains NaN")
            return

        try:
            T_vr_current_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_now_base
            vr_current_rotm = T_vr_current_in_left[:3, :3]
            vr_current_pos = T_vr_current_in_left[:3, 3]
            delta_rotm_vr = vr_current_rotm @ np.linalg.inv(self.vr_start_rotm)
            delta_rotm_lt = delta_rotm_vr
            arm_start_rotm = R.from_euler('xyz', self.arm_start_rpy).as_matrix()
            target_rotm = delta_rotm_lt @ arm_start_rotm
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
            self.get_logger().info(
                f"Δpos(前左上)=[{delta_pos[0]*100:.1f},{delta_pos[1]*100:.1f},{delta_pos[2]*100:.1f}]cm "
                f"Δrpy=[{delta_rpy_deg[0]:.1f},{delta_rpy_deg[1]:.1f},{delta_rpy_deg[2]:.1f}]°")

            self.publish_servo_p_command(out_pose)

        except Exception as e:
            self.get_logger().error(f"Error in rotation matrix calculation: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
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
            pose[5],
            0.0
        ]

        self.get_logger().info(
            f"→ servo_p: [{msg.position[0]:.0f},{msg.position[1]:.0f},{msg.position[2]:.0f}]mm "
            f"[{np.degrees(msg.position[3]):.1f},{np.degrees(msg.position[4]):.1f},{np.degrees(msg.position[5]):.1f}]°")

        self.left_servo_pub.publish(msg)

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
