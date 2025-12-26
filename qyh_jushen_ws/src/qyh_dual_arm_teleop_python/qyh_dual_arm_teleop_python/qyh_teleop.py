#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
from std_msgs.msg import String


# def make_transform_matrix(translation, rpy):
#     """根据平移和欧拉角生成 4x4 变换矩阵"""
#     t = np.eye(4)
#     t[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
#     t[:3, 3] = translation
#     return t

# def invert_transform(T):
#     """4x4 变换矩阵求逆"""
#     R_inv = T[:3, :3].T
#     t_inv = -R_inv @ T[:3, 3]
#     T_inv = np.eye(4)
#     T_inv[:3, :3] = R_inv
#     T_inv[:3, 3] = t_inv
#     return T_inv

# # -----------------------
# # 已知变换
# # -----------------------
# # VR在base_link下
# T_vr_base = make_transform_matrix(
#     translation=[0.5, 0.2, 1.0],   # 例子
#     rpy=[0, 0, 0]
# )

# # base_link -> base_link_left (固定-30° yaw)
# T_base_base_left = make_transform_matrix(
#     translation=[0, 0, 0], 
#     rpy=[0, 0, -np.pi/6]
# )

# # lt -> forward_lt (固定旋转)
# T_lt_forward = make_transform_matrix(
#     translation=[0,0,0],
#     rpy=[0, -np.pi/2, -np.pi/2]
# )

# # 当前机械臂读取到的 lt 位姿 (在 base_link_left 下)
# T_lt_current = make_transform_matrix(
#     translation=[0.6, 0.0, 0.8],   # 例子
#     rpy=[0, 0, 0]
# )

# # -----------------------
# # 计算逻辑
# # -----------------------

# # 1️⃣ VR 转到 base_link_left
# T_vr_left = invert_transform(T_base_base_left) @ T_vr_base

# # 2️⃣ 对齐到 forward_lt
# T_vr_forward_aligned = T_vr_left @ invert_transform(T_lt_forward)

# # 3️⃣ 当前 forward_lt 位姿
# T_forward_current = T_lt_current @ T_lt_forward

# # 4️⃣ 计算增量 Δ
# delta_T = T_vr_forward_aligned @ invert_transform(T_forward_current)

# # 5️⃣ 计算目标 forward_lt
# T_forward_target = delta_T @ T_forward_current

# # 6️⃣ 计算目标 lt 位姿
# T_lt_target = T_forward_target @ T_lt_forward

# -----------------------
# 输出结果
# -----------------------
#print("目标 lt 位姿 (T_lt_target):\n", T_lt_target)

class QyhTeleopNode(Node):
    def __init__(self):
        super().__init__('qyh_teleop')

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
        self.tcp_ready = False
        
        # -30 degree yaw transform
        self.T_base_base_left = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, 0, -math.pi / 6]
        )
        
        # lt -> forward_lt
        self.T_lt_forward = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.pi / 2, -math.pi / 2]
        )
        
        # Pico手柄 → 人体手势坐标系
        self.T_vr_human_align = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.radians(35), 0]
        )
        
        # ---------- 增量模式缓存 ----------
        self.T_vr_start_base = None          # 冻结时刻的 VR（base_link）
        self.T_tcp_start_forward = None      # 冻结时刻的 TCP（forward_lt）

        self.get_logger().info('qyh_teleop node started')
    # ----------------------------------------------------
    # 基础矩阵工具
    # ----------------------------------------------------

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

    # ----------------------------------------------------
    # 回调
    # ----------------------------------------------------
    def left_joy_callback(self, msg):
        if hasattr(msg, 'axes') and len(msg.axes) > 3:
            self.left_grip_value_ = float(msg.axes[3])


    def left_tcp_pose_callback(self, msg):
        mm_to_m = 0.001
        try:
            if len(msg.position) >= 7:
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
                ]  # x y z w
                self.cur_tcp_rotm = R.from_quat(q).as_matrix()
                self.tcp_ready = True
        except Exception:
            pass

    def vr_left_pose_callback(self, msg):
        grip = self.left_grip_value_
        pressed = grip > self.grip_engage
        released = grip < self.grip_release
        
        # # log received vr pose and
        # msgRotation = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # try:
        #     self.get_logger().info(f"[LEFT] vr_pose received position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) "
        #                            f"orientation=({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f}) "
        #                            f"rpy=({math.degrees(msgRotation.as_euler('xyz')[0]):.1f}, {math.degrees(msgRotation.as_euler('xyz')[1]):.1f}, {math.degrees(msgRotation.as_euler('xyz')[2]):.1f}) "
        #                            f"grip={grip_value:.2f}")
        # except Exception:
        #     pass
                
        if not self.tcp_ready:
            self.get_logger().warn("TCP pose not ready, ignore clutch")
            return
        
        # 1.将msg转换成T_vr_base[np.eye(4)]
         # VR 当前（base_link）
        T_vr_now_base = self.make_transform_matrix(
            translation=[
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ],
            rpy=R.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]).as_euler('xyz')
        )
        
        # Pico → 人体手势对齐（必须在冻结/增量前）
        T_vr_now_base = T_vr_now_base @ self.T_vr_human_align
        
        state = self.left_clutch_state


        # ---------- 状态机 ----------

        if state == 'IDLE':
            if pressed:
                self.left_clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            # 1️⃣ 冻结 VR 起点
            self.T_vr_start_base = T_vr_now_base

            # 2️⃣ 冻结机械臂起点（forward_lt）
            T_lt_start = self.make_transform_matrix(
                self.cur_tcp_position,
                R.from_matrix(self.cur_tcp_rotm).as_euler('xyz')
            )
            self.T_tcp_start_forward = T_lt_start @ self.T_lt_forward

            self.left_clutch_state = 'TRACKING'
            self.get_logger().info('[LEFT] Tracking started')
            return

        if state == 'TRACKING':
            if released:
                self.left_clutch_state = 'IDLE'
                self.get_logger().info('[LEFT] Clutch RELEASED')
                return
        
        if self.left_clutch_state != 'TRACKING':
            return
        if self.T_vr_start_base is None or self.T_tcp_start_forward is None:
            return
        # ---------- 增量计算 ----------

        # VR → base_link_left
        T_vr_now_left = self.invert_transform(
            self.T_base_base_left
        ) @ T_vr_now_base

        T_vr_start_left = self.invert_transform(
            self.T_base_base_left
        ) @ self.T_vr_start_base

        # 对齐到 forward_lt
        T_vr_now_forward = T_vr_now_left @ self.invert_transform(self.T_lt_forward)
        T_vr_start_forward = T_vr_start_left @ self.invert_transform(self.T_lt_forward)

        # ⭐ 增量
        delta_T = T_vr_now_forward @ self.invert_transform(T_vr_start_forward)

        # 应用到机械臂起点
        T_forward_target = delta_T @ self.T_tcp_start_forward
        T_lt_target = T_forward_target @ self.T_lt_forward

        # ---------- 输出 ----------
        pos = T_lt_target[:3, 3]
        rotvec = R.from_matrix(T_lt_target[:3, :3]).as_rotvec()
        meter_to_mm = 1000.0
        out_pose = [
            pos[0] *  meter_to_mm,
            pos[1] *  meter_to_mm,
            pos[2] *  meter_to_mm,
            rotvec[0],
            rotvec[1], 
            rotvec[2]
        ]
        self.get_logger().info(
            f"target_pos(mm)=({out_pose[0]:.1f},{out_pose[1]:.1f},{out_pose[2]:.1f}) "
            f"target_rot(rad)=({out_pose[3]:.3f},{out_pose[4]:.3f},{out_pose[5]:.3f})"
        )
        self.publish_servo_p_command(out_pose)

    # ----------------------------------------------------

    def publish_servo_p_command(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = pose
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