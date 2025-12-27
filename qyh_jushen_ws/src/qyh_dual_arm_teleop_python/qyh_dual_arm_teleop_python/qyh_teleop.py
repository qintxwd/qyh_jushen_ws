#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 左乘是“换参考系（改坐标系的解释）”，右乘是“在当前坐标系里再做一次变换（定义子系 / 叠加运动）”。
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
        self.T_vr_start_left = None          # 冻结时刻的 VR（base_link_left）
        self.T_tcp_start_forward = None      # 冻结时刻的 TCP（forward_lt）
        
        # self.last_rotation = None  # 使用Rotation对象跟踪，避免Euler多值性
        
        
        # #测试代码：
        P = self.make_transform_matrix(
            translation=[1.0, 0, 0],
            rpy=[0, 0, 0])
        # # TempTest = self.invert_transform(
        # #     self.T_base_base_left
        # # ) @ P
        # # self.print_pose(TempTest, label="TempTest")
        T = P @ self.invert_transform(self.T_lt_forward)
        self.print_pose(T, label="T")
        P2 = T @ self.T_lt_forward
        self.print_pose(P2, label="P2")

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
        """
        更新机械臂 TCP 当前位姿（绝对值），不进行任何积分或累积，
        姿态使用旋转矩阵保持稳定。
        """
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
                self.get_logger().info(f"TCP position x,y,z = {self.cur_tcp_position}")
                self.get_logger().info(f"TCP quaternion x,y,z,w = {q}")
                # print rpy
                self.get_logger().info(f"TCP rpy (deg) = {np.degrees(R.from_quat(q).as_euler('xyz'))}")
                self.cur_tcp_rotm = R.from_quat(q).as_matrix()
                self.tcp_ready = True
        except Exception:
            pass
    # print 4x4 numpy matrix pose
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
        
        # 1.将msg转换成T_vr_base[np.eye(4)]
         # VR 当前（base_link）
        T_vr_now_base = np.eye(4)
        T_vr_now_base[:3, :3] = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]).as_matrix()
        T_vr_now_base[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # self.print_pose(T_vr_now_base, label="T_vr_now_base before alignment")
        # Pico → 人体手势对齐（必须在冻结/增量前）
        T_vr_now_base = T_vr_now_base @ self.invert_transform(self.T_vr_human_align)
        # 左乘：用对齐变换修改参考系（T_new = T_align * T_old）
        # 这里是把 Pico 手柄的原始 pose 解释为人体对齐坐标系下的 pose。
        # 打印一下
        self.print_pose(T_vr_now_base, label="T_vr_now_base after alignment")
        
        state = self.left_clutch_state


        # ---------- 状态机 ----------

        if state == 'IDLE':
            if pressed:
                self.left_clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            # 1️⃣ 冻结 VR 起点
            self.T_vr_start_base = T_vr_now_base
            self.print_pose(self.T_vr_start_base, label="T_vr_start_base")
            self.T_vr_start_left = self.invert_transform(
                self.T_base_base_left
            ) @ self.T_vr_start_base
            self.print_pose(self.T_vr_start_left, label="T_vr_start_left")
            # 2️⃣ 冻结机械臂起点（forward_lt）
            T_lt_start = np.eye(4)
            T_lt_start[:3, :3] = self.cur_tcp_rotm   # ✅ 直接用 rotm
            T_lt_start[:3, 3]  = self.cur_tcp_position
            self.print_pose(T_lt_start, label="T_lt_start")
            # 右乘：在 lt_start 的坐标系里应用 lt->forward（子系/叠加运动）
            # T_tcp_start_forward = T_lt_start * T_lt_forward
            # 这样得到的是 forward 相对于 base 的位姿（以 T_lt_start 为参考，右乘应用子框架变换）。
            self.T_tcp_start_forward = T_lt_start @ self.T_lt_forward
            self.print_pose(self.T_tcp_start_forward, label="T_tcp_start_forward")
            self.left_clutch_state = 'TRACKING'
            self.get_logger().info('---[LEFT] Tracking started')
            return

        if state == 'TRACKING':
            if released:
                self.left_clutch_state = 'IDLE'
                self.get_logger().info('[LEFT] Clutch RELEASED')
                return
        
        if self.left_clutch_state != 'TRACKING':
            return

        # ---------- 增量计算 ----------
        if self.T_vr_start_base is None or self.T_tcp_start_forward is None:
            return
        
        if np.isnan(T_vr_now_base).any(): self.get_logger().error("T_vr_now_base contains NaN")
        if np.isnan(self.cur_tcp_rotm).any(): self.get_logger().error("cur_tcp_rotm contains NaN")
        # VR → base_link_left
        # 左乘 inv(T_base_base_left)：将在 base_link 表示的 pose 转换到 base_link_left 的参考系
        # T_vr_now_left = inv(T_base_base_left) * T_vr_now_base
        T_vr_now_left = self.invert_transform(
            self.T_base_base_left
        ) @ T_vr_now_base
        
        self.print_pose(T_vr_now_left, label="T_vr_now_left")

        # 对齐到 forward_lt
        # 右乘 inv(T_lt_forward)：将 vr 在 left 的位姿表达为 forward 坐标系下的位姿
        # （T_lt_forward 是 lt->forward，因此其逆是 forward->lt，在右侧应用可将坐标表达到 forward）
        T_vr_now_forward = T_vr_now_left @ self.invert_transform(self.T_lt_forward)
        self.print_pose(T_vr_now_forward, label="T_vr_now_forward")
        # 同上，针对冻结时刻的起点
        T_vr_start_forward = self.T_vr_start_left @ self.invert_transform(self.T_lt_forward)
        self.print_pose(T_vr_start_forward, label="T_vr_start_forward")
        
        # 相对变换 ΔT（唯一合法的“增量”）
        # delta = now * inv(start) —— 通过在左侧乘以 now 并右侧乘以 inv(start) 得到从 start 到 now 的变换
        delta_T = T_vr_now_forward @ self.invert_transform(T_vr_start_forward)
        self.print_pose(delta_T, label="delta_T")
        rotvec = R.from_matrix(delta_T[:3,:3]).as_rotvec()
        self.get_logger().info(f"delta rotvec: {rotvec}")
        if np.isnan(delta_T).any() or np.isinf(delta_T).any():
            self.get_logger().error("delta_T contains NaN or Inf!")


        # 应用到机械臂起点
        # 左乘 delta_T：在 world/forward 参考系中应用该增量到起始 forward 位姿
        T_forward_target = delta_T @ self.T_tcp_start_forward
        self.print_pose(T_forward_target, label="T_forward_target")
        # 将 forward 目标转换回 lt 坐标系（右乘 inv(T_lt_forward)）
        T_lt_target = T_forward_target @ self.invert_transform(self.T_lt_forward)
        self.print_pose(T_lt_target, label="T_lt_target")
        if np.isnan(T_lt_target).any() or np.isinf(T_lt_target).any():
            self.get_logger().error("T_lt_target contains NaN/Inf")
            return
        # 1️⃣ 位置
        pos = T_lt_target[:3, 3]
        rpy = R.from_matrix(T_lt_target[:3, :3]).as_euler('xyz', degrees=False)

        # 2️⃣ 输出（转换为 mm）
        meter_to_mm = 1000.0
        out_pose = [
            pos[0] *  meter_to_mm,
            pos[1] *  meter_to_mm,
            pos[2] *  meter_to_mm,
            rpy[0],
            rpy[1], 
            rpy[2]
        ]
        self.get_logger().info(
            f"target_pos(mm)=({out_pose[0]:.1f},{out_pose[1]:.1f},{out_pose[2]:.1f}) " # xyz
            f"target_rot(rad)=({out_pose[3]:.3f},{out_pose[4]:.3f},{out_pose[5]:.3f})" # rpy
            # rpy in deg
            f" target_rot(deg)=({math.degrees(out_pose[3]):.1f},{math.degrees(out_pose[4]):.1f},{math.degrees(out_pose[5]):.1f})"
        )
        self.publish_servo_p_command(out_pose)

    # ----------------------------------------------------

    def publish_servo_p_command(self, pose):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = pose
        # self.left_servo_pub.publish(msg)

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