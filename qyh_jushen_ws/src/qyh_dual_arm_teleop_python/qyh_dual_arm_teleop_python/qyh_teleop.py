#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 左乘是"换参考系（改坐标系的解释）"，右乘是"在当前坐标系里再做一次变换（定义子系 / 叠加运动）"。
#
# 坐标系说明：
# - VR控制器：在vr_origin下，坐标系x前y左z上 (ROS标准坐标系)
# - 机械臂base_link ≈ vr_origin (双臂中心)
# - 机械臂base_link_left = base_link * R_yaw(-30°) (左臂安装点)
# - 机械臂末端lt：在base_link_left下，坐标系x左y上z前
#
# RPY直接增量计算逻辑：
# 1. 冻结时记录VR和机械臂的RPY起点
# 2. 运行时计算VR当前RPY（经坐标系对齐和30°偏移变换）
# 3. delta_rpy = vr_current_rpy - vr_start_rpy
# 4. target_rpy = arm_start_rpy + delta_rpy
# 5. 发送格式：xyz(mm) + rpy(degree)
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
        self.cur_tcp_rpy = [0.0, 0.0, 0.0]
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

        # VR控制器坐标系(x前y左z上) -> 机械臂末端坐标系(x左y上z前)的对齐变换
        # VR: x前 y左 z上 -> 机械臂: x左 y上 z前
        self.R_vr_to_arm = np.array([
            [ 0,  0,  1],  # VR x前 -> 机械臂 z前
            [ 1,  0,  0],  # VR y左 -> 机械臂 x左
            [ 0,  1,  0]   # VR z上 -> 机械臂 y上
        ])

        # Pico手柄 → 人体手势坐标系 (暂时注释，避免双重变换)
        self.T_vr_human_align = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.radians(35), 0]
        )
        
        # ---------- 增量模式缓存 ----------
        self.vr_start_rpy = None             # 冻结时刻的 VR RPY（弧度）
        self.arm_start_rpy = None            # 冻结时刻的 机械臂 RPY（弧度）
        self.arm_start_position = None       # 冻结时刻的 机械臂位置（米）

        self.get_logger().info('qyh_teleop node started')

        # 调试：验证坐标系变换和RPY增量计算
        self.debug_coordinate_transforms()
        self.debug_rpy_incremental()
    # ----------------------------------------------------
    # 坐标系调试
    # ----------------------------------------------------

    def debug_coordinate_transforms(self):
        """调试坐标系变换关系"""
        self.get_logger().info("=== Coordinate System Debug ===")

        # VR控制器默认姿态 (x前y左z上)
        T_vr_default = np.eye(4)
        self.print_pose(T_vr_default, "VR控制器默认姿态(x前y左z上)")

        # 应用VR到机械臂坐标系对齐
        T_vr_aligned = np.eye(4)
        T_vr_aligned[:3, :3] = self.R_vr_to_arm
        self.print_pose(T_vr_aligned, "VR对齐到机械臂坐标系(x左y上z前)")

        # base_link_left的30度偏移
        self.print_pose(self.T_base_base_left, "base_link -> base_link_left (-30°yaw)")

        self.get_logger().info("=== Debug Complete ===")

    def debug_rpy_incremental(self):
        """调试RPY增量计算逻辑"""
        self.get_logger().info("=== RPY Incremental Debug ===")

        # 模拟VR控制器旋转30°
        vr_start_rpy = np.array([0, 0, 0])  # 起始：无旋转
        vr_current_rpy = np.array([np.radians(30), 0, 0])  # 当前：roll +30°

        # 机械臂起始姿态
        arm_start_rpy = np.array([np.radians(10), np.radians(20), np.radians(30)])  # 10°, 20°, 30°

        # 计算增量
        delta_rpy = vr_current_rpy - vr_start_rpy
        target_rpy = arm_start_rpy + delta_rpy

        self.get_logger().info("RPY增量计算示例:")
        self.get_logger().info(f"VR start RPY: {np.degrees(vr_start_rpy)}°")
        self.get_logger().info(f"VR current RPY: {np.degrees(vr_current_rpy)}°")
        self.get_logger().info(f"Delta RPY: {np.degrees(delta_rpy)}°")
        self.get_logger().info(f"Arm start RPY: {np.degrees(arm_start_rpy)}°")
        self.get_logger().info(f"Target RPY: {np.degrees(target_rpy)}°")

        self.get_logger().info("=== RPY Debug Complete ===")

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

    def choose_rpy_closest(self, rpy_prev, rotm):
        """
        Given previous rpy (array-like, radians) and a 3x3 rotation matrix (numpy),
        return an Euler `xyz` triple nearest to rpy_prev using candidate enumeration
        and 2*pi unwrapping. Handles the +-90deg singularity by preserving yaw.
        """
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
                        # normalize each component into prev +/- pi
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

        # normalize yaw to [-pi,pi]
        best[2] = math.atan2(math.sin(best[2]), math.cos(best[2]))
        return best

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
            if len(msg.position) >= 10:
                self.cur_tcp_position = [
                    msg.position[0] * mm_to_m,
                    msg.position[1] * mm_to_m,
                    msg.position[2] * mm_to_m
                ]
                # C++ publishes tcp_pose as [x,y,z, w,x,y,z, rpy...]
                # indices: [3]=w, [4]=x, [5]=y, [6]=z
                q = [
                    msg.position[4],
                    msg.position[5],
                    msg.position[6],
                    msg.position[3]
                ]  # x y z w
                self.cur_tcp_rpy = [msg.position[7], msg.position[8], msg.position[9]]                
                self.get_logger().info(f"TCP position x,y,z = {self.cur_tcp_position}")
                self.get_logger().info(f"TCP quaternion x,y,z,w = {q}")
                self.get_logger().info(f"TCP rpy (deg) = {np.degrees(self.cur_tcp_rpy)}")
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
        # Pico → 人体手势对齐（必须在冻结/增量前）
        T_vr_now_base = T_vr_now_base @ self.invert_transform(self.T_vr_human_align)
        # 打印VR位姿
        self.print_pose(T_vr_now_base, label="T_vr_now_base")
        
        state = self.left_clutch_state


        # ---------- 状态机 ----------

        if state == 'IDLE':
            if pressed:
                self.left_clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            try:
                # 1️⃣ 冻结 VR 起点RPY（考虑坐标系对齐）
                # VR控制器 -> 机械臂坐标系对齐 + base_link到base_link_left的30°偏移
                T_vr_aligned = np.eye(4)
                T_vr_aligned[:3, :3] = self.R_vr_to_arm @ T_vr_now_base[:3, :3] @ self.R_vr_to_arm.T
                T_vr_aligned[:3, 3] = T_vr_now_base[:3, 3]

                # 应用base_link到base_link_left的变换
                T_vr_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_aligned

                self.vr_start_rpy = R.from_matrix(T_vr_in_left[:3, :3]).as_euler('xyz', degrees=False)
                self.get_logger().info(f"VR start RPY (in base_link_left): "
                                     f"roll={np.degrees(self.vr_start_rpy[0]):.1f}°, "
                                     f"pitch={np.degrees(self.vr_start_rpy[1]):.1f}°, "
                                     f"yaw={np.degrees(self.vr_start_rpy[2]):.1f}°")

                # 2️⃣ 冻结机械臂起点RPY和位置
                self.arm_start_rpy = np.array(self.cur_tcp_rpy).copy()  # 弧度
                self.arm_start_position = np.array(self.cur_tcp_position).copy()  # 米
                self.get_logger().info(f"Arm start RPY: roll={np.degrees(self.arm_start_rpy[0]):.1f}°, "
                                     f"pitch={np.degrees(self.arm_start_rpy[1]):.1f}°, "
                                     f"yaw={np.degrees(self.arm_start_rpy[2]):.1f}°")
                self.get_logger().info(f"Arm start position: x={self.arm_start_position[0]:.3f}, "
                                     f"y={self.arm_start_position[1]:.3f}, z={self.arm_start_position[2]:.3f} m")

                self.left_clutch_state = 'TRACKING'
                self.get_logger().info('---[LEFT] Tracking started')
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

        # ---------- RPY直接增量计算 ----------
        if self.vr_start_rpy is None or self.arm_start_rpy is None or self.arm_start_position is None:
            return

        # 检查数据有效性
        if np.isnan(T_vr_now_base).any():
            self.get_logger().error("T_vr_now_base contains NaN")
            return

        try:
            # 1️⃣ 计算当前VR的RPY（应用坐标系对齐和30°偏移）
            T_vr_current_aligned = np.eye(4)
            T_vr_current_aligned[:3, :3] = self.R_vr_to_arm @ T_vr_now_base[:3, :3] @ self.R_vr_to_arm.T
            T_vr_current_aligned[:3, 3] = T_vr_now_base[:3, 3]

            T_vr_current_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_current_aligned
            # vr_current_rpy = R.from_matrix(T_vr_current_in_left[:3, :3]).as_euler('xyz', degrees=False)
            vr_current_rpy = self.choose_rpy_closest(
                self.vr_start_rpy,
                T_vr_current_in_left[:3, :3]
            )

            # 2️⃣ 计算VR的RPY增量
            delta_rpy = vr_current_rpy - self.vr_start_rpy

            # 3️⃣ 计算目标RPY：机械臂起点RPY + VR的RPY增量
            target_rpy = self.arm_start_rpy + delta_rpy

            # 4️⃣ 计算目标位置：机械臂起点位置 + VR的位置增量
            # VR控制器位置变化直接映射到机械臂位置变化
            vr_current_pos = T_vr_now_base[:3, 3]  # 当前VR位置
            vr_start_pos = np.array([0, 0, 0])     # VR起点位置（相对增量，从0开始）

            # 计算VR的位置增量（在VR坐标系下）
            delta_pos_vr = vr_current_pos - vr_start_pos

            # 应用VR到机械臂坐标系的对齐变换到位置增量
            delta_pos_aligned = self.R_vr_to_arm @ delta_pos_vr

            # 计算目标位置
            target_position = self.arm_start_position + delta_pos_aligned

            # 5️⃣ 发送目标位姿：xyz(mm) + rpy(degree)
            out_pose = [
                target_position[0],      # x (m)
                target_position[1],      # y (m)
                target_position[2],      # z (m)
                np.degrees(target_rpy[0]),  # roll (degree)
                np.degrees(target_rpy[1]),  # pitch (degree)
                np.degrees(target_rpy[2])   # yaw (degree)
            ]

            self.get_logger().info(
                f"VR current RPY: roll={np.degrees(vr_current_rpy[0]):.1f}°, "
                f"pitch={np.degrees(vr_current_rpy[1]):.1f}°, yaw={np.degrees(vr_current_rpy[2]):.1f}°")
            self.get_logger().info(
                f"Delta RPY: roll={np.degrees(delta_rpy[0]):.1f}°, "
                f"pitch={np.degrees(delta_rpy[1]):.1f}°, yaw={np.degrees(delta_rpy[2]):.1f}°")
            self.get_logger().info(
                f"Target: pos(m)=({out_pose[0]:.3f},{out_pose[1]:.3f},{out_pose[2]:.3f}) "
                f"rpy(deg)=({out_pose[3]:.1f},{out_pose[4]:.1f},{out_pose[5]:.1f})")

            self.publish_servo_p_command(out_pose)

        except Exception as e:
            self.get_logger().error(f"Error in RPY incremental calculation: {e}")
            return

    # ----------------------------------------------------

    def publish_servo_p_command(self, pose):
        # Expecting pose: [x_m, y_m, z_m, roll_deg, pitch_deg, yaw_deg]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 发送格式：xyz(mm) + rpy(degree)
        msg.position = [
            pose[0] * 1000.0,  # x: m -> mm
            pose[1] * 1000.0,  # y: m -> mm
            pose[2] * 1000.0,  # z: m -> mm
            pose[3],           # roll (degree)
            pose[4],           # pitch (degree)
            pose[5]            # yaw (degree)
        ]

        self.get_logger().info(f"Sending to arm: x={msg.position[0]:.1f}mm, y={msg.position[1]:.1f}mm, "
                             f"z={msg.position[2]:.1f}mm, r={msg.position[3]:.1f}°, "
                             f"p={msg.position[4]:.1f}°, y={msg.position[5]:.1f}°")

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