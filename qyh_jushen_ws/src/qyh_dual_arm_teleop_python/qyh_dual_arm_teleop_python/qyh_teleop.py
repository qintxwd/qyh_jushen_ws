#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========================================================================================
# VR手柄遥控双臂机械臂系统 - 坐标系变换和增量控制
# ========================================================================================
#
# 总体目标：将VR手柄的姿态变化映射到机械臂末端的姿态变化
# 当人手"向前"移动时，机械臂末端也"向前"移动
#
# ========================================================================================
# 坐标系定义
# ========================================================================================
#
# 1. VR控制器坐标系 (经过T_vr_human_align变换后):
#    - x轴: 前 (人手向前伸直方向)
#    - y轴: 左 (人手向左倾斜方向)
#    - z轴: 上 (人手上抬方向)
#    - 位置: 在 vr_origin/base_link 坐标系下
#
# 2. 机械臂base_link坐标系:
#    - x轴: 前 (机器人前进方向)
#    - y轴: 左 (机器人左侧方向)
#    - z轴: 上 (机器人上方)
#    - vr_origin ≈ base_link (双臂中心位置)
#
# 3. 机械臂base_link_left坐标系:
#    - 相对于base_link有 -30° yaw旋转 (绕z轴)
#    - xyz偏移: [-0.0004, 0.08522, 0.0030]
#    - 这是左臂的实际安装点，用于补偿URDF与实际机器人的差异
#
# 4. 机械臂末端lt坐标系 (在base_link_left下):
#    - x轴: 左 (机械臂向左移动方向)
#    - y轴: 上 (机械臂向上移动方向)
#    - z轴: 前 (机械臂向前伸直方向)
#
# ========================================================================================
# 期望的映射关系 (核心需求)
# ========================================================================================
#
# 当人手"向前伸直"时 (VR x轴向前), 期望机械臂也"向前伸直" (lt z轴向前)
# 当人手"向左倾斜"时 (VR y轴向左), 期望机械臂也"向左倾斜" (lt x轴向左)
# 当人手"上抬"时 (VR z轴向上), 期望机械臂也"上抬" (lt y轴向上)
#
# 映射关系: VR x → lt z, VR y → lt x, VR z → lt y
#
# ========================================================================================
# 坐标变换逻辑
# ========================================================================================
#
# 1. 坐标系对齐变换矩阵 R_vr_to_arm:
#    将VR控制器坐标系(x前y左z上) 变换到 机械臂末端坐标系(x左y上z前)
#
#    向量变换: [vx, vy, vz]_vr → [vy, vz, vx]_arm
#    矩阵形式:
#    R_vr_to_arm = [
#        [0, 1, 0],  # lt_x = VR_y (左)
#        [0, 0, 1],  # lt_y = VR_z (上)
#        [1, 0, 0]   # lt_z = VR_x (前)
#    ]
#
# 2. 空间变换:
#    - VR数据在base_link下 (经过T_vr_human_align调整姿态)
#    - 需要变换到base_link_left坐标系: inv(T_base_base_left) @ T_vr_in_base_link
#    - T_base_base_left: base_link -> base_link_left (包含-30°yaw + xyz偏移[-0.0004, 0.08522, 0.0030])
#    - xyz偏移对RPY增量影响较小，主要影响位置计算
#
# ========================================================================================
# 旋转矩阵增量控制逻辑（参考官方 vr_robot_pose_converter_30degree_servo_p.py）
# ========================================================================================
#
# 核心思想: 
#   - VR旋转增量使用旋转矩阵计算（避免欧拉角不连续问题）
#   - 机械臂起始姿态使用原始RPY（直接从cur_tcp_rpy获取，避免四元数→RPY转换）
#   - 只在最后发送时才转换为RPY
#
# 1. ENGAGING阶段 (按下grip按钮时):
#    - 记录VR起始旋转矩阵: vr_start_rotm (用于计算增量)
#    - 记录机械臂起始RPY: arm_start_rpy (直接使用控制器报告的RPY，不做转换！)
#    - 记录起始位置: vr_start_position, arm_start_position
#
# 2. TRACKING阶段 (遥控过程中):
#    - 计算VR旋转矩阵增量: delta_R = R_current @ R_start^(-1)
#    - 从arm_start_rpy构建旋转矩阵: arm_start_rotm = R.from_euler('xyz', arm_start_rpy)
#    - 应用增量: target_rotm = delta_R @ arm_start_rotm
#    - 转换为RPY发送: target_rpy = R.from_matrix(target_rotm).as_euler('xyz')
#
# 3. 发送格式:
#    - xyz: 米 -> 毫米 (在publish函数中转换)
#    - rpy: 弧度 (JAKA SDK期望弧度)
#
# ========================================================================================
# 关键优势
# ========================================================================================
#
# 1. 直观性: VR手柄的转动直接对应机械臂的转动
# 2. 连续性: RPY增量避免了四元数到欧拉角的多解问题
# 3. 实时性: 计算简单，延迟低
# 4. 稳定性: 相对增量计算对漂移不敏感
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
        
        # base_link -> base_link_left 变换 (包含30°yaw和xyz偏移)
        # URDF定义: xyz="-0.0004 0.08522 0.0030" rpy="0 0 -0.5236" (-30°)
        # 注意: 左臂的y坐标是正值(0.08522)，表示在base_link右侧
        self.T_base_base_left = self.make_transform_matrix(
            translation=[-0.0004, 0.08522, 0.0030],  # URDF中的xyz偏移
            rpy=[0, 0, -math.pi / 6]  # -30° yaw (弧度)
        )
       
        
        # lt -> forward_lt
        self.T_lt_forward = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.pi / 2, -math.pi / 2]
        )

        # VR控制器坐标系(x前y左z上) -> 机械臂末端坐标系(x左y上z前)的对齐变换
        #
        # 坐标系分析：
        # VR控制器:     x前, y左, z上
        # 机械臂末端lt:  x左, y上, z前
        #
        # 期望的映射关系：
        # - 人手向前伸直(VR x向前) → 机械臂向前伸直(lt z向前)
        # - 人手向左倾斜(VR y向左) → 机械臂向左倾斜(lt x向左)
        # - 人手上抬(VR z向上) → 机械臂上抬(lt y向上)
        #
        # 所以映射关系：VR x -> lt z, VR y -> lt x, VR z -> lt y
        #
        # 变换矩阵推导：
        # 如果向量在VR坐标系下的坐标为 [vx, vy, vz]
        # 在lt坐标系下的坐标应该是 [vy, vz, vx]
        # 因为 lt_x = VR_y, lt_y = VR_z, lt_z = VR_x
        #
        # 变换矩阵 R 满足：R @ [vx, vy, vz] = [vy, vz, vx]
        # 解得：
        self.R_vr_to_arm = np.array([
            [0, 1, 0],  # lt_x = 0*vx + 1*vy + 0*vz = vy
            [0, 0, 1],  # lt_y = 0*vx + 0*vy + 1*vz = vz
            [1, 0, 0]   # lt_z = 1*vx + 0*vy + 0*vz = vx
        ])

        # 验证：
        # R @ [1, 0, 0] = [0, 0, 1] ✓ (VR x轴 -> lt z轴)
        # R @ [0, 1, 0] = [1, 0, 0] ✓ (VR y轴 -> lt x轴)
        # R @ [0, 0, 1] = [0, 1, 0] ✓ (VR z轴 -> lt y轴)

        # Pico手柄 → 人体手势坐标系
        self.T_vr_human_align = self.make_transform_matrix(
            translation=[0, 0, 0],
            rpy=[0, -math.radians(35), 0]
        )
        
        # ---------- 增量模式缓存 ----------
        # 使用旋转矩阵计算VR增量，使用原始RPY作为机械臂基准（避免四元数→RPY转换）
        self.vr_start_rotm = None            # 冻结时刻的 VR 旋转矩阵（3x3）
        self.vr_start_position = None        # 冻结时刻的 VR位置（米）
        self.arm_start_rpy = None            # 冻结时刻的 机械臂 RPY（弧度）- 直接使用机械臂报告的RPY
        self.arm_start_position = None       # 冻结时刻的 机械臂位置（米）

        self.get_logger().info('qyh_teleop node started')

        # 调试：验证坐标系变换和RPY增量计算
        self.debug_coordinate_alignment()
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

    def debug_coordinate_alignment(self):
        """调试坐标系对齐变换"""
        self.get_logger().info("=== Coordinate Alignment Debug ===")

        # 测试VR坐标系到机械臂坐标系的变换
        # VR x轴 (前) -> 机械臂坐标系
        vr_x_axis = np.array([1, 0, 0])
        arm_x_from_vr_x = self.R_vr_to_arm @ vr_x_axis
        self.get_logger().info(f"VR x轴(前) -> 机械臂坐标: {arm_x_from_vr_x} (期望: [0,0,1] 机械臂z轴)")

        # VR y轴 (左) -> 机械臂坐标系
        vr_y_axis = np.array([0, 1, 0])
        arm_y_from_vr_y = self.R_vr_to_arm @ vr_y_axis
        self.get_logger().info(f"VR y轴(左) -> 机械臂坐标: {arm_y_from_vr_y} (期望: [1,0,0] 机械臂x轴)")

        # VR z轴 (上) -> 机械臂坐标系
        vr_z_axis = np.array([0, 0, 1])
        arm_z_from_vr_z = self.R_vr_to_arm @ vr_z_axis
        self.get_logger().info(f"VR z轴(上) -> 机械臂坐标: {arm_z_from_vr_z} (期望: [0,1,0] 机械臂y轴)")

        # 注意：这里不测试完整的姿态变换，因为我们的增量控制不使用这种方式

        self.get_logger().info("=== Coordinate Debug Complete ===")

    def debug_rpy_incremental(self):
        """调试RPY增量计算逻辑"""
        self.get_logger().info("=== RPY Incremental Debug ===")

        # 模拟VR控制器旋转30° roll (VR坐标系下的roll增加)
        vr_start_rpy = np.array([0, 0, 0])  # 起始：无旋转
        vr_current_rpy = np.array([np.radians(30), 0, 0])  # 当前：VR roll +30°

        # 计算RPY增量
        delta_rpy_raw = vr_current_rpy - vr_start_rpy
        self.get_logger().info(f"VR roll+30° -> delta_rpy_raw: {np.degrees(delta_rpy_raw)}°")

        # 应用坐标系对齐: VR roll -> 机械臂 yaw (因为VR x->机械臂 z, roll绕x->yaw绕z)
        delta_rpy_aligned = self.R_vr_to_arm @ delta_rpy_raw
        self.get_logger().info(f"坐标系对齐后: delta_rpy_aligned: {np.degrees(delta_rpy_aligned)}°")
        self.get_logger().info("解释: VR的roll(绕x轴) -> 机械臂的yaw(绕z轴)")

        # 机械臂起始姿态
        arm_start_rpy = np.array([np.radians(10), np.radians(20), np.radians(30)])  # 10°, 20°, 30°
        target_rpy = arm_start_rpy + delta_rpy_aligned

        self.get_logger().info(f"机械臂起始RPY: {np.degrees(arm_start_rpy)}°")
        self.get_logger().info(f"目标RPY: {np.degrees(target_rpy)}°")
        self.get_logger().info("结果: 机械臂yaw增加了30°, 实现了VR roll -> 机械臂 yaw的映射")

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

    def rotation_matrix_z(self, angle_deg):
        """
        生成绕z轴旋转的3x3旋转矩阵
        参考官方: rotation_matrix_z(-30) 用于补偿左臂30度安装角
        
        Args:
            angle_deg: 旋转角度（度）
        Returns:
            3x3 旋转矩阵
        """
        angle_rad = math.radians(angle_deg)
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])

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
                # 从四元数统一生成RPY，确保一致性（避免C++端rpy和quat不一致的问题）
                self.cur_tcp_rotm = R.from_quat(q).as_matrix()
                self.cur_tcp_rpy = [msg.position[7], msg.position[8], msg.position[9]]
                # 只在首次就绪时打印
                if not self.tcp_ready:
                    self.get_logger().info(f"TCP ready: pos={[round(x*1000,1) for x in self.cur_tcp_position]}mm")
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
                
        state = self.left_clutch_state


        # ---------- 状态机 ----------

        if state == 'IDLE':
            if pressed:
                self.left_clutch_state = 'ENGAGING'
            return

        if state == 'ENGAGING':
            try:
                # ============================================================
                # 参考官方实现：vr_robot_pose_converter_30degree_servo_p.py
                # 
                # 核心思想：
                # 1. VR旋转增量使用旋转矩阵计算（正确）
                # 2. 机械臂起始姿态使用原始RPY（直接从cur_tcp_rpy获取，不做转换）
                # 3. 应用增量后再转换为RPY发送
                # 
                # 这样避免了：机械臂四元数→RPY的多解问题
                # ============================================================
                
                # 1️⃣ 冻结 VR 起点姿态和位置（在base_link_left坐标系下）
                T_vr_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_now_base
                
                # 记录VR起点旋转矩阵（用于计算增量）
                self.vr_start_rotm = T_vr_in_left[:3, :3].copy()
                self.vr_start_position = T_vr_in_left[:3, 3].copy()
                
                # 2️⃣ 冻结机械臂起点
                self.arm_start_rpy = np.array(self.cur_tcp_rpy).copy()
                self.arm_start_position = np.array(self.cur_tcp_position).copy()

                self.left_clutch_state = 'TRACKING'
                # 简化日志：只显示起点位置
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

        # ============================================================
        # 参考官方实现：vr_robot_pose_converter_30degree_servo_p.py
        # 
        # 核心流程：
        # 1. VR旋转增量使用旋转矩阵计算
        # 2. 将增量应用到机械臂起始旋转矩阵（从arm_start_rpy构建）
        # 3. 最后转换为RPY发送
        # ============================================================
        if (self.vr_start_rotm is None or self.vr_start_position is None or
            self.arm_start_rpy is None or self.arm_start_position is None):
            return

        # 检查数据有效性
        if np.isnan(T_vr_now_base).any():
            self.get_logger().error("T_vr_now_base contains NaN")
            return

        try:
            # ============================================================
            # 完全按照官方 vr_robot_pose_converter_30degree_servo_p.py 实现
            # ============================================================
            
            # 1️⃣ 计算当前VR位姿（在base_link_left坐标系下）
            T_vr_current_in_left = self.invert_transform(self.T_base_base_left) @ T_vr_now_base
            vr_current_rotm = T_vr_current_in_left[:3, :3]
            vr_current_pos = T_vr_current_in_left[:3, 3]

            # 2️⃣ 计算VR的旋转矩阵增量
            # 参考官方: left_rotvr_diff = np.dot(left_rot, np.linalg.inv(left_init_rot))
            delta_rotm_vr = vr_current_rotm @ np.linalg.inv(self.vr_start_rotm)

            # 3️⃣ 旋转增量直接应用（无需坐标变换！）
            # 
            # 关键理解：VR和lt都在同一个坐标系（base_link_left）下
            # 
            # 虽然VR坐标系是x前y左z上，lt坐标系是x左y上z前，
            # 但它们的"前""左""上"在base_link_left下是同一个方向！
            # 
            # 初始对应：VR的x轴(前) 和 lt的z轴(前) 在base_link_left下指向相同方向
            # 应用相同的delta_R后，这个对应关系保持不变！
            # 
            # 所以：直接使用delta_rotm_vr，不需要坐标变换
            delta_rotm_lt = delta_rotm_vr

            # 4️⃣ 从机械臂起始RPY构建旋转矩阵（使用原始RPY，避免四元数转换）
            arm_start_rotm = R.from_euler('xyz', self.arm_start_rpy).as_matrix()

            # 5️⃣ 应用旋转增量到机械臂起始姿态
            # 使用转换后的delta_rotm_lt（在lt坐标系下）
            target_rotm = delta_rotm_lt @ arm_start_rotm

            # 6️⃣ 转换为RPY发送（只在最后一步转换）
            # 参考官方: euler_angles_xyz_fixed = rotation.as_euler('xyz', degrees=False)
            target_rpy = R.from_matrix(target_rotm).as_euler('xyz', degrees=False)

            # 7️⃣ 计算目标位置（直接映射！）
            # 
            # VR位置增量和lt位置都在base_link_left坐标系下
            # 
            # VR向前移动(+x) → lt的x坐标也增加（都是base_link_left的x方向）
            # VR向左移动(+y) → lt的y坐标也增加（都是base_link_left的y方向）
            # VR向上移动(+z) → lt的z坐标也增加（都是base_link_left的z方向）
            # 
            # 直接加即可，无需坐标变换
            delta_pos = vr_current_pos - self.vr_start_position
            target_position = self.arm_start_position + delta_pos

            # 8️⃣ 发送目标位姿：xyz(m) + rpy(弧度)
            out_pose = [
                target_position[0],      # x (m)
                target_position[1],      # y (m)
                target_position[2],      # z (m)
                target_rpy[0],           # roll (弧度)
                target_rpy[1],           # pitch (弧度)
                target_rpy[2]            # yaw (弧度)
            ]

            # 简化日志：一行显示增量，便于快速判断
            # 向前移动10cm → Δx=0.100, Δy≈0, Δz≈0
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

    # ----------------------------------------------------

    def publish_servo_p_command(self, pose):
        """
        发送servo_p命令到机械臂
        
        Args:
            pose: [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]
                  位置单位: 米
                  姿态单位: 弧度
        
        发送格式:
            jaka_control_node期望7个值: [x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad, unused]
            JAKA SDK的edg_servo_p期望: xyz(mm) + rpy(弧度)
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # 发送格式：xyz(mm) + rpy(弧度) + 1个占位值（共7个值）
        # 注意：jaka_control_node检查 msg->position.size() != 7
        msg.position = [
            pose[0] * 1000.0,  # x: m -> mm
            pose[1] * 1000.0,  # y: m -> mm
            pose[2] * 1000.0,  # z: m -> mm
            pose[3],           # roll (弧度)
            pose[4],           # pitch (弧度)
            pose[5],           # yaw (弧度)
            0.0                # 占位值（jaka_control_node期望7个值）
        ]

        # 简化：只显示目标位置和姿态
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