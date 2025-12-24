#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node

# 订阅 /vr/left_controller/pose_raw 话题 [geometry_msgs/PoseStamped]  ==> vr_left_pose_callback
# 订阅 /left_arm/joint_states 话题 [sensor_msgs/JointState]  ==> joint_states_callback
# 订阅 /left_arm/tcp_pose 话题 [sensor_msgs/JointState]  ==> cur_tcp_pose_callback
# 订阅 /vr/left_controller/joy 话题 [sensor_msgs/Joy]  ==> left_joy_callback  left_grip_value_ = msg->axes[3];

# def left_joy_callback(self, msg: Joy):
#     # axes[3] 对应 grip
#     self.left_grip_value_ = msg.axes[3]

# def vr_left_pose_callback(self, msg: PoseStamped):
#     # 直接使用 self.left_grip_value_ 来判断 clutch
#     grip_value = getattr(self, 'left_grip_value_', 0.0)  # 如果还没收到 Joy 消息，默认 0
#     grip_engage = 0.5
#     grip_release = 0.3
#     clutch_pressed = grip_value > grip_engage
#     clutch_released = grip_value < grip_release
#     ... # 其余逻辑保持不变
    
    
# def joint_states_callback(self, msg):
#     self.cur_joint_val = list(msg.position)
        
# def cur_tcp_pose_callback(self, msg):
#         self.cur_tcp_position = [msg.position[0],msg.position[1],msg.position[2]]
#                                 #  pose.position.x, msg.pose.position.y, msg.pose.position.z]
#         # self.get_logger().info(f'update pos: {self.cur_tcp_position}')
#         # 创建一个 Rotation 对象并从四元数初始化,(x, y, z, w)
#         # rotation = R.from_quat([msg.position[4], msg.position[5], msg.position[6], msg.position[3]])
#         # 获取旋转矩阵
#         self.cur_tcp_rotm = rotation.as_matrix()
#         # self.get_logger().info(f'update rot: {self.cur_tcp_rotm}')

# def vr_left_pose_callback(self, msg: geometry_msgs.msg.PoseStamped, grip_value: float):
#     # --- 定义 clutch 门限 ---
#     grip_engage = 0.5   # 按下阈值
#     grip_release = 0.3  # 松开阈值

#     # --- 判断 clutch 状态 ---
#     clutch_pressed = grip_value > grip_engage
#     clutch_released = grip_value < grip_release

#     # 初始化 clutch 状态变量
#     if not hasattr(self, 'left_clutch_state'):
#         self.left_clutch_state = 'IDLE'

#     prev_state = self.left_clutch_state
#     state = self.left_clutch_state

#     # 状态机逻辑
#     if state == 'IDLE':
#         if clutch_pressed:
#             state = 'ENGAGING'
#             self.get_logger().info(f"[LEFT] Clutch ENGAGED (grip={grip_value:.2f})")
#     elif state == 'ENGAGING':
#         state = 'TRACKING'
#         # 在 ENGAGING -> TRACKING 时记录初始 VR 位姿
#         meter_to_mm = 1000.0
#         self.init_left_vr_pose = [
#             msg.pose.position.x * meter_to_mm,
#             msg.pose.position.y * meter_to_mm,
#             msg.pose.position.z * meter_to_mm
#         ]
#         q = [msg.pose.orientation.x, msg.pose.orientation.y,
#              msg.pose.orientation.z, msg.pose.orientation.w]
#         roll, pitch, yaw = tf_transformations.euler_from_quaternion(q, axes='sxyz')
#         self.init_left_vr_pose += [roll, pitch, yaw]

#         # 初始化机械臂当前位置
#         self.leftarm_init_pos = self.cur_tcp_position[:]
#         self.leftarm_init_rot = self.cur_tcp_rotm.copy()
#     elif state == 'TRACKING':
#         if clutch_released:
#             state = 'RELEASING'
#             self.get_logger().info(f"[LEFT] Clutch RELEASED (grip={grip_value:.2f})")
#     elif state == 'RELEASING':
#         state = 'IDLE'

#     self.left_clutch_state = state

#     # --- 如果未进入 TRACKING，则直接返回，不计算增量 ---
#     if state != 'TRACKING':
#         return

#     # --- 计算增量 --- 
#     meter_to_mm = 1000.0
#     left_pos = [
#         msg.pose.position.x * meter_to_mm,
#         msg.pose.position.y * meter_to_mm,
#         msg.pose.position.z * meter_to_mm
#     ]
#     q = [msg.pose.orientation.x, msg.pose.orientation.y,
#          msg.pose.orientation.z, msg.pose.orientation.w]
#     roll, pitch, yaw = tf_transformations.euler_from_quaternion(q, axes='sxyz')
#     left_rot_rad = [roll, pitch, yaw]
#     left_vr_pose = left_pos + left_rot_rad

#     # 计算 VR 增量
#     left_diff = [a - b for a, b in zip(left_vr_pose, self.init_left_vr_pose)]

#     # 旋转矩阵差
#     left_init_rot = self.euler_to_rotation_matrix(
#         -self.init_left_vr_pose[3],
#         -self.init_left_vr_pose[4],
#         self.init_left_vr_pose[5]
#     )
#     left_rot = self.euler_to_rotation_matrix(
#         -left_vr_pose[3],
#         -left_vr_pose[4],
#         left_vr_pose[5]
#     )
#     left_rotvr_diff = np.dot(left_rot, np.linalg.inv(left_init_rot))

#     # 可选旋转缩放
#     rotation_scale_factor = 0.5
#     if rotation_scale_factor != 1.0:
#         left_rotvr_diff = self.scale_rotation_matrix_axis_angle(left_rotvr_diff, rotation_scale_factor)

#     # 坐标系对齐
#     vr_rot = np.array([[0, 0, -1],
#                        [-1, 0, 0],
#                        [0, 1, 0]])
#     left_diff_base = np.dot(vr_rot, np.dot(left_rotvr_diff, np.linalg.inv(vr_rot)))

#     # 左臂 Z 轴修正
#     z_rot = self.rotation_matrix_z(np.deg2rad(-30))
#     left_diff_base = np.dot(z_rot, np.dot(left_diff_base, np.linalg.inv(z_rot)))

#     # 机械臂最终位姿
#     leftarm_finalrot = np.dot(left_diff_base, self.leftarm_init_rot)
#     rotation = R.from_matrix(leftarm_finalrot)
#     euler_angles_xyz_fixed = rotation.as_euler('xyz', degrees=False)

#     next_pose = [0.0]*6
#     next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]
#     next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]
#     next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]
#     next_pose[3] = euler_angles_xyz_fixed[0]
#     next_pose[4] = euler_angles_xyz_fixed[1]
#     next_pose[5] = euler_angles_xyz_fixed[2]

#     self.publish_servo_p_command(next_pose)

# def euler_to_rotation_matrix(self, rx, ry, rz):
#         # rx, ry, rz = np.radians([rx, ry, rz])
#         # 计算旋转矩阵
#         Rx = np.array([
#             [1, 0, 0],
#             [0, np.cos(rx), -np.sin(rx)],
#             [0, np.sin(rx), np.cos(rx)]
#         ])
    
#         Ry = np.array([
#             [np.cos(ry), 0, np.sin(ry)],
#             [0, 1, 0],
#             [-np.sin(ry), 0, np.cos(ry)]
#         ])
    
#         Rz = np.array([
#             [np.cos(rz), -np.sin(rz), 0],
#             [np.sin(rz), np.cos(rz), 0],
#             [0, 0, 1]
#         ])
#         # 组合三个旋转矩阵
#         # R = np.dot(Rz, np.dot(Rx, Ry))
#         R = np.dot(Ry, np.dot(Rx, Rz))
#         return R 
    
#     def rotation_matrix_z(self,angle):
#         theta = np.radians(angle)
#         return np.array([
#             [np.cos(theta),-np.sin(theta),0],
#             [np.sin(theta),np.cos(theta),0],
#             [0,0,1]
#         ])

#  def scale_rotation_matrix_axis_angle(self, rotation_matrix, scale_factor):
#         """
#         通过缩放轴角来缩放旋转矩阵
        
#         参数:
#         rotation_matrix: 3x3 旋转矩阵
#         scale_factor: 缩放系数 (0-1缩小旋转, >1放大旋转)
        
#         返回:
#         缩放后的旋转矩阵
#         """
#         # 使用scipy将旋转矩阵转换为轴角表示
#         rotation = R.from_matrix(rotation_matrix)
        
#         # 获取旋转向量(轴角表示，向量方向为旋转轴，长度为旋转角度)
#         rotvec = rotation.as_rotvec()
        
#         # 缩放旋转角度(保持旋转轴方向不变)
#         scaled_rotvec = rotvec * scale_factor
        
#         # 将缩放后的轴角转换回旋转矩阵
#         scaled_rotation = R.from_rotvec(scaled_rotvec)
#         scaled_matrix = scaled_rotation.as_matrix()
        
#         return scaled_matrix
#     def publish_servo_p_command(self, joint_positions):
#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         # msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
#         msg.position = joint_positions
#         self.publisher_servo_p.publish(msg)

def main(args=None):
    


if __name__ == '__main__':
    main()
