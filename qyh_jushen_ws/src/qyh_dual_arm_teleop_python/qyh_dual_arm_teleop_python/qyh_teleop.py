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

class QyhTeleopNode(Node):
    def __init__(self):
        super().__init__('qyh_teleop')

        # publishers
        self.left_servo_pub = self.create_publisher(JointState, '/teleop/left/servo_p', 10)

        # subscribers
        self.left_pose_sub = self.create_subscription(PoseStamped, '/vr/left_controller/pose_raw', self.vr_left_pose_callback, 10)
        self.left_joy_sub = self.create_subscription(Joy, '/vr/left_controller/joy', self.left_joy_callback, 10)
        self.left_tcp_sub = self.create_subscription(JointState, '/left_arm/tcp_pose', self.left_tcp_pose_callback, 10)
        self.left_joint_sub = self.create_subscription(JointState, '/left_arm/joint_states', self.left_joint_states_callback, 10)

        # clutch thresholds
        self.grip_engage = 0.6
        self.grip_release = 0.3

        # current values
        self.left_grip_value_ = 0.0
        self.cur_tcp_position = [0.0, 0.0, 0.0]
        self.cur_tcp_rotm = np.eye(3)

        # state machine: IDLE, ENGAGING, TRACKING, RELEASING
        self.left_clutch_state = 'IDLE'

        # initial references for tracking
        self.init_left_vr_pose = None
        self.leftarm_init_pos = None
        self.leftarm_init_rot = None

        self.get_logger().info('qyh_teleop node started')

    def left_joy_callback(self, msg):
        if hasattr(msg, 'axes') and len(msg.axes) > 3:
            self.left_grip_value_ = float(msg.axes[3])

    def left_joint_states_callback(self, msg):
        try:
            self.cur_joint_val = list(msg.position)
        except Exception:
            self.cur_joint_val = []

    def left_tcp_pose_callback(self, msg):
        try:
            if len(msg.position) >= 3:
                self.cur_tcp_position = [msg.position[0], msg.position[1], msg.position[2]]
            if len(msg.position) >= 7:
                rotation = R.from_quat([msg.position[4], msg.position[5], msg.position[6], msg.position[3]]) #msg.position[3~6]:w x y z
                self.cur_tcp_rotm = rotation.as_matrix()
                # log received tcp pose (quaternion in x,y,z,w order used with scipy Rotation)
                # try:
                #     q_xyz_w = [msg.position[4], msg.position[5], msg.position[6], msg.position[3]]
                #     self.get_logger().info(f"[LEFT] tcp_pose received position={self.cur_tcp_position} quat(x,y,z,w)={q_xyz_w}")
                # except Exception:
                #     pass
        except Exception:
            pass

    def vr_left_pose_callback(self, msg):
        grip_value = getattr(self, 'left_grip_value_', 0.0)
        clutch_pressed = grip_value > self.grip_engage
        clutch_released = grip_value < self.grip_release
        
        # # log received vr pose
        # try:
        #     self.get_logger().info(f"[LEFT] vr_pose received position=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) "
        #                            f"orientation=({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f}) "
        #                            f"grip={grip_value:.2f}")
        # except Exception:
        #     pass

        state = self.left_clutch_state

        if state == 'IDLE':
            if clutch_pressed:
                state = 'ENGAGING'
                self.get_logger().info(f"[LEFT] Clutch ENGAGING (grip={grip_value:.2f})")
        elif state == 'ENGAGING':
            state = 'TRACKING'
            meter_to_mm = 1000.0
            px = msg.pose.position.x * meter_to_mm
            py = msg.pose.position.y * meter_to_mm
            pz = msg.pose.position.z * meter_to_mm
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            roll, pitch, yaw = R.from_quat(q).as_euler('xyz', degrees=False)
            self.init_left_vr_pose = [px, py, pz, roll, pitch, yaw]

            self.leftarm_init_pos = list(self.cur_tcp_position)
            self.leftarm_init_rot = np.array(self.cur_tcp_rotm)
            self.get_logger().info('[LEFT] Tracking started')
        elif state == 'TRACKING':
            if clutch_released:
                state = 'RELEASING'
                self.get_logger().info(f"[LEFT] Clutch RELEASED (grip={grip_value:.2f})")
        elif state == 'RELEASING':
            state = 'IDLE'

        self.left_clutch_state = state

        if state != 'TRACKING':
            return

        meter_to_mm = 1000.0
        vx = msg.pose.position.x * meter_to_mm
        vy = msg.pose.position.y * meter_to_mm
        vz = msg.pose.position.z * meter_to_mm
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        roll, pitch, yaw = R.from_quat(q).as_euler('xyz', degrees=False)

        left_vr_pose = [vx, vy, vz, roll, pitch, yaw]

        left_diff = [a - b for a, b in zip(left_vr_pose, self.init_left_vr_pose)]

        def euler_to_rotation_matrix(rx, ry, rz):
            cx = math.cos(rx); sx = math.sin(rx)
            cy = math.cos(ry); sy = math.sin(ry)
            cz = math.cos(rz); sz = math.sin(rz)
            Rx = np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
            Ry = np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
            Rz = np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
            return Ry.dot(Rx.dot(Rz))

        left_init_rot = euler_to_rotation_matrix(-self.init_left_vr_pose[3], -self.init_left_vr_pose[4], self.init_left_vr_pose[5])
        left_rot = euler_to_rotation_matrix(-left_vr_pose[3], -left_vr_pose[4], left_vr_pose[5])
        left_rotvr_diff = left_rot.dot(np.linalg.inv(left_init_rot))

        def scale_rotation_matrix_axis_angle(rotation_matrix, scale_factor):
            rot = R.from_matrix(rotation_matrix)
            rotvec = rot.as_rotvec()
            scaled = R.from_rotvec(rotvec * scale_factor)
            return scaled.as_matrix()

        rotation_scale_factor = 0.5
        if rotation_scale_factor != 1.0:
            left_rotvr_diff = scale_rotation_matrix_axis_angle(left_rotvr_diff, rotation_scale_factor)

        vr_rot = np.array([[0,0,-1],[-1,0,0],[0,1,0]])
        left_diff_base = vr_rot.dot(left_rotvr_diff).dot(np.linalg.inv(vr_rot))

        def rotation_matrix_z(angle):
            c = math.cos(angle); s = math.sin(angle)
            return np.array([[c,-s,0],[s,c,0],[0,0,1]])

        z_rot = rotation_matrix_z(math.radians(-30))
        left_diff_base = z_rot.dot(left_diff_base).dot(np.linalg.inv(z_rot))

        leftarm_finalrot = left_diff_base.dot(self.leftarm_init_rot)
        euler_angles_xyz_fixed = R.from_matrix(leftarm_finalrot).as_euler('xyz', degrees=False)

        next_pose = [0.0]*6
        next_pose[0] = self.leftarm_init_pos[0] + left_diff[2]
        next_pose[1] = self.leftarm_init_pos[1] - left_diff[0]
        next_pose[2] = self.leftarm_init_pos[2] + left_diff[1]
        next_pose[3] = euler_angles_xyz_fixed[0]
        next_pose[4] = euler_angles_xyz_fixed[1]
        next_pose[5] = euler_angles_xyz_fixed[2]
        # log the reference tcp and the computed next pose
        try:
            self.get_logger().info(f"[LEFT] leftarm_init_pos={self.leftarm_init_pos} next_pose={next_pose} degrees:({math.degrees(next_pose[3]):.1f}, {math.degrees(next_pose[4]):.1f}, {math.degrees(next_pose[5]):.1f})")
        except Exception:
            pass

        self.publish_servo_p_command(next_pose)

    def publish_servo_p_command(self, joint_positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = joint_positions
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
