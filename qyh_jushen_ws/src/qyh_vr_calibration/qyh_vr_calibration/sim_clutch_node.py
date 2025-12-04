#!/usr/bin/env python3
"""
Simulation Clutch Node - 用于仿真测试的VR Clutch控制节点

与真机版本(vr_clutch_node.py)的区别:
1. 不依赖 qyh_jaka_control_msgs.RobotState
2. 使用MoveIt获取机器人当前末端位姿
3. 通过MoveIt servo或直接发布关节状态来控制仿真机械臂

这个节点用于在RViz中可视化测试VR Clutch功能
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
import numpy as np

from .vr_clutch_controller import VRClutchController, ClutchConfig, ClutchState


class SimClutchNode(Node):
    """仿真用的VR Clutch控制节点"""
    
    def __init__(self):
        super().__init__('sim_clutch_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 声明参数
        self._declare_parameters()
        
        # 加载配置
        self.config = self._load_config()
        
        # 创建左右手的Clutch控制器
        self.left_controller = VRClutchController(self.config, name="left")
        self.right_controller = VRClutchController(self.config, name="right")
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 末端执行器frame名称
        self.left_ee_frame = self.get_parameter('left_ee_frame').value
        self.right_ee_frame = self.get_parameter('right_ee_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # VR当前状态
        self.left_vr_pose: PoseStamped = None
        self.right_vr_pose: PoseStamped = None
        self.left_grip_value: float = 0.0
        self.right_grip_value: float = 0.0
        
        # 机器人当前位姿（从TF获取）
        self.left_robot_pos = np.array([0.3, 0.3, 0.5])
        self.left_robot_ori = np.array([0.0, 0.0, 0.0, 1.0])
        self.right_robot_pos = np.array([0.3, -0.3, 0.5])
        self.right_robot_ori = np.array([0.0, 0.0, 0.0, 1.0])
        
        # === 订阅者 ===
        # VR手柄位姿
        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/vr/left_hand/pose',
            self.left_pose_callback, 10)
        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/vr/right_hand/pose',
            self.right_pose_callback, 10)
        
        # VR按键
        self.left_joy_sub = self.create_subscription(
            Joy, '/vr/left_hand/joy',
            self.left_joy_callback, 10)
        self.right_joy_sub = self.create_subscription(
            Joy, '/vr/right_hand/joy',
            self.right_joy_callback, 10)
        
        # === 发布者 ===
        # 目标位姿（给可视化使用）
        self.left_target_pub = self.create_publisher(
            PoseStamped, '/sim/left_target_pose', 10)
        self.right_target_pub = self.create_publisher(
            PoseStamped, '/sim/right_target_pose', 10)
        
        # Clutch状态
        self.left_clutch_pub = self.create_publisher(
            Bool, '/sim/left_clutch_engaged', 10)
        self.right_clutch_pub = self.create_publisher(
            Bool, '/sim/right_clutch_engaged', 10)
        
        # 调试状态
        self.status_pub = self.create_publisher(
            String, '/sim/clutch_status', 10)
        
        # 控制定时器
        control_rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(
            1.0 / control_rate, self.control_loop,
            callback_group=self.callback_group)
        
        # TF更新定时器
        self.tf_timer = self.create_timer(
            0.02, self.update_robot_pose_from_tf,
            callback_group=self.callback_group)
        
        self.get_logger().info('=== Simulation Clutch Node Started ===')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
        self.get_logger().info(f'  Left EE frame: {self.left_ee_frame}')
        self.get_logger().info(f'  Right EE frame: {self.right_ee_frame}')
        self.get_logger().info('Waiting for VR data...')
    
    def _declare_parameters(self):
        """声明ROS参数"""
        self.declare_parameter('control_rate', 50.0)
        
        # Frame names
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('left_ee_frame', 'left_tool0')
        self.declare_parameter('right_ee_frame', 'right_tool0')
        
        # Clutch配置
        self.declare_parameter('clutch.engage_threshold', 0.8)
        self.declare_parameter('clutch.release_threshold', 0.2)
        self.declare_parameter('clutch.position_scale', 1.0)
        self.declare_parameter('clutch.rotation_scale', 1.0)
        self.declare_parameter('clutch.max_position_delta', 0.02)
        self.declare_parameter('clutch.max_rotation_delta', 0.05)
        self.declare_parameter('clutch.position_deadzone', 0.002)
        self.declare_parameter('clutch.rotation_deadzone', 0.01)
        self.declare_parameter('clutch.smoothing_factor', 0.5)
        self.declare_parameter('clutch.axis_mapping', [0, 1, 2])
        self.declare_parameter('clutch.axis_signs', [1, 1, 1])
    
    def _load_config(self) -> ClutchConfig:
        """加载配置"""
        axis_mapping = self.get_parameter('clutch.axis_mapping').value
        axis_signs = self.get_parameter('clutch.axis_signs').value
        
        return ClutchConfig(
            engage_threshold=self.get_parameter('clutch.engage_threshold').value,
            release_threshold=self.get_parameter('clutch.release_threshold').value,
            position_scale=self.get_parameter('clutch.position_scale').value,
            rotation_scale=self.get_parameter('clutch.rotation_scale').value,
            max_position_delta=self.get_parameter('clutch.max_position_delta').value,
            max_rotation_delta=self.get_parameter('clutch.max_rotation_delta').value,
            position_deadzone=self.get_parameter('clutch.position_deadzone').value,
            rotation_deadzone=self.get_parameter('clutch.rotation_deadzone').value,
            smoothing_factor=self.get_parameter('clutch.smoothing_factor').value,
            axis_mapping=tuple(axis_mapping),
            axis_signs=tuple(axis_signs)
        )
    
    # === 回调 ===
    
    def left_pose_callback(self, msg: PoseStamped):
        self.left_vr_pose = msg
    
    def right_pose_callback(self, msg: PoseStamped):
        self.right_vr_pose = msg
    
    def left_joy_callback(self, msg: Joy):
        if len(msg.axes) >= 4:
            self.left_grip_value = msg.axes[3]
    
    def right_joy_callback(self, msg: Joy):
        if len(msg.axes) >= 4:
            self.right_grip_value = msg.axes[3]
    
    def update_robot_pose_from_tf(self):
        """从TF获取机器人末端位姿"""
        try:
            # 左臂
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.left_ee_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            self.left_robot_pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            self.left_robot_ori = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
        except Exception:
            pass
        
        try:
            # 右臂
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.right_ee_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            self.right_robot_pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])
            self.right_robot_ori = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])
        except Exception:
            pass
    
    def control_loop(self):
        """控制循环"""
        now = self.get_clock().now()
        
        left_state = "IDLE"
        right_state = "IDLE"
        
        # 处理左手
        if self.left_vr_pose is not None:
            vr_pos = np.array([
                self.left_vr_pose.pose.position.x,
                self.left_vr_pose.pose.position.y,
                self.left_vr_pose.pose.position.z
            ])
            vr_ori = np.array([
                self.left_vr_pose.pose.orientation.x,
                self.left_vr_pose.pose.orientation.y,
                self.left_vr_pose.pose.orientation.z,
                self.left_vr_pose.pose.orientation.w
            ])
            
            target_pos, target_ori, state = self.left_controller.update(
                vr_pos, vr_ori,
                self.left_robot_pos, self.left_robot_ori,
                self.left_grip_value
            )
            
            left_state = state.name
            
            # 发布Clutch状态
            clutch_msg = Bool()
            clutch_msg.data = self.left_controller.is_engaged()
            self.left_clutch_pub.publish(clutch_msg)
            
            # 发布目标位姿
            if target_pos is not None:
                self._publish_target(
                    self.left_target_pub, target_pos, target_ori, now)
        
        # 处理右手
        if self.right_vr_pose is not None:
            vr_pos = np.array([
                self.right_vr_pose.pose.position.x,
                self.right_vr_pose.pose.position.y,
                self.right_vr_pose.pose.position.z
            ])
            vr_ori = np.array([
                self.right_vr_pose.pose.orientation.x,
                self.right_vr_pose.pose.orientation.y,
                self.right_vr_pose.pose.orientation.z,
                self.right_vr_pose.pose.orientation.w
            ])
            
            target_pos, target_ori, state = self.right_controller.update(
                vr_pos, vr_ori,
                self.right_robot_pos, self.right_robot_ori,
                self.right_grip_value
            )
            
            right_state = state.name
            
            clutch_msg = Bool()
            clutch_msg.data = self.right_controller.is_engaged()
            self.right_clutch_pub.publish(clutch_msg)
            
            if target_pos is not None:
                self._publish_target(
                    self.right_target_pub, target_pos, target_ori, now)
        
        # 发布状态
        status = String()
        status.data = (
            f"L: {left_state} grip={self.left_grip_value:.1f} | "
            f"R: {right_state} grip={self.right_grip_value:.1f}"
        )
        self.status_pub.publish(status)
    
    def _publish_target(self, pub, pos, ori, stamp):
        """发布目标位姿"""
        msg = PoseStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(ori[0])
        msg.pose.orientation.y = float(ori[1])
        msg.pose.orientation.z = float(ori[2])
        msg.pose.orientation.w = float(ori[3])
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimClutchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
