#!/usr/bin/env python3
"""
VR Clutch Node - VR遥操作的Clutch模式控制节点

功能:
1. 订阅VR手柄位姿和按键
2. 订阅机器人当前末端位姿
3. 实现Clutch模式：按住grip跟踪，松开保持
4. 发布机器人目标位姿

Clutch模式工作原理:
- 按住Grip键: 建立VR-机器人位姿参考，跟踪VR增量
- 松开Grip键: 机器人保持最后位置
- VR增量直接映射为机器人增量（无需复杂坐标变换）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from qyh_jaka_control_msgs.msg import RobotState
import numpy as np

from .vr_clutch_controller import VRClutchController, ClutchConfig, ClutchState


class VRClutchNode(Node):
    """VR Clutch模式控制节点"""
    
    def __init__(self):
        super().__init__('vr_clutch_node')
        
        # 声明参数
        self._declare_parameters()
        
        # 仿真模式
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # 加载配置
        self.config = self._load_config()
        
        # 创建左右手的Clutch控制器，传入logger
        self.left_controller = VRClutchController(
            self.config, name="left", logger=self.get_logger())
        self.right_controller = VRClutchController(
            self.config, name="right", logger=self.get_logger())
        
        # 机器人当前状态
        self.robot_state: RobotState = None
        self.robot_state_received = False
        
        # 仿真模式的当前末端位姿（从 sim_arm_controller 订阅获取）
        self.sim_left_pose = Pose()
        self.sim_right_pose = Pose()
        self.sim_poses_received = False  # 是否已收到仿真末端位姿
        
        # VR当前状态
        self.left_vr_pose: PoseStamped = None
        self.right_vr_pose: PoseStamped = None
        self.left_grip_value: float = 0.0
        self.right_grip_value: float = 0.0
        
        # QoS设置
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # === 订阅者 ===
        # 机器人状态（仅非仿真模式）
        if not self.simulation_mode:
            self.robot_state_sub = self.create_subscription(
                RobotState,
                '/jaka/robot_state',
                self.robot_state_callback,
                10
            )
        else:
            self.robot_state_received = True  # 仿真模式直接设为已接收
            # 订阅仿真末端位姿（从 sim_arm_controller 发布）
            self.sim_left_pose_sub = self.create_subscription(
                PoseStamped,
                '/sim/left_current_pose',
                self.sim_left_pose_callback,
                10
            )
            self.sim_right_pose_sub = self.create_subscription(
                PoseStamped,
                '/sim/right_current_pose',
                self.sim_right_pose_callback,
                10
            )
        
        # VR手柄位姿 - vr_bridge_node 输出的已经是 ROS 坐标系
        left_pose_topic = '/vr/left_hand/pose'
        right_pose_topic = '/vr/right_hand/pose'

        self.left_pose_sub = self.create_subscription(
            PoseStamped,
            left_pose_topic,
            self.left_pose_callback,
            10
        )
        self.right_pose_sub = self.create_subscription(
            PoseStamped,
            right_pose_topic,
            self.right_pose_callback,
            10
        )
        
        # VR按键（获取grip值）
        self.left_joy_sub = self.create_subscription(
            Joy,
            '/vr/left_hand/joy',
            self.left_joy_callback,
            10
        )
        self.right_joy_sub = self.create_subscription(
            Joy,
            '/vr/right_hand/joy',
            self.right_joy_callback,
            10
        )
        
        # === 发布者 ===
        # 机器人目标位姿
        # 仿真模式: /sim/*_target_pose (给 sim_arm_controller)
        # 真机模式: /vr/*_target_pose (给 teleoperation_controller)
        if self.simulation_mode:
            left_target_topic = '/sim/left_target_pose'
            right_target_topic = '/sim/right_target_pose'
        else:
            left_target_topic = '/vr/left_target_pose'
            right_target_topic = '/vr/right_target_pose'
        
        self.left_target_pub = self.create_publisher(
            PoseStamped,
            left_target_topic,
            10
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped,
            right_target_topic,
            10
        )
        
        # Clutch状态（用于前端显示）
        self.left_clutch_status_pub = self.create_publisher(
            Bool,
            '/vr/left_clutch_engaged',
            10
        )
        self.right_clutch_status_pub = self.create_publisher(
            Bool,
            '/vr/right_clutch_engaged',
            10
        )
        
        # 控制循环定时器
        control_rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('=== VR Clutch Node Initialized ===')
        mode_str = "SIMULATION" if self.simulation_mode else "REAL ROBOT"
        self.get_logger().info(f'  Mode: {mode_str}')
        self.get_logger().info(f'  Control rate: {control_rate} Hz')
        self.get_logger().info(f'  Engage threshold: {self.config.engage_threshold}')
        self.get_logger().info(f'  Release threshold: {self.config.release_threshold}')
        self.get_logger().info(f'  Position scale: {self.config.position_scale}')
        self.get_logger().info(f'  Rotation scale: {self.config.rotation_scale}')
        self.get_logger().info(f'  VR pose topic: {left_pose_topic}')
        self.get_logger().info('  Note: VR data is already in ROS coordinate')
        if self.simulation_mode:
            self.get_logger().info('  Target topics: /sim/*_target_pose')
            self.get_logger().info('Waiting for VR data...')
        else:
            self.get_logger().info('  Target topics: /vr/*_target_pose')
            self.get_logger().info('Waiting for VR data and robot state...')
    
    def _declare_parameters(self):
        """声明ROS参数"""
        # 模式选择
        self.declare_parameter('simulation_mode', False)  # True=仿真, False=真机
        
        # 控制频率
        self.declare_parameter('control_rate', 50.0)  # Hz
        
        # Clutch阈值
        self.declare_parameter('clutch.engage_threshold', 0.8)
        self.declare_parameter('clutch.release_threshold', 0.2)
        
        # 仿真模式初始位姿（米）
        self.declare_parameter('sim.left_init_pose', [0.3, 0.2, 0.4, 0.0, 0.0, 0.0, 1.0])  # x,y,z,qx,qy,qz,qw
        self.declare_parameter('sim.right_init_pose', [0.3, -0.2, 0.4, 0.0, 0.0, 0.0, 1.0])
        
        # 缩放: 机械臂长度(1.2m) / 手臂长度(0.52m) ≈ 2.3
        self.declare_parameter('clutch.position_scale', 2.3)
        self.declare_parameter('clutch.rotation_scale', 1.0)
        
        # 增量限制
        self.declare_parameter('clutch.max_position_delta', 0.05)  # m
        self.declare_parameter('clutch.max_rotation_delta', 0.1)   # rad
        
        # 坐标轴映射 (VR -> Robot base_link)
        # [已废弃] 坐标变换现在在 vr_bridge_node (C++) 中完成
        # vr_bridge_node 输出的数据已经是 ROS 坐标系 (X前 Y左 Z上)
        # 这些参数仅用于向后兼容
        self.declare_parameter('clutch.axis_mapping', [2, 0, 1])
        self.declare_parameter('clutch.axis_signs', [-1, -1, 1])

        # [已废弃] use_transformed_pose 不再需要
        # vr_bridge_node 现在只输出 ROS 坐标系数据
        self.declare_parameter('use_transformed_pose', True)
    
    def _load_config(self) -> ClutchConfig:
        """加载Clutch配置"""
        return ClutchConfig(
            engage_threshold=self.get_parameter('clutch.engage_threshold').value,
            release_threshold=self.get_parameter('clutch.release_threshold').value,
            position_scale=self.get_parameter('clutch.position_scale').value,
            rotation_scale=self.get_parameter('clutch.rotation_scale').value,
            max_position_delta=self.get_parameter('clutch.max_position_delta').value,
            max_rotation_delta=self.get_parameter('clutch.max_rotation_delta').value
        )
    
    # === 回调函数 ===
    
    def robot_state_callback(self, msg: RobotState):
        """机器人状态回调（仅真机模式使用）"""
        self.robot_state = msg
        if not self.robot_state_received:
            self.robot_state_received = True
            self.get_logger().info('✓ Robot state received')
    
    def sim_left_pose_callback(self, msg: PoseStamped):
        """仿真模式: 左臂末端位姿回调"""
        self.sim_left_pose = msg.pose
        if not self.sim_poses_received:
            self.sim_poses_received = True
            self.get_logger().info(
                f'✓ Sim end-effector poses received: '
                f'left=[{msg.pose.position.x:.3f}, '
                f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]')
    
    def sim_right_pose_callback(self, msg: PoseStamped):
        """仿真模式: 右臂末端位姿回调"""
        self.sim_right_pose = msg.pose
    
    def left_pose_callback(self, msg: PoseStamped):
        """左手VR位姿回调"""
        self.left_vr_pose = msg
    
    def right_pose_callback(self, msg: PoseStamped):
        """右手VR位姿回调"""
        self.right_vr_pose = msg
    
    def left_joy_callback(self, msg: Joy):
        """左手按键回调"""
        if len(msg.axes) >= 4:
            self.left_grip_value = msg.axes[3]  # grip在axes[3]
            # 调试: 按下grip时输出
            if self.left_grip_value > 0.5:
                self.get_logger().info(
                    f'Left grip: {self.left_grip_value:.2f}',
                    throttle_duration_sec=0.5)
    
    def right_joy_callback(self, msg: Joy):
        """右手按键回调"""
        if len(msg.axes) >= 4:
            self.right_grip_value = msg.axes[3]  # grip在axes[3]
            if self.right_grip_value > 0.5:
                self.get_logger().info(
                    f'Right grip: {self.right_grip_value:.2f}',
                    throttle_duration_sec=0.5)
    
    # === 控制循环 ===
    
    def control_loop(self):
        """主控制循环"""
        # 检查是否有必要的数据
        if not self.robot_state_received:
            return
        
        # 仿真模式需要等待sim_arm_controller发布末端位姿
        if self.simulation_mode and not self.sim_poses_received:
            return
        
        now = self.get_clock().now()
        
        # 获取当前机械臂位姿（仿真模式从FK获取，真机模式用实际状态）
        if self.simulation_mode:
            left_robot_pose = self.sim_left_pose
            right_robot_pose = self.sim_right_pose
        else:
            left_robot_pose = self.robot_state.left_cartesian_pose
            right_robot_pose = self.robot_state.right_cartesian_pose
        
        # 处理左手
        if self.left_vr_pose is not None:
            self._process_arm(
                'left',
                self.left_controller,
                self.left_vr_pose,
                self.left_grip_value,
                left_robot_pose,
                self.left_target_pub,
                self.left_clutch_status_pub,
                now
            )
        
        # 处理右手
        if self.right_vr_pose is not None:
            self._process_arm(
                'right',
                self.right_controller,
                self.right_vr_pose,
                self.right_grip_value,
                right_robot_pose,
                self.right_target_pub,
                self.right_clutch_status_pub,
                now
            )
    
    def _process_arm(self, arm_name: str, 
                     controller: VRClutchController,
                     vr_pose: PoseStamped,
                     grip_value: float,
                     robot_pose: Pose,
                     target_pub,
                     clutch_pub,
                     now):
        """处理单臂的Clutch控制"""
        # 提取VR位姿
        vr_pos = np.array([
            vr_pose.pose.position.x,
            vr_pose.pose.position.y,
            vr_pose.pose.position.z
        ])
        vr_ori = np.array([
            vr_pose.pose.orientation.x,
            vr_pose.pose.orientation.y,
            vr_pose.pose.orientation.z,
            vr_pose.pose.orientation.w
        ])
        
        # 提取机器人当前位姿
        robot_pos = np.array([
            robot_pose.position.x,
            robot_pose.position.y,
            robot_pose.position.z
        ])
        robot_ori = np.array([
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w
        ])
        
        # ★★★ 调试：打印VR ROS位置 和 机械臂末端位置 ★★★
        if grip_value > 0.5:
            self.get_logger().info(
                f'[{arm_name}] VR_ROS=[{vr_pos[0]:.3f},{vr_pos[1]:.3f},{vr_pos[2]:.3f}] '
                f'EE=[{robot_pos[0]:.3f},{robot_pos[1]:.3f},{robot_pos[2]:.3f}]',
                throttle_duration_sec=0.5)
        
        # 更新Clutch控制器
        target_pos, target_ori, state = controller.update(
            vr_pos, vr_ori,
            robot_pos, robot_ori,
            grip_value
        )
        
        # ★★★ 调试输出：VR位置偏移 ★★★
        if controller.is_engaged() and controller.vr_origin_pos is not None:
            vr_offset = vr_pos - controller.vr_origin_pos
            self.get_logger().info(
                f'[{arm_name}] VR offset: '
                f'x={vr_offset[0]:+.4f}, y={vr_offset[1]:+.4f}, z={vr_offset[2]:+.4f}',
                throttle_duration_sec=0.5)
        
        # 发布Clutch状态
        clutch_msg = Bool()
        clutch_msg.data = controller.is_engaged()
        clutch_pub.publish(clutch_msg)
        
        # 如果有目标位姿，发布
        if target_pos is not None and target_ori is not None:
            target_msg = PoseStamped()
            target_msg.header.stamp = now.to_msg()
            target_msg.header.frame_id = 'base_link'
            
            target_msg.pose.position.x = float(target_pos[0])
            target_msg.pose.position.y = float(target_pos[1])
            target_msg.pose.position.z = float(target_pos[2])
            
            target_msg.pose.orientation.x = float(target_ori[0])
            target_msg.pose.orientation.y = float(target_ori[1])
            target_msg.pose.orientation.z = float(target_ori[2])
            target_msg.pose.orientation.w = float(target_ori[3])
            
            target_pub.publish(target_msg)
            
            # 打印转换后的目标位姿 (xyz + rpy)
            from scipy.spatial.transform import Rotation
            rpy = Rotation.from_quat(target_ori).as_euler('xyz', degrees=True)
            self.get_logger().info(
                f'[{arm_name}] Target: '
                f'xyz=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}] '
                f'rpy=[{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}]°',
                throttle_duration_sec=0.2)
            
            # 状态变化时打印日志
            if state == ClutchState.ENGAGING:
                self.get_logger().info(f'[{arm_name}] Clutch ENGAGED - grip={grip_value:.2f}')
            elif state == ClutchState.RELEASING:
                self.get_logger().info(f'[{arm_name}] Clutch RELEASED')


def main(args=None):
    rclpy.init(args=args)
    node = VRClutchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
