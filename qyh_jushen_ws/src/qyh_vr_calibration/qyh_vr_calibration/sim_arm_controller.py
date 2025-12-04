#!/usr/bin/env python3
"""
Simulation Arm Controller - 接收目标位姿，控制仿真机械臂

功能:
1. 订阅目标位姿 (/sim/left_target_pose, /sim/right_target_pose)
2. 使用MoveIt的MoveGroupInterface进行笛卡尔运动规划
3. 执行到目标位姿

这个节点将VR Clutch的输出转换为仿真机械臂的运动
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, RobotState as RobotStateMsg
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from threading import Lock


class SimArmController(Node):
    """仿真机械臂控制器"""
    
    def __init__(self):
        super().__init__('sim_arm_controller')
        
        # 参数
        self.declare_parameter('left_group_name', 'left_arm')
        self.declare_parameter('right_group_name', 'right_arm')
        self.declare_parameter('left_ee_link', 'left_tool0')
        self.declare_parameter('right_ee_link', 'right_tool0')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('orientation_tolerance', 0.1)
        self.declare_parameter('planning_time', 0.5)
        self.declare_parameter('max_velocity_scaling', 0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        
        self.left_group = self.get_parameter('left_group_name').value
        self.right_group = self.get_parameter('right_group_name').value
        self.left_ee = self.get_parameter('left_ee_link').value
        self.right_ee = self.get_parameter('right_ee_link').value
        self.planning_frame = self.get_parameter('planning_frame').value
        
        # 锁，防止同时规划
        self.left_lock = Lock()
        self.right_lock = Lock()
        
        # 当前目标
        self.left_target: PoseStamped = None
        self.right_target: PoseStamped = None
        self.left_target_new = False
        self.right_target_new = False
        
        # 当前关节状态
        self.current_joint_state: JointState = None
        
        # Callback groups
        self.cb_group = MutuallyExclusiveCallbackGroup()
        
        # === 订阅者 ===
        self.left_target_sub = self.create_subscription(
            PoseStamped, '/sim/left_target_pose',
            self.left_target_callback, 10)
        self.right_target_sub = self.create_subscription(
            PoseStamped, '/sim/right_target_pose',
            self.right_target_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10)
        
        # === Action Clients ===
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_group')
        
        # 等待MoveGroup可用
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available!')
        else:
            self.get_logger().info('MoveGroup action server connected')
        
        # 控制定时器（较低频率，给规划时间）
        self.timer = self.create_timer(
            0.1, self.control_loop,  # 10Hz
            callback_group=self.cb_group)
        
        self.get_logger().info('=== Simulation Arm Controller Started ===')
    
    def left_target_callback(self, msg: PoseStamped):
        self.left_target = msg
        self.left_target_new = True
    
    def right_target_callback(self, msg: PoseStamped):
        self.right_target = msg
        self.right_target_new = True
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def control_loop(self):
        """控制循环"""
        # 处理左臂
        if self.left_target_new and self.left_target is not None:
            with self.left_lock:
                self.left_target_new = False
                self.plan_and_execute(
                    self.left_group, self.left_ee, self.left_target)
        
        # 处理右臂
        if self.right_target_new and self.right_target is not None:
            with self.right_lock:
                self.right_target_new = False
                self.plan_and_execute(
                    self.right_group, self.right_ee, self.right_target)
    
    def plan_and_execute(self, group_name: str, ee_link: str, target: PoseStamped):
        """规划并执行到目标位姿"""
        if self.current_joint_state is None:
            return
        
        # 构建MotionPlanRequest
        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = 1
        request.allowed_planning_time = self.get_parameter('planning_time').value
        request.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling').value
        request.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling').value
        
        # 设置起始状态
        request.start_state.joint_state = self.current_joint_state
        
        # 目标约束
        goal_constraints = Constraints()
        goal_constraints.name = "goal"
        
        # 位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.planning_frame
        pos_constraint.link_name = ee_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        # 边界区域（球形）
        bounding = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.get_parameter('position_tolerance').value]
        bounding.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position = target.pose.position
        sphere_pose.orientation.w = 1.0
        bounding.primitive_poses.append(sphere_pose)
        
        pos_constraint.constraint_region = bounding
        pos_constraint.weight = 1.0
        goal_constraints.position_constraints.append(pos_constraint)
        
        # 姿态约束
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.planning_frame
        ori_constraint.link_name = ee_link
        ori_constraint.orientation = target.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = self.get_parameter('orientation_tolerance').value
        ori_constraint.absolute_y_axis_tolerance = self.get_parameter('orientation_tolerance').value
        ori_constraint.absolute_z_axis_tolerance = self.get_parameter('orientation_tolerance').value
        ori_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(ori_constraint)
        
        request.goal_constraints.append(goal_constraints)
        
        # 发送请求
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False
        
        # 异步发送
        future = self.move_group_client.send_goal_async(goal_msg)
        # 不等待结果，让它在后台执行


def main(args=None):
    rclpy.init(args=args)
    node = SimArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
