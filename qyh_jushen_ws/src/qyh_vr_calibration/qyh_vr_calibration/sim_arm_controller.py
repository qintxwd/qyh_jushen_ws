#!/usr/bin/env python3
"""
Simulation Arm Controller - 接收目标位姿，控制仿真机械臂

功能:
1. 订阅目标位姿 (/sim/left_target_pose, /sim/right_target_pose)
2. 使用MoveIt的IK服务计算关节角度
3. 发布fake关节状态，驱动RViz显示

这个节点将VR Clutch的输出转换为仿真机械臂的运动
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK


class SimArmController(Node):
    """仿真机械臂控制器 - 使用IK服务"""
    
    def __init__(self):
        super().__init__('sim_arm_controller')
        
        # 参数
        self.declare_parameter('left_group_name', 'left_arm')
        self.declare_parameter('right_group_name', 'right_arm')
        self.declare_parameter('left_ee_link', 'left_tool0')
        self.declare_parameter('right_ee_link', 'right_tool0')
        
        self.left_group = self.get_parameter('left_group_name').value
        self.right_group = self.get_parameter('right_group_name').value
        self.left_ee = self.get_parameter('left_ee_link').value
        self.right_ee = self.get_parameter('right_ee_link').value
        
        # 当前目标
        self.left_target: PoseStamped = None
        self.right_target: PoseStamped = None
        self.left_target_new = False
        self.right_target_new = False
        
        # IK请求进行中标志
        self.left_ik_pending = False
        self.right_ik_pending = False
        
        # 当前关节状态
        self.current_joint_state: JointState = None
        
        # 左右臂的关节名称
        self.left_joints = [f'left_joint{i}' for i in range(1, 8)]
        self.right_joints = [f'right_joint{i}' for i in range(1, 8)]
        
        # 当前关节位置
        self.left_positions = [0.0] * 7
        self.right_positions = [0.0] * 7
        
        # 使用可重入回调组
        self.cb_group = ReentrantCallbackGroup()
        
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
        
        # === IK服务客户端 ===
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.cb_group)
        
        self.get_logger().info('Waiting for IK service (/compute_ik)...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
        else:
            self.get_logger().info('IK service connected')
        
        # === 发布者 ===
        self.fake_joint_pub = self.create_publisher(
            JointState, '/move_group/fake_controller_joint_states', 10)
        
        # 发布当前末端位姿（供vr_clutch_node使用）
        self.left_ee_pose_pub = self.create_publisher(
            PoseStamped, '/sim/left_current_pose', 10)
        self.right_ee_pose_pub = self.create_publisher(
            PoseStamped, '/sim/right_current_pose', 10)
        
        # FK服务客户端
        self.fk_client = self.create_client(
            GetPositionFK, '/compute_fk',
            callback_group=self.cb_group)
        
        # FK定时器（发布末端位姿）
        self.fk_timer = self.create_timer(0.1, self.publish_ee_poses)  # 10Hz
        
        # 控制定时器
        self.timer = self.create_timer(
            0.05, self.control_loop,
            callback_group=self.cb_group)  # 20Hz
        
        self.get_logger().info('=== Simulation Arm Controller Started ===')
        self.get_logger().info(f'  Left arm: {self.left_group}')
        self.get_logger().info(f'  Right arm: {self.right_group}')
    
    def left_target_callback(self, msg: PoseStamped):
        self.left_target = msg
        self.left_target_new = True
        self.get_logger().info(
            f'Received left target: pos=[{msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]',
            throttle_duration_sec=1.0)
    
    def right_target_callback(self, msg: PoseStamped):
        self.right_target = msg
        self.right_target_new = True
        self.get_logger().info(
            f'Received right target: pos=[{msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]',
            throttle_duration_sec=1.0)
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        # 更新当前关节位置
        for i, name in enumerate(msg.name):
            if name in self.left_joints:
                idx = self.left_joints.index(name)
                self.left_positions[idx] = msg.position[i]
            elif name in self.right_joints:
                idx = self.right_joints.index(name)
                self.right_positions[idx] = msg.position[i]
    
    def control_loop(self):
        """控制循环"""
        # 处理左臂
        if self.left_target_new and not self.left_ik_pending:
            self.left_target_new = False
            self.get_logger().info('Sending IK request for left arm')
            self.send_ik_request('left', self.left_group, self.left_target)
        
        # 处理右臂
        if self.right_target_new and not self.right_ik_pending:
            self.right_target_new = False
            self.get_logger().info('Sending IK request for right arm')
            self.send_ik_request('right', self.right_group, self.right_target)
    
    def send_ik_request(self, arm: str, group_name: str, target: PoseStamped):
        """发送异步IK请求"""
        if self.current_joint_state is None:
            self.get_logger().warn('No joint state received yet')
            return
        
        # 设置pending标志
        if arm == 'left':
            self.left_ik_pending = True
        else:
            self.right_ik_pending = True
        
        # 构建IK请求
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.robot_state.joint_state = self.current_joint_state
        request.ik_request.pose_stamped = target
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 100000000  # 100ms
        request.ik_request.avoid_collisions = False  # 仿真不检查碰撞
        
        # 异步调用
        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda f: self.ik_response_callback(f, arm))
    
    def ik_response_callback(self, future, arm: str):
        """IK响应回调"""
        # 清除pending标志
        if arm == 'left':
            self.left_ik_pending = False
            joint_names = self.left_joints
            current_positions = self.left_positions
        else:
            self.right_ik_pending = False
            joint_names = self.right_joints
            current_positions = self.right_positions
        
        try:
            response = future.result()
            self.get_logger().info(
                f'IK response for {arm}: error_code={response.error_code.val}')
            
            if response.error_code.val == 1:  # SUCCESS
                # 提取目标关节位置
                new_positions = []
                for jn in joint_names:
                    if jn in response.solution.joint_state.name:
                        idx = list(response.solution.joint_state.name).index(jn)
                        new_positions.append(
                            response.solution.joint_state.position[idx])
                    else:
                        idx = joint_names.index(jn)
                        new_positions.append(current_positions[idx])
                
                # 发布fake关节状态
                fake_msg = JointState()
                fake_msg.header.stamp = self.get_clock().now().to_msg()
                fake_msg.name = joint_names
                fake_msg.position = new_positions
                self.fake_joint_pub.publish(fake_msg)
                
                self.get_logger().info(
                    f'Published fake joint states for {arm}')
                
                # 更新本地状态
                for i, pos in enumerate(new_positions):
                    current_positions[i] = pos
            else:
                self.get_logger().warn(
                    f'IK failed for {arm}: code={response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'IK callback error: {e}')


    def publish_ee_poses(self):
        """发布当前末端位姿"""
        if self.current_joint_state is None:
            return
        
        # 发布左臂末端位姿
        self._request_fk('left', self.left_group, 'left_tool0', self.left_ee_pose_pub)
        # 发布右臂末端位姿
        self._request_fk('right', self.right_group, 'right_tool0', self.right_ee_pose_pub)
    
    def _request_fk(self, arm: str, group_name: str, ee_link: str, pub):
        """请求FK计算并发布末端位姿"""
        if not self.fk_client.service_is_ready():
            return
        
        request = GetPositionFK.Request()
        request.header.frame_id = 'base_link'
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = [ee_link]
        request.robot_state.joint_state = self.current_joint_state
        
        # 同步调用FK（速度快，可以同步）
        try:
            future = self.fk_client.call_async(request)
            # 不等待结果，使用回调
            future.add_done_callback(
                lambda f: self._fk_callback(f, arm, pub))
        except Exception as e:
            self.get_logger().error(f'FK request error: {e}')
    
    def _fk_callback(self, future, arm: str, pub):
        """FK响应回调"""
        try:
            response = future.result()
            if response.error_code.val == 1 and len(response.pose_stamped) > 0:
                pose_msg = response.pose_stamped[0]
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pub.publish(pose_msg)
        except Exception as e:
            pass  # FK失败静默处理


def main(args=None):
    rclpy.init(args=args)
    node = SimArmController()
    
    # 使用多线程执行器支持异步回调
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
