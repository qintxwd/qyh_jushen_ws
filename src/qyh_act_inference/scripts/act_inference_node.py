#!/usr/bin/env python3
"""
ACT 推理 ROS2 节点

主节点，负责：
1. 订阅观测数据（图像 + 关节状态）
2. 执行 ACT 推理
3. 发布/调用机器人控制命令

使用方法:
    ros2 run qyh_act_inference act_inference_node.py --ros-args -p model_path:=/path/to/policy.pt
"""

import os
import time
import threading
import numpy as np
from collections import deque
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# ROS2 消息类型
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose

# 尝试导入 cv_bridge
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False
    print("[Warning] cv_bridge not available, image processing disabled")

# 项目模块
from qyh_act_inference import ACTPolicy, ActionExecutor, InferenceConfig
from qyh_act_inference.act_policy import Observation


class ACTInferenceNode(Node):
    """
    ACT 推理节点
    """
    
    def __init__(self):
        super().__init__('act_inference_node')
        
        self.get_logger().info("="*50)
        self.get_logger().info("ACT Inference Node Starting...")
        self.get_logger().info("="*50)
        
        # 加载配置
        self.config = InferenceConfig.from_ros_params(self)
        self.get_logger().info(f"Config: {self.config.to_dict()}")
        
        # 初始化组件
        self.policy = ACTPolicy(self.config)
        self.executor = ActionExecutor(self.config)
        
        if HAS_CV_BRIDGE:
            self.cv_bridge = CvBridge()
        else:
            self.cv_bridge = None
        
        # 回调组
        self.sensor_cb_group = ReentrantCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 状态变量
        self._is_running = False
        self._is_model_loaded = False
        self._last_inference_time = 0.0
        self._current_model_name = ""  # 当前加载的模型名称
        
        # 当前观测数据
        self._current_obs = Observation()
        self._obs_lock = threading.Lock()
        
        # 数据时间戳（用于过期检测）
        self._head_image_timestamp = 0.0
        self._left_wrist_image_timestamp = 0.0
        self._right_wrist_image_timestamp = 0.0
        self._left_arm_timestamp = 0.0
        self._right_arm_timestamp = 0.0
        
        # 数据过期阈值（秒）
        self._image_staleness_threshold = 0.5
        self._joint_staleness_threshold = 0.2
        
        # 当前关节状态（用于增量计算）
        self._left_arm_joints: Optional[np.ndarray] = None
        self._right_arm_joints: Optional[np.ndarray] = None
        self._left_gripper_state: float = 0.0
        self._right_gripper_state: float = 0.0
        
        # 创建订阅者
        self._create_subscribers()
        
        # 创建发布者/服务客户端
        self._create_publishers()
        self._create_service_clients()
        
        # 创建控制接口
        self._create_control_interface()
        
        # 控制定时器（高频）
        self._control_timer = self.create_timer(
            self.config.control_period,
            self._control_loop,
            callback_group=self.control_cb_group
        )
        
        # 推理定时器（可能低于控制频率）
        self._inference_timer = self.create_timer(
            self.config.inference_period,
            self._inference_loop,
            callback_group=self.control_cb_group
        )
        
        self.get_logger().info("ACT Inference Node initialized")
    
    def _create_subscribers(self):
        """创建观测数据订阅者"""
        
        # 左臂关节状态
        if self.config.use_left_arm:
            self._left_arm_sub = self.create_subscription(
                JointState,
                self.config.left_arm_state_topic,
                self._left_arm_callback,
                10,
                callback_group=self.sensor_cb_group
            )
            self.get_logger().info(f"Subscribed to: {self.config.left_arm_state_topic}")
        
        # 右臂关节状态
        if self.config.use_right_arm:
            self._right_arm_sub = self.create_subscription(
                JointState,
                self.config.right_arm_state_topic,
                self._right_arm_callback,
                10,
                callback_group=self.sensor_cb_group
            )
            self.get_logger().info(f"Subscribed to: {self.config.right_arm_state_topic}")
        
        # 头部相机
        if self.config.use_head_camera and HAS_CV_BRIDGE:
            self._head_cam_sub = self.create_subscription(
                Image,
                self.config.head_camera_topic,
                self._head_camera_callback,
                10,
                callback_group=self.sensor_cb_group
            )
            self.get_logger().info(f"Subscribed to: {self.config.head_camera_topic}")
        
        # 腕部相机（可选）
        if self.config.use_wrist_cameras and HAS_CV_BRIDGE:
            self._left_wrist_cam_sub = self.create_subscription(
                Image,
                self.config.left_wrist_camera_topic,
                self._left_wrist_camera_callback,
                10,
                callback_group=self.sensor_cb_group
            )
            self._right_wrist_cam_sub = self.create_subscription(
                Image,
                self.config.right_wrist_camera_topic,
                self._right_wrist_camera_callback,
                10,
                callback_group=self.sensor_cb_group
            )
        
        # 头部深度相机（可选）
        if self.config.use_depth and HAS_CV_BRIDGE:
            self._head_depth_sub = self.create_subscription(
                Image,
                self.config.head_camera_depth_topic,
                self._head_depth_callback,
                10,
                callback_group=self.sensor_cb_group
            )
            self.get_logger().info(f"Subscribed to depth: {self.config.head_camera_depth_topic}")
    
    def _create_publishers(self):
        """创建命令发布者"""
        
        # 状态发布
        self._status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )
        
        # 夹爪命令发布
        if self.config.use_left_gripper:
            # 使用项目的夹爪消息类型
            try:
                from qyh_gripper_msgs.msg import GripperCommand
                self._left_gripper_pub = self.create_publisher(
                    GripperCommand,
                    self.config.left_gripper_command_topic,
                    10
                )
            except ImportError:
                self.get_logger().warn("qyh_gripper_msgs not found, using Float32")
                self._left_gripper_pub = self.create_publisher(
                    Float32,
                    self.config.left_gripper_command_topic,
                    10
                )
        
        if self.config.use_right_gripper:
            try:
                from qyh_gripper_msgs.msg import GripperCommand
                self._right_gripper_pub = self.create_publisher(
                    GripperCommand,
                    self.config.right_gripper_command_topic,
                    10
                )
            except ImportError:
                self._right_gripper_pub = self.create_publisher(
                    Float32,
                    self.config.right_gripper_command_topic,
                    10
                )
    
    def _create_service_clients(self):
        """创建机械臂控制服务客户端"""
        
        try:
            from qyh_jaka_control_msgs.srv import ServoJ
            
            self._servo_j_client = self.create_client(
                ServoJ,
                self.config.left_arm_command_service,
                callback_group=self.control_cb_group
            )
            self.get_logger().info(f"Created ServoJ client: {self.config.left_arm_command_service}")
            
        except ImportError:
            self.get_logger().warn("qyh_jaka_control_msgs not found, arm control disabled")
            self._servo_j_client = None
    
    def _create_control_interface(self):
        """创建外部控制接口（启动/停止推理）"""
        
        from std_srvs.srv import SetBool, Trigger
        
        # 启动/停止服务
        self._start_srv = self.create_service(
            SetBool,
            '~/start',
            self._handle_start,
            callback_group=self.control_cb_group
        )
        
        # 加载模型服务
        self._load_model_srv = self.create_service(
            Trigger,
            '~/load_model',
            self._handle_load_model,
            callback_group=self.control_cb_group
        )
        
        # 重置服务
        self._reset_srv = self.create_service(
            Trigger,
            '~/reset',
            self._handle_reset,
            callback_group=self.control_cb_group
        )
        
        # 模型路径设置话题（用于动态切换模型）
        self._model_path_sub = self.create_subscription(
            String,
            '~/set_model_path',
            self._handle_set_model_path,
            10,
            callback_group=self.control_cb_group
        )
        
        # 当前模型名称发布
        self._model_name_pub = self.create_publisher(
            String,
            '~/current_model',
            10
        )
    
    # ==================== 观测回调 ====================
    
    def _left_arm_callback(self, msg: JointState):
        """左臂关节状态回调"""
        with self._obs_lock:
            self._left_arm_joints = np.array(msg.position[:7])
            self._current_obs.left_arm_joints = self._left_arm_joints.copy()
            self._left_arm_timestamp = time.time()
    
    def _right_arm_callback(self, msg: JointState):
        """右臂关节状态回调"""
        with self._obs_lock:
            self._right_arm_joints = np.array(msg.position[:7])
            self._current_obs.right_arm_joints = self._right_arm_joints.copy()
            self._right_arm_timestamp = time.time()
    
    def _head_camera_callback(self, msg: Image):
        """头部相机回调"""
        if not self.cv_bridge:
            return
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            with self._obs_lock:
                self._current_obs.head_image = cv_image
                self._head_image_timestamp = time.time()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
    
    def _left_wrist_camera_callback(self, msg: Image):
        """左腕相机回调"""
        if not self.cv_bridge:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            with self._obs_lock:
                self._current_obs.left_wrist_image = cv_image
                self._left_wrist_image_timestamp = time.time()
        except Exception as e:
            pass
    
    def _right_wrist_camera_callback(self, msg: Image):
        """右腕相机回调"""
        if not self.cv_bridge:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            with self._obs_lock:
                self._current_obs.right_wrist_image = cv_image
                self._right_wrist_image_timestamp = time.time()
        except Exception as e:
            pass
    
    def _head_depth_callback(self, msg: Image):
        """头部深度相机回调"""
        if not self.cv_bridge:
            return
        try:
            # 深度图像通常是 16UC1（毫米）或 32FC1（米）
            if msg.encoding == '16UC1':
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "16UC1")
                # 转换为 float32，单位毫米
                depth_image = depth_image.astype(np.float32)
            elif msg.encoding == '32FC1':
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")
                # 米转毫米
                depth_image = depth_image * 1000.0
            else:
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                depth_image = depth_image.astype(np.float32)
            
            with self._obs_lock:
                self._current_obs.head_depth = depth_image
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
    
    # ==================== 控制循环 ====================
    
    def _check_data_staleness(self) -> bool:
        """
        检查观测数据是否过期
        
        Returns:
            True: 数据新鲜可用
            False: 数据过期，不应进行推理
        """
        now = time.time()
        
        # 检查头部相机（如果启用）
        if self.config.use_head_camera and self._head_image_timestamp > 0:
            staleness = now - self._head_image_timestamp
            if staleness > self._image_staleness_threshold:
                self.get_logger().error(
                    f"Head camera STALE! Last update: {staleness:.2f}s ago. Safety stop."
                )
                return False
        
        # 检查左臂关节（如果启用）
        if self.config.use_left_arm and self._left_arm_timestamp > 0:
            staleness = now - self._left_arm_timestamp
            if staleness > self._joint_staleness_threshold:
                self.get_logger().error(
                    f"Left arm joints STALE! Last update: {staleness:.2f}s ago. Safety stop."
                )
                return False
        
        # 检查右臂关节（如果启用）
        if self.config.use_right_arm and self._right_arm_timestamp > 0:
            staleness = now - self._right_arm_timestamp
            if staleness > self._joint_staleness_threshold:
                self.get_logger().error(
                    f"Right arm joints STALE! Last update: {staleness:.2f}s ago. Safety stop."
                )
                return False
        
        return True
    
    def _inference_loop(self):
        """推理循环（低频）"""
        if not self._is_running or not self._is_model_loaded:
            return
        
        # 安全检查：数据是否过期
        if not self._check_data_staleness():
            # 数据过期，清空动作队列，停止执行
            self.executor.reset()
            return
        
        # 添加当前观测到缓冲区
        with self._obs_lock:
            obs = Observation(
                left_arm_joints=self._current_obs.left_arm_joints.copy() if self._current_obs.left_arm_joints is not None else None,
                right_arm_joints=self._current_obs.right_arm_joints.copy() if self._current_obs.right_arm_joints is not None else None,
                left_gripper=self._left_gripper_state,
                right_gripper=self._right_gripper_state,
                head_image=self._current_obs.head_image.copy() if self._current_obs.head_image is not None else None,
                timestamp=self._head_image_timestamp  # 使用实际的图像时间戳
            )
        
        self.policy.add_observation(obs)
        
        # 检查是否可以推理
        if not self.policy.is_ready():
            return
        
        # 执行推理
        start_time = time.time()
        action_chunk = self.policy.infer()
        inference_time = time.time() - start_time
        
        if action_chunk is None:
            self.get_logger().warn("Inference returned None")
            return
        
        # 设置动作序列
        self.executor.set_action_chunk(action_chunk[0])  # 取 batch 的第一个
        
        if self.config.verbose:
            self.get_logger().info(f"Inference time: {inference_time*1000:.1f}ms, action_shape: {action_chunk.shape}")
    
    def _control_loop(self):
        """控制循环（高频）"""
        if not self._is_running:
            return
        
        # 从执行器获取下一个命令
        if not self.executor.has_pending_actions():
            return
        
        command = self.executor.get_next_command()
        if command is None:
            return
        
        # 执行命令
        self._execute_command(command)
    
    def _execute_command(self, command):
        """
        执行机器人命令
        
        Args:
            command: RobotCommand 对象
        """
        # 执行右臂命令
        if command.right_arm is not None and self._servo_j_client:
            self._send_arm_command(
                command.right_arm,
                robot_id=2,  # 右臂
                current_joints=self._right_arm_joints
            )
        
        # 执行左臂命令
        if command.left_arm is not None and self._servo_j_client:
            self._send_arm_command(
                command.left_arm,
                robot_id=1,  # 左臂
                current_joints=self._left_arm_joints
            )
        
        # 执行右夹爪命令
        if command.right_gripper is not None:
            self._send_gripper_command(command.right_gripper, 'right')
        
        # 执行左夹爪命令
        if command.left_gripper is not None:
            self._send_gripper_command(command.left_gripper, 'left')
    
    def _send_arm_command(self, arm_cmd, robot_id: int, current_joints: Optional[np.ndarray]):
        """发送机械臂命令"""
        if current_joints is None:
            self.get_logger().warn(f"No current joint state for robot {robot_id}")
            return
        
        try:
            from qyh_jaka_control_msgs.srv import ServoJ
            
            # 计算目标位置
            if arm_cmd.is_delta:
                target_joints = current_joints + arm_cmd.joint_positions
            else:
                target_joints = arm_cmd.joint_positions
            
            # 创建请求
            request = ServoJ.Request()
            request.robot_id = robot_id
            request.joint_positions = target_joints.tolist()
            
            # 异步调用（不等待结果）
            self._servo_j_client.call_async(request)
            
        except Exception as e:
            self.get_logger().error(f"Failed to send arm command: {e}")
    
    def _send_gripper_command(self, gripper_cmd, side: str):
        """发送夹爪命令"""
        try:
            from qyh_gripper_msgs.msg import GripperCommand
            
            msg = GripperCommand()
            msg.position = gripper_cmd.position
            
            if side == 'left' and hasattr(self, '_left_gripper_pub'):
                self._left_gripper_pub.publish(msg)
                self._left_gripper_state = gripper_cmd.position
            elif side == 'right' and hasattr(self, '_right_gripper_pub'):
                self._right_gripper_pub.publish(msg)
                self._right_gripper_state = gripper_cmd.position
                
        except ImportError:
            # 降级为 Float32
            msg = Float32()
            msg.data = gripper_cmd.position
            
            if side == 'left' and hasattr(self, '_left_gripper_pub'):
                self._left_gripper_pub.publish(msg)
            elif side == 'right' and hasattr(self, '_right_gripper_pub'):
                self._right_gripper_pub.publish(msg)
    
    # ==================== 服务处理 ====================
    
    def _handle_set_model_path(self, msg: String):
        """处理模型路径设置"""
        new_path = msg.data.strip()
        if new_path:
            self.get_logger().info(f"Setting model path to: {new_path}")
            self.config.model_path = new_path
            # 提取模型名称（用于标识）
            self._current_model_name = os.path.basename(os.path.dirname(new_path))
    
    def _handle_start(self, request, response):
        """处理启动/停止请求"""
        from std_srvs.srv import SetBool
        
        if request.data:
            # 启动
            if not self._is_model_loaded:
                response.success = False
                response.message = "Model not loaded. Call /act_inference/load_model first."
                return response
            
            # 重置数据时间戳，确保新启动时需要新鲜数据
            self._head_image_timestamp = 0.0
            self._left_wrist_image_timestamp = 0.0
            self._right_wrist_image_timestamp = 0.0
            self._left_arm_timestamp = 0.0
            self._right_arm_timestamp = 0.0
            
            self._is_running = True
            self.policy.reset()
            self.executor.reset()
            response.success = True
            response.message = f"ACT inference started (model: {self._current_model_name})"
            self.get_logger().info(f"Inference STARTED with model: {self._current_model_name}")
        else:
            # 停止
            self._is_running = False
            self.executor.reset()  # 清空动作队列，防止残留动作
            response.success = True
            response.message = "ACT inference stopped"
            self.get_logger().info("Inference STOPPED")
        
        return response
    
    def _handle_load_model(self, request, response):
        """处理模型加载请求"""
        from std_srvs.srv import Trigger
        
        if self._is_running:
            response.success = False
            response.message = "Cannot load model while running. Stop first."
            return response
        
        self.get_logger().info(f"Loading model from: {self.config.model_path}")
        
        success = self.policy.load()
        
        if success:
            self._is_model_loaded = True
            self._current_model_name = os.path.basename(os.path.dirname(self.config.model_path))
            response.success = True
            response.message = f"Model loaded: {self._current_model_name}"
            
            # 发布当前模型名称
            model_msg = String()
            model_msg.data = self._current_model_name
            self._model_name_pub.publish(model_msg)
            
            self.get_logger().info(f"Model loaded successfully: {self._current_model_name}")
        else:
            response.success = False
            response.message = f"Failed to load model from: {self.config.model_path}"
        
        return response
    
    def _handle_reset(self, request, response):
        """处理重置请求"""
        from std_srvs.srv import Trigger
        
        self._is_running = False
        self.policy.reset()
        self.executor.reset()
        
        response.success = True
        response.message = "Reset complete"
        self.get_logger().info("Reset complete")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ACTInferenceNode()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor(num_threads=4)
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
