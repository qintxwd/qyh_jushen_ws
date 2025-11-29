"""
底盘技能节点 (Base/Chassis Skills)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


class BaseMoveToNode(SkillNode):
    """
    底盘导航节点
    
    参数:
        x: 目标 X 坐标 (米)
        y: 目标 Y 坐标 (米)
        theta: 目标朝向 (弧度)
        location: 预设点位名称（可选，与 x/y/theta 二选一）
        timeout: 超时时间 (秒)，默认 60
    """
    
    NODE_TYPE = "BaseMoveTo"
    
    PARAM_SCHEMA = {
        'x': {'type': 'float', 'required': False},
        'y': {'type': 'float', 'required': False},
        'theta': {'type': 'float', 'default': 0.0},
        'location': {'type': 'string', 'required': False},
        'timeout': {'type': 'float', 'default': 60.0},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._nav_client = None
        self._goal_handle = None
        self._is_navigating = False
        self._start_time: Optional[float] = None
    
    def setup(self) -> bool:
        """初始化导航动作客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            
            self._nav_client = ActionClient(
                self.ros_node,
                NavigateToPose,
                '/navigate_to_pose'
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to create navigation client: {e}")
            # 导航可能不可用，使用 mock 模式
            return True
    
    def execute(self) -> SkillResult:
        """执行导航"""
        timeout = self.params.get('timeout', 60.0)
        
        # 解析目标位置
        target = self._resolve_target()
        if target is None:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid target: missing x/y or location"
            )
        
        x, y, theta = target
        self.log_info(f"Navigating to ({x:.2f}, {y:.2f}, θ={theta:.2f})")
        
        # Mock 模式
        if not self._nav_client:
            if self._start_time is None:
                self._start_time = time.time()
            
            # 模拟导航时间
            elapsed = time.time() - self._start_time
            if elapsed > 2.0:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Arrived at ({x:.2f}, {y:.2f}) (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Navigating... {elapsed:.1f}s"
            )
        
        # 真实执行
        if not self._is_navigating:
            if not self._nav_client.wait_for_server(timeout_sec=5.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Navigation server not available"
                )
            
            from nav2_msgs.action import NavigateToPose
            from geometry_msgs.msg import PoseStamped
            import math
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.z = math.sin(theta / 2)
            goal_msg.pose.pose.orientation.w = math.cos(theta / 2)
            
            self._goal_handle = self._nav_client.send_goal_async(goal_msg)
            self._is_navigating = True
            self._start_time = time.time()
            
            return SkillResult(status=SkillStatus.RUNNING, message="Navigation started...")
        
        # 检查超时
        if time.time() - self._start_time > timeout:
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Navigation timeout ({timeout}s)"
            )
        
        # 检查是否完成
        if self._goal_handle and self._goal_handle.done():
            goal_handle = self._goal_handle.result()
            if goal_handle.accepted:
                result = goal_handle.get_result_async()
                if result.done():
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message="Navigation completed"
                    )
            else:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Navigation goal rejected"
                )
        
        elapsed = time.time() - self._start_time
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Navigating... {elapsed:.1f}s"
        )
    
    def _resolve_target(self) -> Optional[tuple]:
        """解析目标位置"""
        # 优先使用预设点位
        if 'location' in self.params:
            location_name = self.params['location']
            
            # 优先从持久化预设加载
            loc_preset = preset_loader.get_location(location_name)
            if loc_preset:
                return (
                    loc_preset.get('x', 0),
                    loc_preset.get('y', 0),
                    loc_preset.get('theta', 0)
                )
            
            # 从黑板读取（兼容旧格式）
            locations = self.read_from_blackboard('assets.locations', {})
            if location_name in locations:
                loc = locations[location_name]
                return (loc.get('x', 0), loc.get('y', 0), loc.get('theta', 0))
            
            self.log_error(f"Unknown location: {location_name}")
            return None
        
        # 使用坐标
        if 'x' in self.params and 'y' in self.params:
            return (
                self.params['x'],
                self.params['y'],
                self.params.get('theta', 0.0)
            )
        
        return None
    
    def halt(self):
        """中断导航"""
        super().halt()
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self.log_info("Navigation cancelled")


class BaseVelocityNode(SkillNode):
    """
    底盘速度控制节点
    
    参数:
        linear_x: 前进速度 (m/s)
        angular_z: 旋转速度 (rad/s)
        duration: 持续时间 (秒)
    """
    
    NODE_TYPE = "BaseVelocity"
    
    PARAM_SCHEMA = {
        'linear_x': {'type': 'float', 'default': 0.0},
        'angular_z': {'type': 'float', 'default': 0.0},
        'duration': {'type': 'float', 'required': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._cmd_vel_pub = None
        self._start_time: Optional[float] = None
    
    def setup(self) -> bool:
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from geometry_msgs.msg import Twist
            self._cmd_vel_pub = self.ros_node.create_publisher(
                Twist, '/cmd_vel', 10
            )
            return True
        except Exception as e:
            self.log_error(f"Failed to create cmd_vel publisher: {e}")
            return False
    
    def execute(self) -> SkillResult:
        linear_x = self.params.get('linear_x', 0.0)
        angular_z = self.params.get('angular_z', 0.0)
        duration = self.params.get('duration', 1.0)
        
        if self._start_time is None:
            self._start_time = time.time()
            self.log_info(f"Moving: linear={linear_x:.2f}m/s, angular={angular_z:.2f}rad/s for {duration}s")
        
        elapsed = time.time() - self._start_time
        
        if elapsed >= duration:
            # 停止
            self._publish_velocity(0.0, 0.0)
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Velocity command completed after {duration}s"
            )
        
        # 发送速度命令
        self._publish_velocity(linear_x, angular_z)
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Moving... {elapsed:.1f}/{duration}s"
        )
    
    def _publish_velocity(self, linear_x: float, angular_z: float):
        if not self._cmd_vel_pub:
            return
        
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)
    
    def halt(self):
        super().halt()
        self._publish_velocity(0.0, 0.0)
        self.log_info("Base stopped")
