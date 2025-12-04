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
        location: 预设点位名称（如 station_1）
        x: 目标 X 坐标 (米) - 仅当不使用预设时
        y: 目标 Y 坐标 (米) - 仅当不使用预设时  
        theta: 目标朝向 (弧度)
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
        self._nav_site_client = None  # 站点导航服务客户端
        self._is_navigating = False
        self._start_time: Optional[float] = None
        self._station_id: Optional[int] = None
    
    def setup(self) -> bool:
        """初始化导航服务客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            # 使用底盘的站点导航服务
            from qyh_standard_robot_msgs.srv import GoNavigateToSite
            self._nav_site_client = self.ros_node.create_client(
                GoNavigateToSite,
                'go_navigate_to_site_simple'
            )
            self.log_info("Created chassis navigation client")
            return True
        except Exception as e:
            self.log_warn(f"Failed to create navigation client: {e}")
            return True
    
    def execute(self) -> SkillResult:
        """执行导航"""
        timeout = self.params.get('timeout', 60.0)
        
        # 解析站点ID
        station_id = self._resolve_station_id()
        if station_id is None:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid target: cannot resolve station ID"
            )
        
        self._station_id = station_id
        
        # Mock 模式
        if not self._nav_site_client:
            if self._start_time is None:
                self._start_time = time.time()
                self.log_info(f"Navigating to station {station_id} (mock mode)")
            
            elapsed = time.time() - self._start_time
            if elapsed > 2.0:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Arrived at station {station_id} (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Navigating to station {station_id}... {elapsed:.1f}s"
            )
        
        # 真实执行
        if not self._is_navigating:
            if not self._nav_site_client.wait_for_service(timeout_sec=5.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Navigation service not available"
                )
            
            from qyh_standard_robot_msgs.srv import GoNavigateToSite
            
            request = GoNavigateToSite.Request()
            request.site_id = station_id
            
            self.log_info(f"Sending navigation request to station {station_id}")
            self._future = self._nav_site_client.call_async(request)
            self._is_navigating = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Navigation to station {station_id} started..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Navigation timeout ({timeout}s)"
            )
        
        # 检查服务调用是否完成
        if hasattr(self, '_future') and self._future.done():
            try:
                result = self._future.result()
                if result.success:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=f"Arrived at station {station_id}"
                    )
                else:
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=f"Navigation failed: {result.message}"
                    )
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Navigation error: {e}"
                )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Navigating to station {station_id}... {elapsed:.1f}s"
        )
    
    def _resolve_station_id(self) -> Optional[int]:
        """解析站点ID"""
        # 优先使用预设点位
        if 'location' in self.params:
            location_name = self.params['location']
            
            # 从预设加载
            loc_preset = preset_loader.get_location(location_name)
            if loc_preset:
                # 获取原始站点ID
                station_id = loc_preset.get('station_id')
                if station_id is not None:
                    return int(station_id)
                
                # 尝试从 id 字段解析 (格式: station_X)
                preset_id = loc_preset.get('id', '')
                if preset_id.startswith('station_'):
                    try:
                        return int(preset_id.replace('station_', ''))
                    except ValueError:
                        pass
            
            # 尝试直接从 location 参数解析 (格式: station_X)
            if location_name.startswith('station_'):
                try:
                    return int(location_name.replace('station_', ''))
                except ValueError:
                    pass
            
            self.log_error(f"Cannot resolve station ID from location: {location_name}")
            return None
        
        # 如果没有 location 参数，不支持坐标导航（底盘只支持站点）
        self.log_error("No location parameter provided")
        return None
    
    def halt(self):
        """中断导航"""
        super().halt()
        self._is_navigating = False
        self.log_info("Navigation halted")


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
