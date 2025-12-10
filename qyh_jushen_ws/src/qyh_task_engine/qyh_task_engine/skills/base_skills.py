"""
底盘技能节点 (Base/Chassis Skills)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


# 导航相关的系统状态
NAV_STATUS_IDLE = 0x02
NAV_STATUS_PATHFINDING = 0x06
NAV_STATUS_WAITING_ARRIVAL = 0x07
NAV_STATUS_OBSTACLE_DETECTED = 0x08
NAV_STATUS_REPATHING = 0x09
NAV_STATUS_OBSTACLE_PAUSED = 0x0A
NAV_STATUS_CANNOT_ARRIVE = 0x0B
NAV_STATUS_NAV_ERROR = 0x15

# 正在导航中的状态集合
NAVIGATING_STATUSES = {
    NAV_STATUS_PATHFINDING,
    NAV_STATUS_WAITING_ARRIVAL,
    NAV_STATUS_OBSTACLE_DETECTED,
    NAV_STATUS_REPATHING,
    NAV_STATUS_OBSTACLE_PAUSED,
}

# 导航失败的状态
NAV_FAILURE_STATUSES = {
    NAV_STATUS_CANNOT_ARRIVE,
    NAV_STATUS_NAV_ERROR,
}


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
        self._status_sub = None       # 底盘状态订阅器
        self._is_navigating = False
        self._command_sent = False    # 命令是否已发送
        self._start_time: Optional[float] = None
        self._station_id: Optional[int] = None
        self._current_sys_status = NAV_STATUS_IDLE
        self._last_sys_status = NAV_STATUS_IDLE
    
    def setup(self) -> bool:
        """初始化导航服务客户端"""
        self.log_info("="*40)
        self.log_info("[BaseMoveTo] Setting up navigation node")
        self.log_info(f"  Node ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        if not self.ros_node:
            self.log_warn("  No ROS node available, running in MOCK mode")
            return True
        
        try:
            # 使用底盘的站点导航服务
            from qyh_standard_robot_msgs.srv import GoNavigateToSite
            from qyh_standard_robot_msgs.msg import StandardRobotStatus
            
            self._nav_site_client = self.ros_node.create_client(
                GoNavigateToSite,
                'go_navigate_to_site_simple'
            )
            self.log_info("  Created chassis navigation client")
            self.log_info("  Service: go_navigate_to_site_simple")
            
            # 订阅底盘状态以监控导航进度
            self._status_sub = self.ros_node.create_subscription(
                StandardRobotStatus,
                'standard_robot_status',
                self._status_callback,
                10
            )
            self.log_info("  Subscribed to: standard_robot_status")
            self.log_info("="*40)
            return True
        except Exception as e:
            self.log_warn(f"  Failed to create client: {e}")
            self.log_info("="*40)
            return True
    
    def _status_callback(self, msg):
        """底盘状态回调"""
        old_status = self._current_sys_status
        self._last_sys_status = old_status
        self._current_sys_status = msg.system_status
        if old_status != msg.system_status:
            self.log_info(f"[DEBUG] Status callback: 0x{old_status:02X} -> 0x{msg.system_status:02X}")
    
    def execute(self) -> SkillResult:
        """执行导航"""
        timeout = self.params.get('timeout', 60.0)
        
        # 解析站点ID
        station_id = self._resolve_station_id()
        if station_id is None:
            self.log_error(f"Cannot resolve station ID: {self.params}")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid target: cannot resolve station ID"
            )
        
        self._station_id = station_id
        
        # Mock 模式
        if not self._nav_site_client:
            if self._start_time is None:
                self._start_time = time.time()
                self.log_info(f"[MOCK] Nav to station {station_id}")
            
            elapsed = time.time() - self._start_time
            if elapsed > 2.0:
                self.log_info(f"[MOCK] Arrived station {station_id} ({elapsed:.1f}s)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Arrived at station {station_id} (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Navigating to station {station_id}... {elapsed:.1f}s"
            )
        
        # 真实执行 - 阶段1: 发送导航命令
        if not self._is_navigating:
            self.log_info(f"[NAV] Start nav to station {station_id}")
            self.log_info("   Waiting for service...")
            
            if not self._nav_site_client.wait_for_service(timeout_sec=5.0):
                self.log_error("Navigation service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Navigation service not available"
                )
            
            self.log_info("   Service available")
            
            from qyh_standard_robot_msgs.srv import GoNavigateToSite
            
            request = GoNavigateToSite.Request()
            request.site_id = station_id
            
            self.log_info(f"   Sending request: site_id={station_id}")
            # 使用 call_async 但不等待结果，因为这是 fire-and-forget 类型的服务
            # 服务只发送命令，真正的导航状态需要通过 standard_robot_status 话题监控
            future = self._nav_site_client.call_async(request)
            # 不保存 future，不等待响应
            
            self._is_navigating = True
            self._command_sent = True  # 直接标记命令已发送
            self._start_time = time.time()
            
            self.log_info(f"   Command sent, monitoring navigation status...")
            self.log_info(f"   Current system status: 0x{self._current_sys_status:02X}")
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Navigation to station {station_id} started, monitoring..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            self.log_error(f"Navigation timeout after {timeout}s")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Navigation timeout ({timeout}s)"
            )
        
        # 监控导航状态直到完成
        sys_status = self._current_sys_status
        self.log_info(f"[DEBUG] execute() monitoring, sys_status=0x{sys_status:02X}, elapsed={elapsed:.1f}s, command_sent={self._command_sent}")
        
        # 检查导航失败状态
        if sys_status in NAV_FAILURE_STATUSES:
            status_name = self._get_status_name(sys_status)
            self.log_error(f"Navigation failed with status: {status_name} (0x{sys_status:02X})")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Navigation failed: {status_name}"
            )
        
        # 检查是否完成（从导航状态变为IDLE）
        self.log_info(f"[DEBUG] Checking completion: sys_status==IDLE? {sys_status == NAV_STATUS_IDLE}, command_sent={self._command_sent}, elapsed={elapsed:.1f}")
        if sys_status == NAV_STATUS_IDLE and self._command_sent:
            # 确认是从导航状态变过来的（避免初始状态就是IDLE的误判）
            if elapsed > 1.0:  # 至少经过1秒
                self.log_info(f"✓ Arrived at station {station_id} ({elapsed:.1f}s)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Arrived at station {station_id}"
                )
            else:
                self.log_info(f"[DEBUG] elapsed < 1.0s, waiting more...")
        
        # 仍在导航中
        status_name = self._get_status_name(sys_status)
        self.log_info(f"[DEBUG] Still navigating, returning RUNNING")
        
        # 每5秒打印一次进度
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            if not hasattr(self, '_last_progress_log'):
                self._last_progress_log = -1
            if self._last_progress_log != int(elapsed):
                self._last_progress_log = int(elapsed)
                self.log_info(f"   Navigating to {station_id}... {elapsed:.0f}s (status: {status_name})")
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Navigating to station {station_id}... {elapsed:.1f}s ({status_name})"
        )
    
    def _get_status_name(self, status: int) -> str:
        """获取状态名称"""
        status_names = {
            0x01: "Initializing",
            0x02: "Idle",
            0x03: "Error",
            0x04: "Locating",
            0x05: "NavInit",
            0x06: "Pathfinding",
            0x07: "WaitingArrival",
            0x08: "ObstacleDetected",
            0x09: "Repathing",
            0x0A: "ObstaclePaused",
            0x0B: "CannotArrive",
            0x15: "NavError",
            0x16: "HardwareError",
        }
        return status_names.get(status, f"Unknown(0x{status:02X})")
    
    def _resolve_station_id(self) -> Optional[int]:
        """解析站点ID"""
        self.log_info("   Resolving station ID from params...")
        
        # 优先使用预设点位
        if 'location' in self.params:
            location_name = self.params['location']
            self.log_info(f"   Location param: '{location_name}'")
            
            # 从预设加载
            loc_preset = preset_loader.get_location(location_name)
            if loc_preset:
                self.log_info(f"   Found preset: {loc_preset}")
                # 获取原始站点ID
                station_id = loc_preset.get('station_id')
                if station_id is not None:
                    self.log_info(f"   >> Resolved station_id: {station_id}")
                    return int(station_id)
                
                # 尝试从 id 字段解析 (格式: station_X)
                preset_id = loc_preset.get('id', '')
                if preset_id.startswith('station_'):
                    try:
                        station_id = int(preset_id.replace('station_', ''))
                        self.log_info(f"   >> Resolved from preset id: {station_id}")
                        return station_id
                    except ValueError:
                        pass
            else:
                self.log_warn(f"   No preset found: {location_name}")
            
            # 尝试直接从 location 参数解析 (格式: station_X)
            if location_name.startswith('station_'):
                try:
                    station_id = int(location_name.replace('station_', ''))
                    self.log_info(f"   >> Resolved from location name: {station_id}")
                    return station_id
                except ValueError:
                    pass
            
            self.log_error(f"   Cannot resolve station: {location_name}")
            return None
        
        # 如果没有 location 参数，不支持坐标导航（底盘只支持站点）
        self.log_error(f"   No location param in: {self.params}")
        return None
    
    def halt(self):
        """中断导航"""
        super().halt()
        self._is_navigating = False
        self._command_sent = False
        self.log_info(f"Navigation halted (station {self._station_id})")
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._is_navigating = False
        self._command_sent = False
        self._start_time = None
        self._station_id = None
        self._current_sys_status = NAV_STATUS_IDLE
        self._last_sys_status = NAV_STATUS_IDLE
        if hasattr(self, '_last_progress_log'):
            delattr(self, '_last_progress_log')


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
