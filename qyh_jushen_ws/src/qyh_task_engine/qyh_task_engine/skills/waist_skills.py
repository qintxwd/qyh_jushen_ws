"""
腰部电机技能节点 (Waist Skills)

控制机器人腰部前倾角度
位置范围: 163711(前倾45°) ~ 230715(竖直0°)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


# 位置常量
POSITION_UPRIGHT = 230715    # 竖直位置
POSITION_MAX_LEAN = 163711   # 最大前倾位置 (45度)
MAX_ANGLE = 45.0


def angle_to_position(angle: float) -> int:
    """角度转位置"""
    angle = max(0.0, min(angle, MAX_ANGLE))
    ratio = angle / MAX_ANGLE
    return int(POSITION_UPRIGHT + ratio * (POSITION_MAX_LEAN - POSITION_UPRIGHT))


def position_to_angle(position: int) -> float:
    """位置转角度"""
    if POSITION_MAX_LEAN == POSITION_UPRIGHT:
        return 0.0
    ratio = (position - POSITION_UPRIGHT) / (POSITION_MAX_LEAN - POSITION_UPRIGHT)
    ratio = max(0.0, min(ratio, 1.0))
    return ratio * MAX_ANGLE


class WaistMoveToNode(SkillNode):
    """
    腰部电机移动节点
    
    参数:
        angle: 目标角度 (度, 0~45)
        angle_name: 预设角度名称（可选，与 angle 二选一）
        speed: 移动速度，默认 1000
        timeout: 超时时间 (秒)，默认 30
    """
    
    NODE_TYPE = "WaistMoveTo"
    
    PARAM_SCHEMA = {
        'angle': {'type': 'float', 'required': False},
        'angle_name': {'type': 'string', 'required': False},
        'speed': {'type': 'int', 'default': 1000},
        'timeout': {'type': 'float', 'default': 30.0},
    }
    
    # 角度到达容差 (度)
    ANGLE_TOLERANCE = 1.0
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._waist_client = None
        self._status_sub = None
        self._is_moving = False
        self._command_sent = False
        self._start_time: Optional[float] = None
        self._target_angle: Optional[float] = None
        # 腰部状态
        self._current_angle = 0.0
        self._position_reached = False
        self._status_received = False
    
    def setup(self) -> bool:
        """初始化腰部控制客户端"""
        self.log_info("="*40)
        self.log_info(f"[WaistMoveTo] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        if not self.ros_node:
            self.log_warn("  No ROS node, running in MOCK mode")
            return True
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            from qyh_waist_msgs.msg import WaistState
            
            self._waist_client = self.ros_node.create_client(
                WaistControl, '/waist/control'
            )
            self.log_info("  Waist client created")
            
            # 订阅腰部状态
            self._status_sub = self.ros_node.create_subscription(
                WaistState,
                '/waist/state',
                self._status_callback,
                10
            )
            self.log_info("  Subscribed to /waist/state")
            self.log_info("="*40)
            return True
        except Exception as e:
            self.log_warn(f"  Failed to create waist client: {e}")
            return True
    
    def _status_callback(self, msg):
        """腰部状态回调"""
        self._current_angle = msg.current_angle
        self._position_reached = msg.position_reached
        # 只有在命令发送后才标记状态已接收
        if self._command_sent:
            self._status_received = True
    
    def execute(self) -> SkillResult:
        """执行腰部移动"""
        timeout = self.params.get('timeout', 30.0)
        
        # 解析目标角度
        target_angle = self._resolve_angle()
        if target_angle is None:
            self.log_error("Cannot resolve target angle")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid angle: missing angle or angle_name"
            )
        
        # 限制角度范围
        target_angle = max(0.0, min(target_angle, MAX_ANGLE))
        
        self.log_info(f"[Waist] Moving to {target_angle:.1f} deg")
        
        # Mock 模式
        if not self._waist_client:
            if self._start_time is None:
                self._start_time = time.time()
            
            elapsed = time.time() - self._start_time
            if elapsed > 1.0:
                self.log_info(f"[Waist] At {target_angle:.1f} deg (mock)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Waist at {target_angle:.1f} deg (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Moving... {elapsed:.1f}s"
            )
        
        # 真实执行
        if not self._command_sent:
            if not self._waist_client.wait_for_service(timeout_sec=2.0):
                self.log_error("Waist service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Waist service not available"
                )
            
            # 先设置速度
            speed = self.params.get('speed', 1000)
            from qyh_waist_msgs.srv import WaistControl
            
            speed_request = WaistControl.Request()
            speed_request.command = 3  # SET_SPEED
            speed_request.value = float(speed)
            speed_request.hold = False
            self._waist_client.call_async(speed_request)
            
            # 发送角度命令
            request = WaistControl.Request()
            request.command = 5  # GO_ANGLE
            request.value = float(target_angle)
            request.hold = False
            
            self.log_info(f"   Sending waist cmd: {target_angle:.1f} deg, speed={speed}")
            # Fire-and-forget，不等待响应
            self._waist_client.call_async(request)
            
            self._command_sent = True
            self._is_moving = True
            self._start_time = time.time()
            self._target_angle = target_angle
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Waist moving to {target_angle:.1f}°..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            self.log_error(f"Waist timeout ({timeout}s), current: {self._current_angle:.1f}°")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Waist timeout ({timeout}s)"
            )
        
        # 发送命令后等待至少 0.5 秒再检查状态
        MIN_WAIT_TIME = 0.5
        if elapsed < MIN_WAIT_TIME:
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Waist command sent, waiting... {elapsed:.1f}s"
            )
        
        # 检查是否到位（只通过角度差判断，position_reached 标志不可靠）
        angle_diff = abs(self._current_angle - target_angle)
        
        if angle_diff < self.ANGLE_TOLERANCE:
            self.log_info(f"[Waist] ✓ Reached {target_angle:.1f}° (current: {self._current_angle:.1f}°, diff: {angle_diff:.1f}°, {elapsed:.1f}s)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Waist at {target_angle:.1f}°"
            )
        
        # 每 2 秒打印进度
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            if not hasattr(self, '_last_progress_log'):
                self._last_progress_log = -1
            if self._last_progress_log != int(elapsed):
                self._last_progress_log = int(elapsed)
                self.log_info(f"   Waist moving... {self._current_angle:.1f}° -> {target_angle:.1f}° ({elapsed:.0f}s)")
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Moving to {target_angle:.1f}°... (current: {self._current_angle:.1f}°)"
        )
    
    def _resolve_angle(self) -> Optional[float]:
        """解析目标角度"""
        # 直接指定角度
        if 'angle' in self.params:
            return float(self.params['angle'])
        
        # 使用预设角度名称
        if 'angle_name' in self.params:
            angle_name = self.params['angle_name']
            
            # 从持久化预设加载
            angle_preset = preset_loader.get_waist_angle(angle_name)
            if angle_preset:
                return float(angle_preset.get('angle', 0.0))
            
            # 内置预设
            builtin_angles = {
                'upright': 0.0,
                '竖直': 0.0,
                'slight_lean': 15.0,
                '轻微前倾': 15.0,
                'medium_lean': 30.0,
                '中等前倾': 30.0,
                'max_lean': 45.0,
                '最大前倾': 45.0,
            }
            
            if angle_name in builtin_angles:
                return builtin_angles[angle_name]
            
            self.log_error(f"Unknown waist angle: {angle_name}")
            return None
        
        return None
    
    def halt(self):
        """中断腰部移动"""
        super().halt()
        # TODO: 发送停止命令
        self.log_info("Waist halted")
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._is_moving = False
        self._command_sent = False
        self._start_time = None
        self._target_angle = None
        self._position_reached = False
        self._status_received = False
        if hasattr(self, '_last_progress_log'):
            delattr(self, '_last_progress_log')


class WaistStopNode(SkillNode):
    """
    腰部电机停止节点
    """
    
    NODE_TYPE = "WaistStop"
    
    PARAM_SCHEMA = {}
    
    def setup(self) -> bool:
        return True
    
    def execute(self) -> SkillResult:
        self.log_info("Stopping waist")
        
        if not self.ros_node:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Waist stopped (mock mode)"
            )
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            client = self.ros_node.create_client(
                WaistControl, '/waist/control'
            )
            
            if not client.wait_for_service(timeout_sec=1.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Waist service not available"
                )
            
            request = WaistControl.Request()
            request.command = 9  # STOP
            request.value = 0.0
            request.hold = False
            
            # 不等待响应，立即返回
            client.call_async(request)
            
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Waist stop command sent"
            )
        except Exception as e:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Failed to stop waist: {e}"
            )


class WaistUprightNode(SkillNode):
    """
    腰部回正节点 - 回到竖直位置
    """
    
    NODE_TYPE = "WaistUpright"
    
    PARAM_SCHEMA = {
        'speed': {'type': 'int', 'default': 1000},
        'timeout': {'type': 'float', 'default': 30.0},
    }
    
    # 角度到达容差 (度) - 竖直位置是 0 度
    ANGLE_TOLERANCE = 1.0
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._waist_client = None
        self._status_sub = None
        self._is_moving = False
        self._command_sent = False
        self._start_time: Optional[float] = None
        # 腰部状态
        self._current_angle = 0.0
        self._position_reached = False
        self._status_received = False
    
    def setup(self) -> bool:
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            from qyh_waist_msgs.msg import WaistState
            
            self._waist_client = self.ros_node.create_client(
                WaistControl, '/waist/control'
            )
            
            # 订阅腰部状态
            self._status_sub = self.ros_node.create_subscription(
                WaistState,
                '/waist/state',
                self._status_callback,
                10
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to create waist client: {e}")
            return True
    
    def _status_callback(self, msg):
        """腰部状态回调"""
        self._current_angle = msg.current_angle
        self._position_reached = msg.position_reached
        # 只有在命令发送后才标记状态已接收
        if self._command_sent:
            self._status_received = True
    
    def execute(self) -> SkillResult:
        timeout = self.params.get('timeout', 30.0)
        
        # Mock 模式
        if not self._waist_client:
            if self._start_time is None:
                self._start_time = time.time()
                self.log_info("Moving waist to upright position (mock)")
            
            elapsed = time.time() - self._start_time
            if elapsed > 1.0:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message="Waist upright (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Moving... {elapsed:.1f}s"
            )
        
        # 真实执行 - 发送命令
        if not self._command_sent:
            if not self._waist_client.wait_for_service(timeout_sec=2.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Waist service not available"
                )
            
            self.log_info("[Waist] Moving to upright position (0°)")
            
            from qyh_waist_msgs.srv import WaistControl
            request = WaistControl.Request()
            request.command = 10  # GO_UPRIGHT
            request.value = 0.0
            request.hold = False
            
            # Fire-and-forget
            self._waist_client.call_async(request)
            self._command_sent = True
            self._is_moving = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message="Waist moving to upright..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            self.log_error(f"Waist timeout ({timeout}s), current: {self._current_angle:.1f}°")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Waist timeout ({timeout}s)"
            )
        
        # 发送命令后等待至少 0.5 秒再检查状态
        MIN_WAIT_TIME = 0.5
        if elapsed < MIN_WAIT_TIME:
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Waist command sent, waiting... {elapsed:.1f}s"
            )
        
        # 检查是否到位（竖直位置是 0 度，只通过角度差判断）
        if abs(self._current_angle) < self.ANGLE_TOLERANCE:
            self.log_info(f"[Waist] ✓ Upright reached (current: {self._current_angle:.1f}°, {elapsed:.1f}s)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Waist upright"
            )
        
        # 每 2 秒打印进度
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            if not hasattr(self, '_last_progress_log'):
                self._last_progress_log = -1
            if self._last_progress_log != int(elapsed):
                self._last_progress_log = int(elapsed)
                self.log_info(f"   Waist moving to upright... {self._current_angle:.1f}° -> 0° ({elapsed:.0f}s)")
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Moving to upright... (current: {self._current_angle:.1f}°)"
        )
    
    def halt(self):
        super().halt()
        self.log_info("Waist upright halted")
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._is_moving = False
        self._command_sent = False
        self._start_time = None
        self._position_reached = False
        self._status_received = False
        if hasattr(self, '_last_progress_log'):
            delattr(self, '_last_progress_log')
