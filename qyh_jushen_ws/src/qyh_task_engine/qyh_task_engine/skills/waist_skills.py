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
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._waist_client = None
        self._future = None
        self._is_moving = False
        self._start_time: Optional[float] = None
    
    def setup(self) -> bool:
        """初始化腰部控制客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            self._waist_client = self.ros_node.create_client(
                WaistControl, '/waist/control'
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to create waist client: {e}")
            return True
    
    def execute(self) -> SkillResult:
        """执行腰部移动"""
        timeout = self.params.get('timeout', 30.0)
        
        # 解析目标角度
        target_angle = self._resolve_angle()
        if target_angle is None:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid angle: missing angle or angle_name"
            )
        
        # 限制角度范围
        target_angle = max(0.0, min(target_angle, MAX_ANGLE))
        
        self.log_info(f"Moving waist to {target_angle:.1f}°")
        
        # Mock 模式
        if not self._waist_client:
            if self._start_time is None:
                self._start_time = time.time()
            
            elapsed = time.time() - self._start_time
            if elapsed > 1.0:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Waist at {target_angle:.1f}° (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Moving... {elapsed:.1f}s"
            )
        
        # 真实执行
        if not self._is_moving:
            if not self._waist_client.wait_for_service(timeout_sec=2.0):
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
            
            self._future = self._waist_client.call_async(request)
            self._is_moving = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message="Waist moving..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Waist timeout ({timeout}s)"
            )
        
        # 检查是否完成
        if self._future and self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=f"Waist at {target_angle:.1f}°"
                    )
                else:
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=response.message
                    )
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Waist failed: {e}"
                )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Waist moving... {elapsed:.1f}s"
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
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._waist_client = None
        self._future = None
        self._is_moving = False
        self._start_time: Optional[float] = None
    
    def setup(self) -> bool:
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            self._waist_client = self.ros_node.create_client(
                WaistControl, '/waist/control'
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to create waist client: {e}")
            return True
    
    def execute(self) -> SkillResult:
        timeout = self.params.get('timeout', 30.0)
        
        self.log_info("Moving waist to upright position")
        
        # Mock 模式
        if not self._waist_client:
            if self._start_time is None:
                self._start_time = time.time()
            
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
        
        # 真实执行
        if not self._is_moving:
            if not self._waist_client.wait_for_service(timeout_sec=2.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Waist service not available"
                )
            
            from qyh_waist_msgs.srv import WaistControl
            request = WaistControl.Request()
            request.command = 10  # GO_UPRIGHT
            request.value = 0.0
            request.hold = False
            
            self._future = self._waist_client.call_async(request)
            self._is_moving = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message="Waist moving to upright..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Waist timeout ({timeout}s)"
            )
        
        # 检查是否完成
        if self._future and self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message="Waist upright"
                    )
                else:
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=response.message
                    )
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Waist failed: {e}"
                )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Waist moving... {elapsed:.1f}s"
        )
    
    def halt(self):
        super().halt()
        self.log_info("Waist upright halted")
