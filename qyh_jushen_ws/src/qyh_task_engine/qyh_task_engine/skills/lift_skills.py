"""
升降电机技能节点 (Lift Skills)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


class LiftMoveToNode(SkillNode):
    """
    升降电机移动节点
    
    参数:
        height: 目标高度 (毫米)
        height_name: 预设高度名称（可选，与 height 二选一）
        speed: 移动速度，默认 50.0
        timeout: 超时时间 (秒)，默认 30
    """
    
    NODE_TYPE = "LiftMoveTo"
    
    PARAM_SCHEMA = {
        'height': {'type': 'float', 'required': False},
        'height_name': {'type': 'string', 'required': False},
        'speed': {'type': 'float', 'default': 50.0},
        'timeout': {'type': 'float', 'default': 30.0},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._lift_client = None
        self._future = None
        self._is_moving = False
        self._start_time: Optional[float] = None
    
    def setup(self) -> bool:
        """初始化升降控制客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_lift_msgs.srv import LiftControl
            self._lift_client = self.ros_node.create_client(
                LiftControl, '/lift/control'
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to create lift client: {e}")
            return True
    
    def execute(self) -> SkillResult:
        """执行升降移动"""
        timeout = self.params.get('timeout', 30.0)
        
        # 解析目标高度
        target_height = self._resolve_height()
        if target_height is None:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid height: missing height or height_name"
            )
        
        self.log_info(f"Moving lift to {target_height}mm")
        
        # Mock 模式
        if not self._lift_client:
            if self._start_time is None:
                self._start_time = time.time()
            
            elapsed = time.time() - self._start_time
            if elapsed > 1.0:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Lift at {target_height}mm (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Moving... {elapsed:.1f}s"
            )
        
        # 真实执行
        if not self._is_moving:
            if not self._lift_client.wait_for_service(timeout_sec=2.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Lift service not available"
                )
            
            from qyh_lift_msgs.srv import LiftControl
            request = LiftControl.Request()
            request.command = 2  # 位置模式
            request.value = float(target_height)
            request.hold = True
            
            self._future = self._lift_client.call_async(request)
            self._is_moving = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message="Lift moving..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Lift timeout ({timeout}s)"
            )
        
        # 检查是否完成
        if self._future and self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=f"Lift at {target_height}mm"
                    )
                else:
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=response.message
                    )
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Lift failed: {e}"
                )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Lift moving... {elapsed:.1f}s"
        )
    
    def _resolve_height(self) -> Optional[float]:
        """解析目标高度"""
        # 直接指定高度
        if 'height' in self.params:
            return float(self.params['height'])
        
        # 使用预设高度名称
        if 'height_name' in self.params:
            height_name = self.params['height_name']
            
            # 从持久化预设加载
            height_preset = preset_loader.get_lift_height(height_name)
            if height_preset:
                return float(height_preset.get('height', 0.0))
            
            self.log_error(f"Unknown lift height: {height_name}")
            return None
        
        return None
    
    def halt(self):
        """中断升降"""
        super().halt()
        # TODO: 发送停止命令
        self.log_info("Lift halted")


class LiftStopNode(SkillNode):
    """
    升降电机停止节点
    """
    
    NODE_TYPE = "LiftStop"
    
    PARAM_SCHEMA = {}
    
    def setup(self) -> bool:
        return True
    
    def execute(self) -> SkillResult:
        self.log_info("Stopping lift")
        
        if not self.ros_node:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Lift stopped (mock mode)"
            )
        
        try:
            from qyh_lift_msgs.srv import LiftControl
            client = self.ros_node.create_client(
                LiftControl, '/lift/control'
            )
            
            if not client.wait_for_service(timeout_sec=1.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Lift service not available"
                )
            
            request = LiftControl.Request()
            request.command = 0  # 停止
            request.value = 0.0
            request.hold = False
            
            # 不等待响应，立即返回
            client.call_async(request)
            
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Lift stop command sent"
            )
        except Exception as e:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Failed to stop lift: {e}"
            )
