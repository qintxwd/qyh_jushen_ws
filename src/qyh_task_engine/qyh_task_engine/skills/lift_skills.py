"""
升降电机技能节点 (Lift Skills)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


# 位置到达的容差 (mm)
POSITION_TOLERANCE = 5.0


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
        self._status_sub = None
        self._is_moving = False
        self._command_sent = False
        self._start_time: Optional[float] = None
        self._target_height: Optional[float] = None
        # 升降机状态
        self._current_position = 0.0
        self._position_reached = False
        self._status_received = False  # 是否收到过状态更新
    
    def setup(self) -> bool:
        """初始化升降控制客户端"""
        self.log_info("="*40)
        self.log_info(f"[LiftMoveTo] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        if not self.ros_node:
            self.log_warn("  No ROS node, running in MOCK mode")
            return True
        
        try:
            from qyh_lift_msgs.srv import LiftControl
            from qyh_lift_msgs.msg import LiftState
            
            self._lift_client = self.ros_node.create_client(
                LiftControl, '/lift/control'
            )
            self.log_info("  Lift client created")
            
            # 订阅升降机状态
            self._status_sub = self.ros_node.create_subscription(
                LiftState,
                '/lift/state',
                self._status_callback,
                10
            )
            self.log_info("  Subscribed to /lift/state")
            self.log_info("="*40)
            return True
        except Exception as e:
            self.log_warn(f"  Failed to create lift client: {e}")
            return True
    
    def _status_callback(self, msg):
        """升降机状态回调"""
        self._current_position = msg.current_position
        self._position_reached = msg.position_reached
        # 只有在命令发送后才标记状态已接收
        if self._command_sent:
            self._status_received = True
    
    def execute(self) -> SkillResult:
        """执行升降移动"""
        timeout = self.params.get('timeout', 30.0)
        
        # 解析目标高度
        target_height = self._resolve_height()
        if target_height is None:
            self.log_error("Cannot resolve target height")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid height: missing height or height_name"
            )
        
        self._target_height = target_height
        
        # Mock 模式
        if not self._lift_client:
            if self._start_time is None:
                self._start_time = time.time()
                self.log_info(f"[Lift] Moving to {target_height}mm (mock)")
            
            elapsed = time.time() - self._start_time
            if elapsed > 2.0:  # Mock 模式等待 2 秒
                self.log_info(f"[Lift] At {target_height}mm (mock, {elapsed:.1f}s)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Lift at {target_height}mm (mock mode)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Moving... {elapsed:.1f}s"
            )
        
        # 真实执行 - 发送命令
        if not self._command_sent:
            self.log_info(f"[Lift] Moving to {target_height}mm")
            
            if not self._lift_client.wait_for_service(timeout_sec=2.0):
                self.log_error("Lift service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Lift service not available"
                )
            
            from qyh_lift_msgs.srv import LiftControl
            request = LiftControl.Request()
            request.command = 4  # CMD_GO_POSITION: 去目标位置
            request.value = float(target_height)
            request.hold = False
            
            self.log_info(f"   Sending lift command: {target_height}mm")
            # Fire-and-forget，不等待响应
            self._lift_client.call_async(request)
            
            self._command_sent = True
            self._is_moving = True
            self._start_time = time.time()
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Lift moving to {target_height}mm..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            self.log_error(f"Lift timeout ({timeout}s), current: {self._current_position}mm")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Lift timeout ({timeout}s)"
            )
        
        # 发送命令后等待至少 0.5 秒再检查状态
        # 这样即使本来就在目标位置，也有最小执行时间
        MIN_WAIT_TIME = 0.5
        if elapsed < MIN_WAIT_TIME:
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Lift command sent, waiting... {elapsed:.1f}s"
            )
        
        # 检查是否到位（只通过位置差判断，position_reached 标志不可靠）
        position_diff = abs(self._current_position - target_height)
        
        if position_diff < POSITION_TOLERANCE:
            self.log_info(f"[Lift] ✓ Reached {target_height}mm (current: {self._current_position:.1f}mm, diff: {position_diff:.1f}mm, {elapsed:.1f}s)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Lift at {target_height}mm"
            )
        
        # 每 2 秒打印进度
        if int(elapsed) % 2 == 0 and int(elapsed) > 0:
            if not hasattr(self, '_last_progress_log'):
                self._last_progress_log = -1
            if self._last_progress_log != int(elapsed):
                self._last_progress_log = int(elapsed)
                self.log_info(f"   Lift moving... {self._current_position:.1f}mm -> {target_height}mm ({elapsed:.0f}s)")
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Moving to {target_height}mm... (current: {self._current_position:.1f}mm)"
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
        """中断升降 - 发送停止命令"""
        super().halt()
        self.log_info("Lift halted - sending STOP command")
        
        # 发送停止命令
        if self.ros_node and self._lift_client:
            try:
                from qyh_lift_msgs.srv import LiftControl
                request = LiftControl.Request()
                request.command = 8  # CMD_STOP
                request.value = 0.0
                request.hold = False
                
                # 异步发送，不等待响应
                self._lift_client.call_async(request)
                self.log_info("Lift STOP command sent")
            except Exception as e:
                self.log_error(f"Failed to send STOP command: {e}")
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._is_moving = False
        self._command_sent = False
        self._start_time = None
        self._target_height = None
        self._position_reached = False
        self._status_received = False  # 重置状态接收标志
        if hasattr(self, '_last_progress_log'):
            delattr(self, '_last_progress_log')


class LiftStopNode(SkillNode):
    """
    升降电机停止节点 - 双重保障机制
    1. 发送当前位置作为目标位置
    2. 发送 CMD_STOP 命令
    """
    
    NODE_TYPE = "LiftStop"
    
    PARAM_SCHEMA = {}
    
    def __init__(self, node_id: str, params: dict = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._status_sub = None
        self._current_position = 0.0
        self._status_received = False
    
    def setup(self) -> bool:
        if not self.ros_node:
            return True
        
        try:
            from qyh_lift_msgs.msg import LiftState
            # 订阅状态以获取当前位置
            self._status_sub = self.ros_node.create_subscription(
                LiftState,
                '/lift/state',
                self._status_callback,
                10
            )
            return True
        except Exception as e:
            self.log_warn(f"Failed to setup lift state subscription: {e}")
            return True
    
    def _status_callback(self, msg):
        """状态回调 - 记录当前位置"""
        self._current_position = msg.current_position
        self._status_received = True
    
    def execute(self) -> SkillResult:
        self.log_info("Stopping lift with dual-safety mechanism")
        
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
            
            # 保障1: 发送当前位置作为目标位置（如果已收到状态）
            if self._status_received:
                self.log_info(f"  Safety 1: Setting target to current position: {self._current_position:.2f}mm")
                request_pos = LiftControl.Request()
                request_pos.command = 4  # CMD_GO_POSITION
                request_pos.value = self._current_position
                request_pos.hold = False
                client.call_async(request_pos)
            else:
                self.log_warn("  No lift state received, skipping position-based stop")
            
            # 保障2: 发送 CMD_STOP
            self.log_info("  Safety 2: Sending CMD_STOP")
            request_stop = LiftControl.Request()
            request_stop.command = 8  # CMD_STOP
            request_stop.value = 0.0
            request_stop.hold = False
            client.call_async(request_stop)
            
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Lift stop commands sent (dual-safety)"
            )
        except Exception as e:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Failed to stop lift: {e}"
            )
