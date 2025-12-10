"""
夹爪技能节点 (Gripper Skills)
"""

import time
from typing import Dict, Any, Optional

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


class GripperControlNode(SkillNode):
    """
    夹爪控制节点
    
    参数:
        side: 机械臂选择 ("left", "right")
        action: 动作 ("open", "close")
        position: 夹爪位置 (0.0-1.0)，可选
        position_name: 预设位置名称（可选）
        force: 夹持力（可选），默认使用系统默认值
        wait_complete: 是否等待完成，默认 True
        timeout: 超时时间 (秒)，默认 10
    """
    
    NODE_TYPE = "GripperControl"
    
    PARAM_SCHEMA = {
        'side': {'type': 'string', 'required': True, 'enum': ['left', 'right']},
        'action': {'type': 'string', 'required': False, 'enum': ['open', 'close']},
        'position': {'type': 'float', 'required': False},
        'position_name': {'type': 'string', 'required': False},
        'force': {'type': 'float', 'required': False},
        'wait_complete': {'type': 'bool', 'default': True},
        'timeout': {'type': 'float', 'default': 10.0},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._gripper_client = None
        self._status_sub = None
        self._command_sent = False
        self._start_time: Optional[float] = None
        # 夹爪状态
        self._is_moving = False
        self._object_status = 0  # 0:moving, 1:inner, 2:outer(gripped), 3:arrived
        self._status_received = False
    
    def setup(self) -> bool:
        """初始化夹爪服务客户端"""
        self.log_info("="*40)
        self.log_info(f"[GripperControl] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        if not self.ros_node:
            self.log_warn("  No ROS node, running in MOCK mode")
            return True
        
        try:
            from qyh_gripper_msgs.srv import MoveGripper
            from qyh_gripper_msgs.msg import GripperState
            
            side = self.params.get('side', 'left')
            service_name = f'/gripper/{side}/move'
            self._gripper_client = self.ros_node.create_client(
                MoveGripper, service_name
            )
            self.log_info(f"  Gripper client created: {service_name}")
            
            # 订阅夹爪状态
            state_topic = f'/gripper/{side}/state'
            self._status_sub = self.ros_node.create_subscription(
                GripperState,
                state_topic,
                self._status_callback,
                10
            )
            self.log_info(f"  Subscribed to {state_topic}")
            self.log_info("="*40)
            return True
        except Exception as e:
            self.log_error(f"  Failed to create gripper client: {e}")
            return False
    
    def _status_callback(self, msg):
        """夹爪状态回调"""
        self._is_moving = msg.is_moving
        self._object_status = msg.object_status
        # 只有在命令发送后才标记状态已接收
        if self._command_sent:
            self._status_received = True
    
    def execute(self) -> SkillResult:
        """执行夹爪控制"""
        side = self.params.get('side', 'left')
        force = self.params.get('force', 100)  # 默认力度
        wait_complete = self.params.get('wait_complete', True)
        timeout = self.params.get('timeout', 10.0)
        
        # 解析动作/位置
        action, position = self._resolve_action()
        if action is None and position is None:
            self.log_error("Missing action or position")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Missing action or position"
            )
        
        # Mock 模式
        if not self._gripper_client:
            if self._start_time is None:
                self._start_time = time.time()
                self.log_info(f"[Gripper] {side}: action={action}, pos={position} (mock)")
            
            elapsed = time.time() - self._start_time
            if elapsed > 0.5:
                self.log_info(f"[Gripper] {action} completed (mock)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Gripper {action} completed (mock mode)"
                )
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Gripper {action}... {elapsed:.1f}s"
            )
        
        # 真实执行 - 发送命令
        if not self._command_sent:
            self.log_info(f"[Gripper] {side}: action={action}, pos={position}")
            
            if not self._gripper_client.wait_for_service(timeout_sec=1.0):
                self.log_error("Gripper service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Gripper service not available"
                )
            
            from qyh_gripper_msgs.srv import MoveGripper
            request = MoveGripper.Request()
            # position: 0=open, 255=close
            request.position = int(position * 255) if position is not None else (0 if action == 'open' else 255)
            request.speed = 255  # 最大速度
            request.force = int(force) if force is not None else 100
            
            self.log_info(f"   Sending gripper command: pos={request.position}, force={request.force}")
            # Fire-and-forget
            self._gripper_client.call_async(request)
            
            self._command_sent = True
            self._start_time = time.time()
            
            # 如果不等待完成，立即返回成功
            if not wait_complete:
                self.log_info(f"[Gripper] Command sent (no wait)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Gripper {action} command sent"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Gripper {action}..."
            )
        
        # 检查超时
        elapsed = time.time() - self._start_time
        if elapsed > timeout:
            self.log_error(f"Gripper timeout ({timeout}s)")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Gripper timeout ({timeout}s)"
            )
        
        # 发送命令后等待至少 0.5 秒再检查状态（等待夹爪开始运动）
        MIN_WAIT_TIME = 0.5
        if elapsed < MIN_WAIT_TIME:
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Command sent, waiting... {elapsed:.1f}s"
            )
        
        # 检查是否完成: object_status != 0 (not moving) 或 is_moving == False
        # object_status: 0=moving, 1=inner_detected, 2=outer_detected(gripped), 3=arrived
        if not self._is_moving or self._object_status != 0:
            status_names = {0: 'moving', 1: 'inner_detected', 2: 'gripped', 3: 'arrived'}
            status_name = status_names.get(self._object_status, 'unknown')
            self.log_info(f"[Gripper] ✓ {action} completed (status: {status_name}, {elapsed:.1f}s)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Gripper {action} completed ({status_name})"
            )
        
        return SkillResult(
            status=SkillStatus.RUNNING, 
            message=f"Gripper {action}... {elapsed:.1f}s"
        )

    def _resolve_action(self) -> tuple:
        """
        解析夹爪动作/位置
        
        Returns:
            (action, position) tuple
        """
        side = self.params.get('side', 'left')
        
        # 使用预设位置名称
        if 'position_name' in self.params:
            pos_name = self.params['position_name']
            
            pos_preset = preset_loader.get_gripper_position(pos_name)
            if pos_preset:
                if side == 'left':
                    position = pos_preset.get('left_position')
                else:
                    position = pos_preset.get('right_position')
                
                if position is not None:
                    action = 'open' if position > 0.5 else 'close'
                    return (action, position)
            
            self.log_error(f"Unknown gripper position: {pos_name}")
            return (None, None)
        
        # 直接指定位置
        if 'position' in self.params:
            position = self.params['position']
            action = 'open' if position > 0.5 else 'close'
            return (action, position)
        
        # 使用动作
        if 'action' in self.params:
            action = self.params['action']
            position = 1.0 if action == 'open' else 0.0
            return (action, position)
        
        return (None, None)
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._command_sent = False
        self._start_time = None
        self._is_moving = False
        self._object_status = 0
        self._status_received = False
