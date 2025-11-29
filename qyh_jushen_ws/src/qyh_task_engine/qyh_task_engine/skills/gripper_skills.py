"""
夹爪技能节点 (Gripper Skills)
"""

import time
from typing import Dict, Any

from ..base_node import SkillNode, SkillStatus, SkillResult


class GripperControlNode(SkillNode):
    """
    夹爪控制节点
    
    参数:
        side: 机械臂选择 ("left", "right")
        action: 动作 ("open", "close")
        force: 夹持力（可选），默认使用系统默认值
        wait_complete: 是否等待完成，默认 True
    """
    
    NODE_TYPE = "GripperControl"
    
    PARAM_SCHEMA = {
        'side': {'type': 'string', 'required': True, 'enum': ['left', 'right']},
        'action': {'type': 'string', 'required': True, 'enum': ['open', 'close']},
        'force': {'type': 'float', 'required': False},
        'wait_complete': {'type': 'bool', 'default': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._gripper_client = None
        self._future = None
        self._is_executing = False
    
    def setup(self) -> bool:
        """初始化夹爪服务客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_gripper_msgs.srv import GripperCommand
            side = self.params.get('side', 'left')
            service_name = f'/gripper/{side}/command'
            self._gripper_client = self.ros_node.create_client(
                GripperCommand, service_name
            )
            return True
        except Exception as e:
            self.log_error(f"Failed to create gripper client: {e}")
            return False
    
    def execute(self) -> SkillResult:
        """执行夹爪控制"""
        side = self.params.get('side', 'left')
        action = self.params.get('action', 'open')
        force = self.params.get('force')
        
        self.log_info(f"{action.capitalize()} {side} gripper")
        
        # Mock 模式
        if not self._gripper_client:
            time.sleep(0.3)
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Gripper {action} completed (mock mode)"
            )
        
        # 真实执行
        if not self._is_executing:
            from qyh_gripper_msgs.srv import GripperCommand
            
            if not self._gripper_client.wait_for_service(timeout_sec=1.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Gripper service not available"
                )
            
            request = GripperCommand.Request()
            request.command = action
            if force is not None:
                request.force = force
            
            self._future = self._gripper_client.call_async(request)
            self._is_executing = True
            return SkillResult(status=SkillStatus.RUNNING, message=f"Executing gripper {action}...")
        
        if self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=response.message
                    )
                else:
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=response.message
                    )
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Gripper control failed: {e}"
                )
        
        return SkillResult(status=SkillStatus.RUNNING, message=f"Executing gripper {action}...")
