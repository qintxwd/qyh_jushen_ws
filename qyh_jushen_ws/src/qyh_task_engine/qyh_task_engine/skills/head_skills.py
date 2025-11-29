"""
头部技能节点 (Head Skills)
"""

import time
from typing import Dict, Any

from ..base_node import SkillNode, SkillStatus, SkillResult


class HeadLookAtNode(SkillNode):
    """
    头部转向节点
    
    参数:
        pitch: 俯仰角 (-1.0 到 1.0 归一化值，或弧度)
        yaw: 偏航角 (-1.0 到 1.0 归一化值，或弧度)
        use_normalized: 是否使用归一化值，默认 True
        wait_stable: 是否等待稳定，默认 True
    """
    
    NODE_TYPE = "HeadLookAt"
    
    PARAM_SCHEMA = {
        'pitch': {'type': 'float', 'required': True},
        'yaw': {'type': 'float', 'required': True},
        'use_normalized': {'type': 'bool', 'default': True},
        'wait_stable': {'type': 'bool', 'default': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._pan_pub = None
        self._tilt_pub = None
        self._command_sent = False
        self._wait_start = None
    
    def setup(self) -> bool:
        """初始化头部控制发布器"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from std_msgs.msg import Float64
            self._pan_pub = self.ros_node.create_publisher(
                Float64, '/head/pan_normalized', 10
            )
            self._tilt_pub = self.ros_node.create_publisher(
                Float64, '/head/tilt_normalized', 10
            )
            return True
        except Exception as e:
            self.log_error(f"Failed to create head publishers: {e}")
            return False
    
    def execute(self) -> SkillResult:
        """执行头部转向"""
        pitch = self.params.get('pitch', 0.0)
        yaw = self.params.get('yaw', 0.0)
        wait_stable = self.params.get('wait_stable', True)
        
        self.log_info(f"Looking at pitch={pitch:.2f}, yaw={yaw:.2f}")
        
        # Mock 模式
        if not self._pan_pub:
            time.sleep(0.3)
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Head moved (mock mode)"
            )
        
        # 发送命令
        if not self._command_sent:
            from std_msgs.msg import Float64
            
            pan_msg = Float64()
            pan_msg.data = yaw
            self._pan_pub.publish(pan_msg)
            
            tilt_msg = Float64()
            tilt_msg.data = pitch
            self._tilt_pub.publish(tilt_msg)
            
            self._command_sent = True
            self._wait_start = time.time()
            
            if not wait_stable:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message="Head command sent"
                )
        
        # 等待稳定（简单延时）
        if time.time() - self._wait_start > 0.5:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Head position reached"
            )
        
        return SkillResult(status=SkillStatus.RUNNING, message="Waiting for head to stabilize...")


class HeadScanNode(SkillNode):
    """
    头部扫描节点
    
    参数:
        pattern: 扫描模式 ("left_right", "up_down", "circle")
        speed: 扫描速度 (0.1 - 1.0)，默认 0.5
        repeat: 重复次数，默认 1
    """
    
    NODE_TYPE = "HeadScan"
    
    PARAM_SCHEMA = {
        'pattern': {'type': 'string', 'default': 'left_right', 'enum': ['left_right', 'up_down', 'circle']},
        'speed': {'type': 'float', 'default': 0.5},
        'repeat': {'type': 'int', 'default': 1},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._pan_pub = None
        self._tilt_pub = None
        self._scan_step = 0
        self._current_repeat = 0
        self._step_start_time = None
    
    def setup(self) -> bool:
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from std_msgs.msg import Float64
            self._pan_pub = self.ros_node.create_publisher(
                Float64, '/head/pan_normalized', 10
            )
            self._tilt_pub = self.ros_node.create_publisher(
                Float64, '/head/tilt_normalized', 10
            )
            return True
        except Exception as e:
            self.log_error(f"Failed to create head publishers: {e}")
            return False
    
    def execute(self) -> SkillResult:
        pattern = self.params.get('pattern', 'left_right')
        speed = self.params.get('speed', 0.5)
        repeat = self.params.get('repeat', 1)
        
        # Mock 模式
        if not self._pan_pub:
            time.sleep(1.0 / speed)
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Head scan {pattern} completed (mock mode)"
            )
        
        # 定义扫描序列
        scan_patterns = {
            'left_right': [(0.0, -0.5), (0.0, 0.5), (0.0, 0.0)],
            'up_down': [(-0.3, 0.0), (0.3, 0.0), (0.0, 0.0)],
            'circle': [(0.2, -0.3), (0.2, 0.3), (-0.2, 0.3), (-0.2, -0.3), (0.0, 0.0)],
        }
        
        steps = scan_patterns.get(pattern, scan_patterns['left_right'])
        step_duration = 0.5 / speed
        
        # 初始化步骤
        if self._step_start_time is None:
            self._step_start_time = time.time()
            self._publish_head_position(steps[0][0], steps[0][1])
        
        # 检查当前步骤是否完成
        elapsed = time.time() - self._step_start_time
        if elapsed >= step_duration:
            self._scan_step += 1
            
            if self._scan_step >= len(steps):
                self._current_repeat += 1
                if self._current_repeat >= repeat:
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=f"Head scan {pattern} completed"
                    )
                self._scan_step = 0
            
            self._step_start_time = time.time()
            pitch, yaw = steps[self._scan_step]
            self._publish_head_position(pitch, yaw)
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Scanning... step {self._scan_step + 1}/{len(steps)}, repeat {self._current_repeat + 1}/{repeat}"
        )
    
    def _publish_head_position(self, pitch: float, yaw: float):
        """发布头部位置"""
        from std_msgs.msg import Float64
        
        pan_msg = Float64()
        pan_msg.data = yaw
        self._pan_pub.publish(pan_msg)
        
        tilt_msg = Float64()
        tilt_msg.data = pitch
        self._tilt_pub.publish(tilt_msg)
