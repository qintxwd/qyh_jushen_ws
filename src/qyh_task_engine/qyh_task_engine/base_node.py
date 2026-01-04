"""
技能节点基类 (Skill Node Base Class)

定义所有行为树叶子节点的抽象接口，包括:
- setup: 初始化
- execute: 执行核心逻辑
- halt: 中断处理
"""

import time
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, Optional, TYPE_CHECKING
from dataclasses import dataclass, field

if TYPE_CHECKING:
    import rclpy.node


class SkillStatus(Enum):
    """技能执行状态"""
    IDLE = "idle"           # 空闲，未开始
    RUNNING = "running"     # 执行中
    SUCCESS = "success"     # 成功完成
    FAILURE = "failure"     # 执行失败
    HALTED = "halted"       # 被中断


@dataclass
class SkillResult:
    """技能执行结果"""
    status: SkillStatus
    message: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


class SkillNode(ABC):
    """
    技能节点基类
    
    所有具体的技能节点（如 ArmMoveJ, GripperControl 等）都应该继承此类。
    每个节点封装一个原子操作，对应行为树中的叶子节点。
    
    Attributes:
        node_id: 节点唯一标识符
        node_type: 节点类型名称
        node_name: 节点显示名称
        params: 节点参数
        status: 当前执行状态
        blackboard: 共享数据黑板（用于节点间通信）
        ros_node: ROS2 节点引用（用于调用服务/动作）
    """
    
    # 节点类型名称（子类必须重写）
    NODE_TYPE: str = "BaseSkill"
    
    # 参数定义（子类应该重写，用于参数校验）
    PARAM_SCHEMA: Dict[str, Any] = {}
    
    def __init__(
        self,
        node_id: str,
        params: Dict[str, Any] = None,
        blackboard: Dict[str, Any] = None,
        ros_node: 'rclpy.node.Node' = None,
        node_name: str = None
    ):
        """
        初始化技能节点
        
        Args:
            node_id: 节点唯一标识符
            params: 节点参数字典
            blackboard: 共享数据黑板
            ros_node: ROS2 节点引用
            node_name: 节点显示名称
        """
        self.node_id = node_id
        self.node_type = self.__class__.NODE_TYPE
        self.node_name = node_name or self.node_type
        self.params = params or {}
        self.blackboard = blackboard if blackboard is not None else {}
        self.ros_node = ros_node
        
        self.status = SkillStatus.IDLE
        self._halt_requested = False
        self._result: Optional[SkillResult] = None
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
    
    def validate_params(self) -> bool:
        """
        验证节点参数
        
        Returns:
            参数是否有效
        """
        for param_name, param_def in self.PARAM_SCHEMA.items():
            if param_def.get('required', False):
                if param_name not in self.params:
                    return False
        return True
    
    @abstractmethod
    def setup(self) -> bool:
        """
        节点初始化（在首次 tick 前调用）
        
        Returns:
            初始化是否成功
        """
        pass
    
    @abstractmethod
    def execute(self) -> SkillResult:
        """
        执行技能的核心逻辑 (同步版本)
        
        这是子类必须实现的主要方法。
        
        Returns:
            SkillResult 包含执行状态和结果
        """
        pass
    
    def tick(self) -> SkillStatus:
        """
        行为树 Tick 方法 (同步版本)
        
        每个 Tick 周期调用一次，返回当前状态。
        
        Returns:
            当前节点状态
        """
        self.log_info(f"[DEBUG] SkillNode.tick() called, current status: {self.status}")
        
        # 检查中断请求
        if self._halt_requested:
            self.log_info(f"[DEBUG] Halt requested")
            self.status = SkillStatus.HALTED
            self._end_time = time.time()
            return self.status
        
        # 如果还没开始，先执行 setup
        if self.status == SkillStatus.IDLE:
            self.log_info(f"[DEBUG] Status is IDLE, calling setup()...")
            self._start_time = time.time()
            if not self.setup():
                self.log_info(f"[DEBUG] setup() returned False")
                self.status = SkillStatus.FAILURE
                self._result = SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Setup failed"
                )
                self._end_time = time.time()
                return self.status
            self.log_info(f"[DEBUG] setup() returned True, status -> RUNNING")
            self.status = SkillStatus.RUNNING
        
        # 如果正在运行，继续执行
        if self.status == SkillStatus.RUNNING:
            self.log_info(f"[DEBUG] Status is RUNNING, calling execute()...")
            try:
                result = self.execute()
                self.log_info(f"[DEBUG] execute() returned: {result.status}")
                self._result = result
                self.status = result.status
                if self.status != SkillStatus.RUNNING:
                    self._end_time = time.time()
            except Exception as e:
                self.log_info(f"[DEBUG] execute() raised exception: {e}")
                import traceback
                self.log_info(f"[DEBUG] Traceback: {traceback.format_exc()}")
                self.status = SkillStatus.FAILURE
                self._result = SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Execution error: {str(e)}"
                )
                self._end_time = time.time()
        
        self.log_info(f"[DEBUG] SkillNode.tick() returning: {self.status}")
        return self.status
    
    def halt(self):
        """
        请求中断节点执行
        
        子类可以重写此方法以实现自定义的中断逻辑（如发送急停命令）。
        """
        self._halt_requested = True
    
    def reset(self):
        """
        重置节点状态
        
        将节点恢复到初始状态，以便重新执行。
        """
        self.status = SkillStatus.IDLE
        self._halt_requested = False
        self._result = None
        self._start_time = None
        self._end_time = None
    
    def get_result(self) -> Optional[SkillResult]:
        """获取执行结果"""
        return self._result
    
    def get_duration(self) -> float:
        """获取执行耗时（秒）"""
        if self._start_time is None:
            return 0.0
        end = self._end_time or time.time()
        return end - self._start_time
    
    def get_state(self) -> Dict[str, Any]:
        """
        获取节点状态（用于前端显示）
        
        Returns:
            包含节点 ID、类型、状态等信息的字典
        """
        return {
            'node_id': self.node_id,
            'node_type': self.node_type,
            'node_name': self.node_name,
            'status': self.status.value,
            'params': self.params,
            'duration': self.get_duration(),
            'result': {
                'message': self._result.message if self._result else '',
                'data': self._result.data if self._result else {}
            } if self._result else None
        }
    
    def read_from_blackboard(self, key: str, default: Any = None) -> Any:
        """
        从黑板读取数据
        
        Args:
            key: 数据键名（支持点分隔的嵌套键，如 'objects.cup.pose'）
            default: 默认值
        
        Returns:
            读取到的数据或默认值
        """
        keys = key.split('.')
        data = self.blackboard
        for k in keys:
            if isinstance(data, dict) and k in data:
                data = data[k]
            else:
                return default
        return data
    
    def write_to_blackboard(self, key: str, value: Any):
        """
        向黑板写入数据
        
        Args:
            key: 数据键名（支持点分隔的嵌套键）
            value: 要写入的值
        """
        keys = key.split('.')
        data = self.blackboard
        for k in keys[:-1]:
            if k not in data:
                data[k] = {}
            data = data[k]
        data[keys[-1]] = value
    
    def log_info(self, msg: str):
        """输出信息日志"""
        if self.ros_node:
            self.ros_node.get_logger().info(f"[{self.node_name}] {msg}")
        else:
            print(f"[INFO] [{self.node_name}] {msg}")
    
    def log_warn(self, msg: str):
        """输出警告日志"""
        if self.ros_node:
            self.ros_node.get_logger().warn(f"[{self.node_name}] {msg}")
        else:
            print(f"[WARN] [{self.node_name}] {msg}")
    
    def log_error(self, msg: str):
        """输出错误日志"""
        if self.ros_node:
            self.ros_node.get_logger().error(f"[{self.node_name}] {msg}")
        else:
            print(f"[ERROR] [{self.node_name}] {msg}")
    
    def __repr__(self) -> str:
        return f"<{self.node_type}(id={self.node_id}, status={self.status.value})>"
