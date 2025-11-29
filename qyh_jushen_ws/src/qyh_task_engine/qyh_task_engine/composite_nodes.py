"""
复合节点 (Composite Nodes)

实现行为树的控制流节点:
- Sequence: 顺序执行，全部成功才成功
- Parallel: 并行执行，可配置成功策略
- Selector: 选择执行，一个成功即成功
"""

import time
from typing import List, Dict, Any, Optional
from enum import Enum

from .base_node import SkillNode, SkillStatus, SkillResult


class CompositeNode:
    """
    复合节点基类
    
    复合节点包含多个子节点，负责控制子节点的执行流程。
    """
    
    NODE_TYPE: str = "Composite"
    
    def __init__(
        self,
        node_id: str,
        children: List[SkillNode] = None,
        node_name: str = None,
        blackboard: Dict[str, Any] = None
    ):
        self.node_id = node_id
        self.node_type = self.__class__.NODE_TYPE
        self.node_name = node_name or self.node_type
        self.children = children or []
        self.blackboard = blackboard if blackboard is not None else {}
        
        self.status = SkillStatus.IDLE
        self._current_child_index = 0
        self._halt_requested = False
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
    
    def add_child(self, child: SkillNode):
        """添加子节点"""
        self.children.append(child)
        # 共享黑板
        if hasattr(child, 'blackboard'):
            child.blackboard = self.blackboard
    
    def tick(self) -> SkillStatus:
        """Tick 方法，子类需要重写"""
        raise NotImplementedError
    
    def halt(self):
        """中断所有子节点"""
        self._halt_requested = True
        for child in self.children:
            child.halt()
    
    def reset(self):
        """重置所有子节点"""
        self.status = SkillStatus.IDLE
        self._current_child_index = 0
        self._halt_requested = False
        self._start_time = None
        self._end_time = None
        for child in self.children:
            child.reset()
    
    def get_duration(self) -> float:
        """获取执行耗时"""
        if self._start_time is None:
            return 0.0
        end = self._end_time or time.time()
        return end - self._start_time
    
    def get_state(self) -> Dict[str, Any]:
        """获取节点状态"""
        return {
            'node_id': self.node_id,
            'node_type': self.node_type,
            'node_name': self.node_name,
            'status': self.status.value,
            'duration': self.get_duration(),
            'current_child_index': self._current_child_index,
            'children_count': len(self.children)
        }
    
    def get_all_states(self) -> List[Dict[str, Any]]:
        """获取所有节点（包括子节点）的状态"""
        states = [self.get_state()]
        for child in self.children:
            if isinstance(child, CompositeNode):
                states.extend(child.get_all_states())
            else:
                states.append(child.get_state())
        return states


class SequenceNode(CompositeNode):
    """
    顺序节点 (Sequence)
    
    按顺序执行子节点:
    - 子节点返回 SUCCESS -> 继续执行下一个
    - 子节点返回 RUNNING -> 返回 RUNNING
    - 子节点返回 FAILURE -> 立即返回 FAILURE
    - 所有子节点都 SUCCESS -> 返回 SUCCESS
    """
    
    NODE_TYPE = "Sequence"
    
    def tick(self) -> SkillStatus:
        if self._halt_requested:
            self.status = SkillStatus.HALTED
            self._end_time = time.time()
            return self.status
        
        if self.status == SkillStatus.IDLE:
            self._start_time = time.time()
            self.status = SkillStatus.RUNNING
        
        # 从当前子节点开始执行
        while self._current_child_index < len(self.children):
            child = self.children[self._current_child_index]
            child_status = child.tick()
            
            if child_status == SkillStatus.RUNNING:
                return SkillStatus.RUNNING
            
            if child_status == SkillStatus.FAILURE:
                self.status = SkillStatus.FAILURE
                self._end_time = time.time()
                return self.status
            
            if child_status == SkillStatus.HALTED:
                self.status = SkillStatus.HALTED
                self._end_time = time.time()
                return self.status
            
            # SUCCESS -> 继续下一个
            self._current_child_index += 1
        
        # 所有子节点都成功
        self.status = SkillStatus.SUCCESS
        self._end_time = time.time()
        return self.status


class ParallelNode(CompositeNode):
    """
    并行节点 (Parallel)
    
    同时执行所有子节点:
    - success_threshold: 需要多少个子节点成功才算成功 (默认全部)
    - failure_threshold: 多少个子节点失败就算失败 (默认 1)
    """
    
    NODE_TYPE = "Parallel"
    
    def __init__(
        self,
        node_id: str,
        children: List[SkillNode] = None,
        node_name: str = None,
        blackboard: Dict[str, Any] = None,
        success_threshold: int = None,
        failure_threshold: int = 1
    ):
        super().__init__(node_id, children, node_name, blackboard)
        self.success_threshold = success_threshold  # None 表示需要全部成功
        self.failure_threshold = failure_threshold
    
    def tick(self) -> SkillStatus:
        if self._halt_requested:
            self.status = SkillStatus.HALTED
            self._end_time = time.time()
            return self.status
        
        if self.status == SkillStatus.IDLE:
            self._start_time = time.time()
            self.status = SkillStatus.RUNNING
        
        success_count = 0
        failure_count = 0
        running_count = 0
        
        # Tick 所有子节点
        for child in self.children:
            if child.status in [SkillStatus.SUCCESS, SkillStatus.FAILURE, SkillStatus.HALTED]:
                # 已完成的节点不再 tick
                if child.status == SkillStatus.SUCCESS:
                    success_count += 1
                elif child.status == SkillStatus.FAILURE:
                    failure_count += 1
                continue
            
            child_status = child.tick()
            
            if child_status == SkillStatus.SUCCESS:
                success_count += 1
            elif child_status == SkillStatus.FAILURE:
                failure_count += 1
            elif child_status == SkillStatus.RUNNING:
                running_count += 1
        
        # 检查失败阈值
        if failure_count >= self.failure_threshold:
            self.status = SkillStatus.FAILURE
            self._end_time = time.time()
            # 中断其他正在运行的子节点
            for child in self.children:
                if child.status == SkillStatus.RUNNING:
                    child.halt()
            return self.status
        
        # 检查成功阈值
        threshold = self.success_threshold or len(self.children)
        if success_count >= threshold:
            self.status = SkillStatus.SUCCESS
            self._end_time = time.time()
            return self.status
        
        # 还有节点在运行
        if running_count > 0:
            return SkillStatus.RUNNING
        
        # 所有节点都完成了，但没达到成功阈值
        self.status = SkillStatus.FAILURE
        self._end_time = time.time()
        return self.status


class SelectorNode(CompositeNode):
    """
    选择节点 (Selector / Fallback)
    
    按顺序尝试子节点，直到有一个成功:
    - 子节点返回 SUCCESS -> 立即返回 SUCCESS
    - 子节点返回 RUNNING -> 返回 RUNNING
    - 子节点返回 FAILURE -> 继续尝试下一个
    - 所有子节点都 FAILURE -> 返回 FAILURE
    """
    
    NODE_TYPE = "Selector"
    
    def tick(self) -> SkillStatus:
        if self._halt_requested:
            self.status = SkillStatus.HALTED
            self._end_time = time.time()
            return self.status
        
        if self.status == SkillStatus.IDLE:
            self._start_time = time.time()
            self.status = SkillStatus.RUNNING
        
        # 从当前子节点开始尝试
        while self._current_child_index < len(self.children):
            child = self.children[self._current_child_index]
            child_status = child.tick()
            
            if child_status == SkillStatus.RUNNING:
                return SkillStatus.RUNNING
            
            if child_status == SkillStatus.SUCCESS:
                self.status = SkillStatus.SUCCESS
                self._end_time = time.time()
                return self.status
            
            if child_status == SkillStatus.HALTED:
                self.status = SkillStatus.HALTED
                self._end_time = time.time()
                return self.status
            
            # FAILURE -> 尝试下一个
            self._current_child_index += 1
        
        # 所有子节点都失败
        self.status = SkillStatus.FAILURE
        self._end_time = time.time()
        return self.status
