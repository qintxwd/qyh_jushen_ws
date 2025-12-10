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
        blackboard: Dict[str, Any] = None,
        ros_node = None
    ):
        self.node_id = node_id
        self.node_type = self.__class__.NODE_TYPE
        self.node_name = node_name or self.node_type
        self.children = children or []
        self.blackboard = blackboard if blackboard is not None else {}
        self.ros_node = ros_node
        
        self.status = SkillStatus.IDLE
        self._current_child_index = 0
        self._halt_requested = False
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
    
    def log_info(self, msg: str):
        """日志输出"""
        if self.ros_node:
            self.ros_node.get_logger().info(f"[{self.node_name}] {msg}")
        else:
            print(f"[{self.node_name}] {msg}")
    
    def log_warn(self, msg: str):
        if self.ros_node:
            self.ros_node.get_logger().warn(f"[{self.node_name}] {msg}")
        else:
            print(f"[WARN][{self.node_name}] {msg}")
    
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
            self.log_info(f"Starting sequence ({len(self.children)} steps)")
        
        # 从当前子节点开始执行
        while self._current_child_index < len(self.children):
            child = self.children[self._current_child_index]
            child_status = child.tick()
            
            if child_status == SkillStatus.RUNNING:
                return SkillStatus.RUNNING
            
            if child_status == SkillStatus.FAILURE:
                self.status = SkillStatus.FAILURE
                self._end_time = time.time()
                child_name = getattr(child, 'node_name', child.node_id)
                self.log_info(f"Sequence FAILED at step {self._current_child_index+1}/{len(self.children)}: {child_name}")
                return self.status
            
            if child_status == SkillStatus.HALTED:
                self.status = SkillStatus.HALTED
                self._end_time = time.time()
                return self.status
            
            # SUCCESS -> 继续下一个
            child_name = getattr(child, 'node_name', child.node_id)
            self.log_info(f"Step {self._current_child_index+1}/{len(self.children)} done: {child_name}")
            self._current_child_index += 1
        
        # 所有子节点都成功
        self.status = SkillStatus.SUCCESS
        self._end_time = time.time()
        elapsed = self._end_time - self._start_time
        self.log_info(f"Sequence completed ({elapsed:.1f}s)")
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
        ros_node = None,
        success_threshold: int = None,
        failure_threshold: int = 1
    ):
        super().__init__(node_id, children, node_name, blackboard, ros_node)
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


class LoopNode(CompositeNode):
    """
    循环节点 (Loop / Repeat)
    
    循环执行子节点指定次数:
    - count: 循环次数，0 表示无限循环
    - break_on_failure: 子节点失败时是否退出循环
    
    行为:
    - 按顺序执行所有子节点（类似 Sequence）
    - 一轮完成后重置子节点状态，开始下一轮
    - 达到循环次数后返回 SUCCESS
    - 如果 break_on_failure=True 且子节点失败，返回 FAILURE
    """
    
    NODE_TYPE = "Loop"
    
    def __init__(
        self,
        node_id: str,
        children: List[SkillNode] = None,
        node_name: str = None,
        blackboard: Dict[str, Any] = None,
        ros_node = None,
        count: int = 1,
        break_on_failure: bool = True
    ):
        super().__init__(node_id, children, node_name, blackboard, ros_node)
        self.count = count  # 0 表示无限循环
        self.break_on_failure = break_on_failure
        self._current_iteration = 0
    
    def tick(self) -> SkillStatus:
        print(f"DEBUG [LoopNode.tick] ENTER - node_id={self.node_id}, halt_requested={self._halt_requested}, status={self.status}")
        
        if self._halt_requested:
            print(f"DEBUG [LoopNode.tick] HALT requested, returning HALTED")
            self.status = SkillStatus.HALTED
            self._end_time = time.time()
            return self.status
        
        if self.status == SkillStatus.IDLE:
            print(f"DEBUG [LoopNode.tick] Status is IDLE, initializing loop")
            self._start_time = time.time()
            self.status = SkillStatus.RUNNING
            self._current_iteration = 0
            self._current_child_index = 0
            loop_desc = f"{self.count} times" if self.count > 0 else "infinite"
            self.log_info(f"Starting loop ({loop_desc})")
        
        print(f"DEBUG [LoopNode.tick] iteration={self._current_iteration}, count={self.count}, child_index={self._current_child_index}, num_children={len(self.children)}")
        
        # 检查是否完成所有循环（count=0 时永不完成）
        if self.count > 0 and self._current_iteration >= self.count:
            print(f"DEBUG [LoopNode.tick] All iterations done, returning SUCCESS")
            self.status = SkillStatus.SUCCESS
            self._end_time = time.time()
            elapsed = self._end_time - self._start_time
            self.log_info(f"Loop completed ({self.count} iterations, {elapsed:.1f}s)")
            return self.status
        
        # 执行当前子节点
        while self._current_child_index < len(self.children):
            child = self.children[self._current_child_index]
            child_name = getattr(child, 'node_name', child.node_id)
            print(f"DEBUG [LoopNode.tick] Ticking child[{self._current_child_index}]: {child_name}, child.status={child.status}")
            
            child_status = child.tick()
            
            print(f"DEBUG [LoopNode.tick] Child {child_name} returned: {child_status}")
            
            if child_status == SkillStatus.RUNNING:
                print(f"DEBUG [LoopNode.tick] Child RUNNING, returning RUNNING")
                return SkillStatus.RUNNING
            
            if child_status == SkillStatus.FAILURE:
                if self.break_on_failure:
                    print(f"DEBUG [LoopNode.tick] Child FAILURE with break_on_failure=True, returning FAILURE")
                    self.status = SkillStatus.FAILURE
                    self._end_time = time.time()
                    self.log_info(f"Loop FAILED at iteration {self._current_iteration+1}: {child_name}")
                    return self.status
                print(f"DEBUG [LoopNode.tick] Child FAILURE with break_on_failure=False, continuing")
                # 不中断，继续下一个子节点
            
            if child_status == SkillStatus.HALTED:
                print(f"DEBUG [LoopNode.tick] Child HALTED, returning HALTED")
                self.status = SkillStatus.HALTED
                self._end_time = time.time()
                return self.status
            
            # 继续下一个子节点
            print(f"DEBUG [LoopNode.tick] Child SUCCESS, moving to next child")
            self._current_child_index += 1
        
        # 一轮完成，重置并开始下一轮
        print(f"DEBUG [LoopNode.tick] Iteration {self._current_iteration} complete, preparing next iteration")
        self._current_iteration += 1
        self._current_child_index = 0
        
        loop_desc = f"{self.count}" if self.count > 0 else "inf"
        self.log_info(f"Loop iteration {self._current_iteration}/{loop_desc} completed")
        
        # 重置所有子节点
        print(f"DEBUG [LoopNode.tick] Resetting all children")
        for i, child in enumerate(self.children):
            child_name = getattr(child, 'node_name', child.node_id)
            print(f"DEBUG [LoopNode.tick] Resetting child[{i}]: {child_name}")
            child.reset()
        
        # 检查是否还需要继续
        if self.count > 0 and self._current_iteration >= self.count:
            print(f"DEBUG [LoopNode.tick] All iterations complete after increment, returning SUCCESS")
            self.status = SkillStatus.SUCCESS
            self._end_time = time.time()
            elapsed = self._end_time - self._start_time
            self.log_info(f"Loop completed ({self.count} iterations, {elapsed:.1f}s)")
            return self.status
        
        # 继续循环
        print(f"DEBUG [LoopNode.tick] More iterations needed, returning RUNNING")
        return SkillStatus.RUNNING
    
    def reset(self):
        """重置节点"""
        super().reset()
        self._current_iteration = 0
    
    def get_state(self) -> Dict[str, Any]:
        """获取节点状态"""
        state = super().get_state()
        state['current_iteration'] = self._current_iteration
        state['total_iterations'] = self.count if self.count > 0 else 'infinite'
        return state
