"""
QYH Task Engine - 具身智能任务编排与执行引擎

基于 py_trees 的行为树执行引擎，支持:
- 可视化编程生成的任务 JSON 解析与执行
- 原子技能封装 (机械臂、夹爪、头部、底盘)
- 实时状态反馈
- 任务暂停/恢复/取消
"""

from .base_node import SkillNode, SkillStatus, SkillResult
from .composite_nodes import SequenceNode, ParallelNode, SelectorNode
from .engine import BehaviorTreeEngine
from .parser import TaskParser

__all__ = [
    'SkillNode',
    'SkillStatus', 
    'SkillResult',
    'SequenceNode',
    'ParallelNode',
    'SelectorNode',
    'BehaviorTreeEngine',
    'TaskParser',
]
