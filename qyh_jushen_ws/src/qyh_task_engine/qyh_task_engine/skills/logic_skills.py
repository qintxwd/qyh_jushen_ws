"""
逻辑技能节点 (Logic Skills)
"""

import time
from typing import Dict, Any, Optional, Union

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


class WaitNode(SkillNode):
    """
    等待节点
    
    参数:
        duration: 等待时间 (秒)
    """
    
    NODE_TYPE = "Wait"
    
    PARAM_SCHEMA = {
        'duration': {'type': 'float', 'required': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._wait_start = None
    
    def setup(self) -> bool:
        self.log_info(f"[Wait] Setup - ID: {self.node_id}, duration={self.params.get('duration')}s")
        return True
    
    def execute(self) -> SkillResult:
        duration = self.params.get('duration', 1.0)
        
        if self._wait_start is None:
            self._wait_start = time.time()
            self.log_info(f"[Wait] Waiting for {duration}s...")
        
        elapsed = time.time() - self._wait_start
        
        if elapsed >= duration:
            self.log_info(f"[Wait] Completed ({duration}s)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Wait completed ({duration}s)"
            )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Waiting... {elapsed:.1f}/{duration}s"
        )
    
    def reset(self):
        """重置节点状态，确保下次执行时重新计时"""
        super().reset()
        self._wait_start = None


class CheckConditionNode(SkillNode):
    """
    条件检查节点
    
    参数:
        condition: 条件表达式 (从黑板读取变量)
        - 格式: "blackboard_key operator value"
        - 例如: "arm.left.is_idle == True"
        - 支持的操作符: ==, !=, >, <, >=, <=
    """
    
    NODE_TYPE = "CheckCondition"
    
    PARAM_SCHEMA = {
        'condition': {'type': 'string', 'required': True},
    }
    
    def setup(self) -> bool:
        self.log_info(f"[CheckCondition] Setup - condition: {self.params.get('condition')}")
        return True
    
    def execute(self) -> SkillResult:
        condition = self.params.get('condition', '')
        
        self.log_info(f"[CheckCondition] Evaluating: {condition}")
        
        try:
            result = self._evaluate_condition(condition)
            
            if result:
                self.log_info(f"[CheckCondition] Result: TRUE")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Condition '{condition}' is True"
                )
            else:
                self.log_info(f"[CheckCondition] Result: FALSE")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Condition '{condition}' is False"
                )
        except Exception as e:
            self.log_error(f"[CheckCondition] Error: {e}")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Failed to evaluate condition: {e}"
            )
    
    def _evaluate_condition(self, condition: str) -> bool:
        """评估条件表达式"""
        # 解析条件
        operators = ['==', '!=', '>=', '<=', '>', '<']
        
        for op in operators:
            if op in condition:
                parts = condition.split(op)
                if len(parts) == 2:
                    key = parts[0].strip()
                    value_str = parts[1].strip()
                    
                    # 从黑板读取值
                    actual_value = self.read_from_blackboard(key)
                    
                    # 解析期望值
                    expected_value = self._parse_value(value_str)
                    
                    # 比较
                    return self._compare(actual_value, expected_value, op)
        
        # 如果没有操作符，尝试直接作为布尔值检查
        value = self.read_from_blackboard(condition)
        return bool(value)
    
    def _parse_value(self, value_str: str) -> Any:
        """解析值字符串"""
        value_str = value_str.strip()
        
        if value_str.lower() == 'true':
            return True
        if value_str.lower() == 'false':
            return False
        if value_str.lower() == 'none':
            return None
        
        try:
            return int(value_str)
        except ValueError:
            pass
        
        try:
            return float(value_str)
        except ValueError:
            pass
        
        # 字符串（去掉引号）
        if (value_str.startswith('"') and value_str.endswith('"')) or \
           (value_str.startswith("'") and value_str.endswith("'")):
            return value_str[1:-1]
        
        return value_str
    
    def _compare(self, actual: Any, expected: Any, op: str) -> bool:
        """比较两个值"""
        if op == '==':
            return actual == expected
        elif op == '!=':
            return actual != expected
        elif op == '>':
            return actual > expected
        elif op == '<':
            return actual < expected
        elif op == '>=':
            return actual >= expected
        elif op == '<=':
            return actual <= expected
        return False


class SubTaskNode(SkillNode):
    """
    子任务引用节点
    
    引用并执行另一个已定义的任务。
    支持从已保存任务文件加载或从预设模板加载。
    
    参数:
        task_id: 任务 ID（从已保存任务列表选择）
        task_name: 任务模板名称或 ID（兼容旧格式）
        params: 传递给子任务的参数（覆盖子任务的默认参数）
    """
    
    NODE_TYPE = "SubTask"
    
    PARAM_SCHEMA = {
        'task_id': {'type': 'string', 'required': False},
        'task_name': {'type': 'string', 'required': False},  # 兼容旧格式
        'params': {'type': 'object', 'required': False, 'default': {}},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._sub_tree = None
        self._sub_engine = None
        self._is_started = False
    
    def setup(self) -> bool:
        """加载子任务树"""
        self.log_info("="*40)
        self.log_info(f"[SubTask] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        task_id = self.params.get('task_id')
        task_name = self.params.get('task_name')  # 兼容旧格式
        
        if not task_id and not task_name:
            self.log_error("  Missing task_id or task_name")
            return False
        
        task_tree = None
        task_label = task_id or task_name
        
        # 1. 首先尝试从已保存任务文件加载
        if task_id:
            self.log_info(f"  Loading from task file: {task_id}")
            task_tree = self._load_from_task_file(task_id)
        
        # 2. 如果没找到，尝试从预设模板加载
        if task_tree is None:
            self.log_info(f"  Loading from preset template...")
            task_template = preset_loader.get_task_template(task_name or task_id)
            if task_template:
                task_tree = task_template.get('task_tree') or task_template.get('root')
        
        if task_tree is None:
            self.log_error(f"  Task not found: {task_label}")
            return False
        
        # 应用参数覆盖
        override_params = self.params.get('params', {})
        if override_params:
            self.log_info(f"  Applying param overrides: {override_params}")
            task_tree = self._apply_params(task_tree, override_params)
        
        self.log_info(f"  Sub-task loaded: {task_label}")
        self.log_info("="*40)
        
        # 保存任务树，稍后在 execute 中解析
        self._task_tree_data = task_tree
        return True
    
    def _load_from_task_file(self, task_id: str):
        """从持久化任务文件加载"""
        import json
        from pathlib import Path
        
        tasks_dir = Path.home() / "qyh_jushen_ws" / "persistent" / "tasks"
        task_file = tasks_dir / f"{task_id}.json"
        
        if not task_file.exists():
            return None
        
        try:
            with open(task_file, 'r', encoding='utf-8') as f:
                task_data = json.load(f)
            return task_data.get('root')
        except Exception as e:
            self.log_error(f"Failed to load task file {task_id}: {e}")
            return None
    
    def execute(self) -> SkillResult:
        """执行子任务"""
        if not self._is_started:
            # 延迟解析子任务树（避免循环导入）
            from ..parser import TaskParser
            from ..engine import BehaviorTreeEngine
            
            parser = TaskParser(ros_node=self.ros_node)
            parser.blackboard = self.blackboard  # 共享黑板
            
            try:
                # 解析子任务
                task_data = {'root': self._task_tree_data}
                self._sub_tree = parser.parse(task_data)
                
                # 创建子引擎（共享 ROS 节点）
                self._sub_engine = BehaviorTreeEngine(
                    ros_node=self.ros_node,
                    tick_rate=10.0
                )
                self._sub_engine._root = self._sub_tree
                self._sub_engine._blackboard = self.blackboard
                
                # 设置所有节点
                self._setup_tree(self._sub_tree)
                
                self._is_started = True
                self.log_info("Sub-task started")
            except Exception as e:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Failed to parse sub-task: {e}"
                )
        
        # 执行一次 tick
        result = self._tick_tree(self._sub_tree)
        
        if result.status == SkillStatus.SUCCESS:
            self.log_info("Sub-task completed successfully")
        elif result.status == SkillStatus.FAILURE:
            self.log_error(f"Sub-task failed: {result.message}")
        
        return result
    
    def _setup_tree(self, node):
        """递归设置节点"""
        if hasattr(node, 'setup'):
            node.setup()
        if hasattr(node, 'children'):
            for child in node.children:
                self._setup_tree(child)
    
    def _tick_tree(self, node) -> SkillResult:
        """递归执行节点树"""
        if hasattr(node, 'tick'):
            return node.tick()
        elif hasattr(node, 'execute'):
            return node.execute()
        return SkillResult(status=SkillStatus.FAILURE, message="Invalid node")
    
    def _apply_params(
        self, 
        tree_data: Dict, 
        params: Dict
    ) -> Dict:
        """递归应用参数覆盖"""
        import copy
        result = copy.deepcopy(tree_data)
        
        # 替换节点参数中的占位符
        if 'params' in result:
            for key, value in result['params'].items():
                if isinstance(value, str) and value.startswith('$'):
                    param_name = value[1:]
                    if param_name in params:
                        result['params'][key] = params[param_name]
        
        # 递归处理子节点
        if 'children' in result:
            result['children'] = [
                self._apply_params(child, params) 
                for child in result['children']
            ]
        
        return result
    
    def halt(self):
        """中断子任务"""
        super().halt()
        if self._sub_tree and hasattr(self._sub_tree, 'halt'):
            self._sub_tree.halt()
        self.log_info("Sub-task halted")
