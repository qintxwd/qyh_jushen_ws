"""
逻辑技能节点 (Logic Skills)
"""

import time
from typing import Dict, Any

from ..base_node import SkillNode, SkillStatus, SkillResult


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
        return True
    
    def execute(self) -> SkillResult:
        duration = self.params.get('duration', 1.0)
        
        if self._wait_start is None:
            self._wait_start = time.time()
            self.log_info(f"Waiting for {duration}s")
        
        elapsed = time.time() - self._wait_start
        
        if elapsed >= duration:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"Wait completed ({duration}s)"
            )
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Waiting... {elapsed:.1f}/{duration}s"
        )


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
        return True
    
    def execute(self) -> SkillResult:
        condition = self.params.get('condition', '')
        
        try:
            result = self._evaluate_condition(condition)
            
            if result:
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message=f"Condition '{condition}' is True"
                )
            else:
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"Condition '{condition}' is False"
                )
        except Exception as e:
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
