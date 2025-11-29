"""
任务 JSON 解析器 (Task Parser)

将前端生成的任务 JSON 解析为行为树结构
"""

import json
import uuid
from typing import Dict, Any, Union, TYPE_CHECKING

from .base_node import SkillNode
from .composite_nodes import SequenceNode, ParallelNode, SelectorNode, CompositeNode
from .skills import SKILL_REGISTRY

if TYPE_CHECKING:
    import rclpy.node


class TaskParser:
    """
    任务解析器
    
    将 JSON 格式的任务描述解析为可执行的行为树
    """
    
    # 复合节点类型映射
    COMPOSITE_TYPES = {
        'Sequence': SequenceNode,
        'Parallel': ParallelNode,
        'Selector': SelectorNode,
    }
    
    def __init__(self, ros_node: 'rclpy.node.Node' = None):
        """
        初始化解析器
        
        Args:
            ros_node: ROS2 节点引用，传递给所有技能节点
        """
        self.ros_node = ros_node
        self.blackboard: Dict[str, Any] = {}
    
    def parse(self, task_json: Union[str, dict]) -> Union[SkillNode, CompositeNode]:
        """
        解析任务 JSON
        
        Args:
            task_json: JSON 字符串或字典
        
        Returns:
            根节点（SkillNode 或 CompositeNode）
        
        Raises:
            ValueError: JSON 格式错误或节点类型未知
        """
        if isinstance(task_json, str):
            try:
                task_data = json.loads(task_json)
            except json.JSONDecodeError as e:
                raise ValueError(f"Invalid JSON: {e}")
        else:
            task_data = task_json
        
        # 验证基本结构
        if 'root' not in task_data:
            raise ValueError("Missing 'root' in task JSON")
        
        # 初始化黑板（加载预设资产）
        self._init_blackboard(task_data)
        
        # 递归解析根节点
        return self._parse_node(task_data['root'])
    
    def _init_blackboard(self, task_data: dict):
        """初始化黑板数据"""
        # 加载预设资产
        self.blackboard['assets'] = {
            'locations': task_data.get('assets', {}).get('locations', {}),
            'poses': task_data.get('assets', {}).get('poses', {}),
            'speed_profiles': task_data.get('assets', {}).get('speed_profiles', {}),
        }
        
        # 内置预设
        self._add_builtin_assets()
    
    def _add_builtin_assets(self):
        """添加内置预设资产"""
        # 内置点位
        if 'locations' not in self.blackboard['assets']:
            self.blackboard['assets']['locations'] = {}
        
        builtin_locations = {
            'charging_station': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'point_A': {'x': 1.0, 'y': 0.0, 'theta': 0.0},
            'point_B': {'x': 2.0, 'y': 1.0, 'theta': 1.57},
            'point_C': {'x': 0.0, 'y': 2.0, 'theta': 3.14},
        }
        for name, loc in builtin_locations.items():
            if name not in self.blackboard['assets']['locations']:
                self.blackboard['assets']['locations'][name] = loc
        
        # 内置姿态
        if 'poses' not in self.blackboard['assets']:
            self.blackboard['assets']['poses'] = {}
        
        builtin_poses = {
            'home_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'observe_pose': [0.0, -0.5, 0.0, 1.2, 0.0, 0.5, 0.0],
            'handover_pose': [0.0, 0.3, 0.0, 1.0, 0.0, 0.7, 0.0],
        }
        for name, pose in builtin_poses.items():
            if name not in self.blackboard['assets']['poses']:
                self.blackboard['assets']['poses'][name] = pose
    
    def _parse_node(self, node_data: dict) -> Union[SkillNode, CompositeNode]:
        """
        递归解析单个节点
        
        Args:
            node_data: 节点数据字典
        
        Returns:
            解析后的节点对象
        """
        node_type = node_data.get('type')
        if not node_type:
            raise ValueError("Missing 'type' in node")
        
        node_id = node_data.get('id', str(uuid.uuid4())[:8])
        node_name = node_data.get('name', node_type)
        params = node_data.get('params', {})
        
        # 处理复合节点
        if node_type in self.COMPOSITE_TYPES:
            return self._parse_composite_node(node_data, node_type, node_id, node_name)
        
        # 处理技能节点
        if node_type in SKILL_REGISTRY:
            skill_class = SKILL_REGISTRY[node_type]
            return skill_class(
                node_id=node_id,
                params=params,
                blackboard=self.blackboard,
                ros_node=self.ros_node,
                node_name=node_name
            )
        
        raise ValueError(f"Unknown node type: {node_type}")
    
    def _parse_composite_node(
        self,
        node_data: dict,
        node_type: str,
        node_id: str,
        node_name: str
    ) -> CompositeNode:
        """解析复合节点"""
        composite_class = self.COMPOSITE_TYPES[node_type]
        
        # 额外参数（如 Parallel 的 success_threshold）
        extra_params = {}
        if node_type == 'Parallel':
            if 'success_threshold' in node_data.get('params', {}):
                extra_params['success_threshold'] = node_data['params']['success_threshold']
            if 'failure_threshold' in node_data.get('params', {}):
                extra_params['failure_threshold'] = node_data['params']['failure_threshold']
        
        composite_node = composite_class(
            node_id=node_id,
            node_name=node_name,
            blackboard=self.blackboard,
            **extra_params
        )
        
        # 解析子节点
        children_data = node_data.get('children', [])
        for child_data in children_data:
            child_node = self._parse_node(child_data)
            composite_node.add_child(child_node)
        
        return composite_node
    
    def validate(self, task_json: Union[str, dict]) -> tuple:
        """
        验证任务 JSON 的有效性
        
        Args:
            task_json: JSON 字符串或字典
        
        Returns:
            (is_valid, error_message)
        """
        try:
            if isinstance(task_json, str):
                task_data = json.loads(task_json)
            else:
                task_data = task_json
            
            # 检查必需字段
            if 'root' not in task_data:
                return False, "Missing 'root' field"
            
            # 递归验证节点
            errors = self._validate_node(task_data['root'], path='root')
            if errors:
                return False, '; '.join(errors)
            
            return True, ""
        
        except json.JSONDecodeError as e:
            return False, f"Invalid JSON: {e}"
        except Exception as e:
            return False, f"Validation error: {e}"
    
    def _validate_node(self, node_data: dict, path: str) -> list:
        """递归验证节点"""
        errors = []
        
        if not isinstance(node_data, dict):
            errors.append(f"{path}: node must be an object")
            return errors
        
        node_type = node_data.get('type')
        if not node_type:
            errors.append(f"{path}: missing 'type'")
            return errors
        
        # 检查节点类型是否已知
        if node_type not in self.COMPOSITE_TYPES and node_type not in SKILL_REGISTRY:
            errors.append(f"{path}: unknown node type '{node_type}'")
        
        # 验证复合节点的子节点
        if node_type in self.COMPOSITE_TYPES:
            children = node_data.get('children', [])
            if not isinstance(children, list):
                errors.append(f"{path}: 'children' must be an array")
            else:
                for i, child in enumerate(children):
                    child_errors = self._validate_node(child, f"{path}.children[{i}]")
                    errors.extend(child_errors)
        
        # 验证技能节点的参数
        if node_type in SKILL_REGISTRY:
            skill_class = SKILL_REGISTRY[node_type]
            params = node_data.get('params', {})
            for param_name, param_def in skill_class.PARAM_SCHEMA.items():
                if param_def.get('required', False) and param_name not in params:
                    errors.append(f"{path}: missing required param '{param_name}'")
        
        return errors
    
    def get_available_skills(self) -> list:
        """
        获取所有可用的技能节点类型
        
        Returns:
            技能节点信息列表
        """
        skills = []
        
        for name, skill_class in SKILL_REGISTRY.items():
            skills.append({
                'type': name,
                'description': skill_class.__doc__ or '',
                'params': skill_class.PARAM_SCHEMA
            })
        
        return skills
