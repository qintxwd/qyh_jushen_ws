"""
QYH ACT Inference Package

ACT (Action Chunking Transformer) 推理部署包
用于具身智能机器人的实时模型推理和控制

模块结构:
- act_policy: ACT 模型加载和推理
- inference_node: ROS2 推理节点
- action_executor: 动作执行器（smoothing, clamp, 安全限制）
- utils: 工具函数（normalization, preprocessing）
"""

from .act_policy import ACTPolicy
from .action_executor import ActionExecutor
from .inference_config import InferenceConfig

__all__ = [
    'ACTPolicy',
    'ActionExecutor', 
    'InferenceConfig',
]

__version__ = '1.0.0'
