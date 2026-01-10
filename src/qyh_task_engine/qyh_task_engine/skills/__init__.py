"""
原子技能节点模块 (Skill Nodes)

包含所有可用的原子技能:
- 机械臂技能: ArmMoveJ, ArmMoveL, ArmStop
- 夹爪技能: GripperControl
- 头部技能: HeadLookAt, HeadScan
- 底盘技能: BaseMoveTo, BaseVelocity
- 升降技能: LiftMoveTo, LiftStop
- 逻辑技能: Wait, CheckCondition
"""

from .arm_skills import ArmMoveJNode, ArmMoveLNode, ArmStopNode
from .gripper_skills import GripperControlNode
from .head_skills import HeadLookAtNode, HeadScanNode
from .base_skills import BaseMoveToNode, BaseVelocityNode
from .lift_skills import LiftMoveToNode, LiftStopNode
from .waist_skills import WaistMoveToNode, WaistStopNode, WaistUprightNode
from .logic_skills import WaitNode, CheckConditionNode, SubTaskNode

__all__ = [
    # 机械臂
    'ArmMoveJNode',
    'ArmMoveLNode',
    'ArmStopNode',
    # 夹爪
    'GripperControlNode',
    # 头部
    'HeadLookAtNode',
    'HeadScanNode',
    # 底盘
    'BaseMoveToNode',
    'BaseVelocityNode',
    # 升降
    'LiftMoveToNode',
    'LiftStopNode',
    # 腰部
    'WaistMoveToNode',
    'WaistStopNode',
    'WaistUprightNode',
    # 逻辑
    'WaitNode',
    'CheckConditionNode',
    'SubTaskNode',
    # ACT 推理
    'ACTExecuteNode',
    'ACTLoadModelNode',
]

# 尝试导入 ACT 技能节点
try:
    from qyh_act_inference.act_skill_node import ACTExecuteNode, ACTLoadModelNode
    _HAS_ACT = True
except ImportError:
    # ACT 包未安装，使用占位
    ACTExecuteNode = None
    ACTLoadModelNode = None
    _HAS_ACT = False

# 节点类型注册表（用于 JSON 解析）
SKILL_REGISTRY = {
    'ArmMoveJ': ArmMoveJNode,
    'ArmMoveL': ArmMoveLNode,
    'ArmStop': ArmStopNode,
    'GripperControl': GripperControlNode,
    'HeadLookAt': HeadLookAtNode,
    'HeadScan': HeadScanNode,
    'BaseMoveTo': BaseMoveToNode,
    'BaseVelocity': BaseVelocityNode,
    'LiftMoveTo': LiftMoveToNode,
    'LiftStop': LiftStopNode,
    'WaistMoveTo': WaistMoveToNode,
    'WaistStop': WaistStopNode,
    'WaistUpright': WaistUprightNode,
    'Wait': WaitNode,
    'CheckCondition': CheckConditionNode,
    'SubTask': SubTaskNode,
}

# 如果 ACT 包可用，添加 ACT 技能
if _HAS_ACT:
    SKILL_REGISTRY['ACTExecute'] = ACTExecuteNode
    SKILL_REGISTRY['ACTLoadModel'] = ACTLoadModelNode

