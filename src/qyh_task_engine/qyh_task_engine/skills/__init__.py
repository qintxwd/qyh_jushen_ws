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
]

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
