"""
机械臂技能节点 (Arm Skills)

包含:
- ArmMoveJNode: 关节空间运动
- ArmMoveLNode: 笛卡尔空间直线运动
- ArmStopNode: 急停
"""

import time
from typing import Dict, Any, List, Optional, Tuple

from ..base_node import SkillNode, SkillStatus, SkillResult
from ..preset_loader import preset_loader


class ArmMoveJNode(SkillNode):
    """
    关节空间运动节点 (MoveJ)
    
    参数:
        side: 机械臂选择 ("left", "right", "both")
        joint_positions: 目标关节角度 (弧度)，7个关节
        pose_name: 预设姿态名称（可选，与 joint_positions 二选一）
        velocity: 关节速度 (rad/s)，默认 0.5
        acceleration: 关节加速度 (rad/s²)，默认 0.3
        is_block: 是否阻塞等待完成，默认 True
    """
    
    NODE_TYPE = "ArmMoveJ"
    
    PARAM_SCHEMA = {
        'side': {'type': 'string', 'required': True, 'enum': ['left', 'right', 'both']},
        'joint_positions': {'type': 'array', 'required': False},
        'pose_name': {'type': 'string', 'required': False},
        'velocity': {'type': 'float', 'default': 0.5},
        'acceleration': {'type': 'float', 'default': 0.3},
        'is_block': {'type': 'bool', 'default': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._move_j_client = None
        self._future = None
        self._is_executing = False
    
    def setup(self) -> bool:
        """初始化 MoveJ 服务客户端"""
        self.log_info("="*40)
        self.log_info(f"[ArmMoveJ] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        if not self.ros_node:
            self.log_warn("  No ROS node, running in MOCK mode")
            return True
        
        try:
            from qyh_jaka_control_msgs.srv import MoveJ
            self._move_j_client = self.ros_node.create_client(
                MoveJ, '/jaka/move_j'
            )
            self.log_info("  MoveJ client created")
            self.log_info("="*40)
            return True
        except Exception as e:
            self.log_error(f"  Failed to create MoveJ client: {e}")
            return False
    
    def execute(self) -> SkillResult:
        """执行 MoveJ"""
        side = self.params.get('side', 'left')
        velocity = self.params.get('velocity', 0.5)
        acceleration = self.params.get('acceleration', 0.3)
        is_block = self.params.get('is_block', True)
        
        # 解析关节位置
        joint_positions = self._resolve_joint_positions()
        if joint_positions is None:
            self.log_error(f"Cannot resolve joint positions")
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Invalid joint positions or pose_name"
            )
        
        # 确定 robot_id
        robot_id = self._get_robot_id(side)
        
        # 显示前3个关节角度
        joints_preview = [f"{j:.2f}" for j in joint_positions[:3]]
        self.log_info(f"[MoveJ] {side} arm -> [{', '.join(joints_preview)}...] v={velocity}")
        
        # Mock 模式
        if not self._move_j_client:
            time.sleep(0.5)
            self.log_info(f"[MoveJ] Completed (mock)")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"MoveJ completed (mock mode)",
                data={'final_positions': joint_positions}
            )
        
        # 真实执行
        if not self._is_executing:
            from qyh_jaka_control_msgs.srv import MoveJ
            
            if not self._move_j_client.wait_for_service(timeout_sec=1.0):
                self.log_error("MoveJ service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="MoveJ service not available"
                )
            
            request = MoveJ.Request()
            request.robot_id = robot_id
            request.joint_positions = joint_positions
            request.velocity = velocity
            request.acceleration = acceleration
            request.is_block = is_block
            
            self.log_info(f"   Sending MoveJ request (robot_id={robot_id})")
            self._future = self._move_j_client.call_async(request)
            self._is_executing = True
            return SkillResult(status=SkillStatus.RUNNING, message="Executing MoveJ...")
        
        # 检查是否完成
        if self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    self.log_info(f"[MoveJ] Completed: {response.message}")
                    return SkillResult(
                        status=SkillStatus.SUCCESS,
                        message=response.message
                    )
                else:
                    self.log_error(f"[MoveJ] Failed: {response.message}")
                    return SkillResult(
                        status=SkillStatus.FAILURE,
                        message=response.message
                    )
            except Exception as e:
                self.log_error(f"[MoveJ] Exception: {e}")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=f"MoveJ failed: {e}"
                )
        
        return SkillResult(status=SkillStatus.RUNNING, message="Executing MoveJ...")
    
    def _resolve_joint_positions(self) -> Optional[List[float]]:
        """
        解析关节位置（从参数或预设资产）
        
        支持的参数形式:
        1. joint_positions: 直接指定关节角度（7个或14个）
        2. pose_name: 预设姿态名称（从持久化存储读取）
        3. left_joints / right_joints: 分别指定左右臂
        
        返回:
        - side='left' 或 'right': 返回 7 个关节
        - side='both': 返回 14 个关节 [left_7, right_7]
        """
        side = self.params.get('side', 'left')
        
        # 直接指定关节位置
        if 'joint_positions' in self.params:
            positions = list(self.params['joint_positions'])
            # 如果是 both 模式且只给了 7 个关节，需要报错
            if side == 'both' and len(positions) == 7:
                self.log_error("side='both' requires 14 joint positions, got 7")
                return None
            return positions
        
        # 从预设姿态读取
        if 'pose_name' in self.params:
            pose_name = self.params['pose_name']
            
            # 从持久化预设加载
            pose_preset = preset_loader.get_arm_pose(pose_name)
            if pose_preset:
                left_joints = pose_preset.get('left_joints')
                right_joints = pose_preset.get('right_joints')
                
                if side == 'left':
                    if left_joints:
                        return list(left_joints)
                    self.log_warn(f"Pose '{pose_name}' has no left_joints")
                elif side == 'right':
                    if right_joints:
                        return list(right_joints)
                    self.log_warn(f"Pose '{pose_name}' has no right_joints")
                elif side == 'both':
                    # both 模式需要拼接 14 个关节
                    if left_joints and right_joints:
                        return list(left_joints) + list(right_joints)
                    self.log_warn(f"Pose '{pose_name}' missing joints for both arms")
                    return None
            
            # 从黑板读取预设姿态（兼容旧格式）
            poses = self.read_from_blackboard('assets.poses', {})
            if pose_name in poses:
                return list(poses[pose_name])
            
            self.log_error(f"Unknown pose_name: {pose_name}")
            return None
        
        # 直接指定 left_joints 或 right_joints
        if side == 'left' and 'left_joints' in self.params:
            return list(self.params['left_joints'])
        if side == 'right' and 'right_joints' in self.params:
            return list(self.params['right_joints'])
        if side == 'both':
            left = self.params.get('left_joints')
            right = self.params.get('right_joints')
            if left and right:
                return list(left) + list(right)
            self.log_error("side='both' requires both left_joints and right_joints")
            return None
        
        self.log_error("Missing joint_positions or pose_name")
        return None
    
    def _get_robot_id(self, side: str) -> int:
        """获取机器人 ID"""
        if side == 'left':
            return 0
        elif side == 'right':
            return 1
        else:  # both
            return -1
    
    def halt(self):
        """中断执行"""
        super().halt()
        # TODO: 发送急停命令
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._future = None
        self._is_executing = False


class ArmMoveLNode(SkillNode):
    """
    笛卡尔空间直线运动节点 (MoveL)
    
    参数:
        side: 机械臂选择 ("left", "right")
        x, y, z: 目标位置 (米)
        rx, ry, rz: 目标姿态 (弧度)
        velocity: 速度 (mm/s)，默认 100
        acceleration: 加速度 (mm/s²)，默认 50
        is_block: 是否阻塞等待完成，默认 True
    """
    
    NODE_TYPE = "ArmMoveL"
    
    PARAM_SCHEMA = {
        'side': {'type': 'string', 'required': True, 'enum': ['left', 'right']},
        'x': {'type': 'float', 'required': True},
        'y': {'type': 'float', 'required': True},
        'z': {'type': 'float', 'required': True},
        'rx': {'type': 'float', 'default': 0.0},
        'ry': {'type': 'float', 'default': 0.0},
        'rz': {'type': 'float', 'default': 0.0},
        'velocity': {'type': 'float', 'default': 100.0},
        'acceleration': {'type': 'float', 'default': 50.0},
        'is_block': {'type': 'bool', 'default': True},
    }
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._move_l_client = None
        self._future = None
        self._is_executing = False
    
    def setup(self) -> bool:
        """初始化 MoveL 服务客户端"""
        if not self.ros_node:
            self.log_warn("No ROS node available, running in mock mode")
            return True
        
        try:
            from qyh_jaka_control_msgs.srv import MoveL
            self._move_l_client = self.ros_node.create_client(
                MoveL, '/jaka/move_l'
            )
            return True
        except Exception as e:
            self.log_error(f"Failed to create MoveL client: {e}")
            return False
    
    def execute(self) -> SkillResult:
        """执行 MoveL"""
        side = self.params.get('side', 'left')
        x = self.params.get('x', 0.0)
        y = self.params.get('y', 0.0)
        z = self.params.get('z', 0.0)
        rx = self.params.get('rx', 0.0)
        ry = self.params.get('ry', 0.0)
        rz = self.params.get('rz', 0.0)
        velocity = self.params.get('velocity', 100.0)
        acceleration = self.params.get('acceleration', 50.0)
        is_block = self.params.get('is_block', True)
        
        robot_id = 0 if side == 'left' else 1
        
        self.log_info(f"Moving {side} arm to ({x:.3f}, {y:.3f}, {z:.3f})")
        
        # Mock 模式
        if not self._move_l_client:
            time.sleep(0.5)
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="MoveL completed (mock mode)"
            )
        
        # 真实执行
        if not self._is_executing:
            from qyh_jaka_control_msgs.srv import MoveL
            from geometry_msgs.msg import Pose
            
            if not self._move_l_client.wait_for_service(timeout_sec=1.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="MoveL service not available"
                )
            
            request = MoveL.Request()
            request.robot_id = robot_id
            request.target_pose = Pose()
            request.target_pose.position.x = x
            request.target_pose.position.y = y
            request.target_pose.position.z = z
            request.target_pose.orientation.x = rx
            request.target_pose.orientation.y = ry
            request.target_pose.orientation.z = rz
            request.target_pose.orientation.w = 1.0
            request.velocity = velocity
            request.acceleration = acceleration
            request.is_block = is_block
            
            self._future = self._move_l_client.call_async(request)
            self._is_executing = True
            return SkillResult(status=SkillStatus.RUNNING, message="Executing MoveL...")
        
        if self._future.done():
            try:
                response = self._future.result()
                if response.success:
                    return SkillResult(status=SkillStatus.SUCCESS, message=response.message)
                else:
                    return SkillResult(status=SkillStatus.FAILURE, message=response.message)
            except Exception as e:
                return SkillResult(status=SkillStatus.FAILURE, message=f"MoveL failed: {e}")
        
        return SkillResult(status=SkillStatus.RUNNING, message="Executing MoveL...")
    
    def reset(self):
        """重置节点状态"""
        super().reset()
        self._future = None
        self._is_executing = False


class ArmStopNode(SkillNode):
    """
    机械臂急停节点
    
    参数:
        side: 机械臂选择 ("left", "right", "both")，默认 "both"
    """
    
    NODE_TYPE = "ArmStop"
    
    PARAM_SCHEMA = {
        'side': {'type': 'string', 'default': 'both', 'enum': ['left', 'right', 'both']},
    }
    
    def setup(self) -> bool:
        return True
    
    def execute(self) -> SkillResult:
        side = self.params.get('side', 'both')
        self.log_info(f"Stopping {side} arm")
        
        if not self.ros_node:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="ArmStop executed (mock mode)"
            )
        
        try:
            from std_srvs.srv import Trigger
            client = self.ros_node.create_client(Trigger, '/jaka/robot/motion_abort')
            
            if not client.wait_for_service(timeout_sec=1.0):
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="Motion abort service not available"
                )
            
            future = client.call_async(Trigger.Request())
            # 急停需要立即返回，不等待响应
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Stop command sent"
            )
        except Exception as e:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=f"Failed to stop: {e}"
            )
