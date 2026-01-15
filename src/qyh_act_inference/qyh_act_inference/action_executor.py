"""
动作执行器

负责将模型输出转换为安全可执行的机器人命令:
1. 动作缩放 (action scale)
2. 动作平滑 (EMA smoothing)
3. 速度/位置限制 (clamp)
4. 夹爪二值化 (thresholding)
5. 双臂动作拆分
"""

import numpy as np
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass
from collections import deque


@dataclass
class ArmCommand:
    """机械臂命令"""
    joint_positions: np.ndarray     # (7,) 目标关节角度
    joint_velocities: Optional[np.ndarray] = None  # (7,) 关节速度（可选）
    is_delta: bool = True           # 是否为增量


@dataclass
class GripperCommand:
    """夹爪命令"""
    position: float                 # 目标位置 [0, 1]
    is_binary: bool = True          # 是否二值化（开/关）


@dataclass
class RobotCommand:
    """完整的机器人命令"""
    left_arm: Optional[ArmCommand] = None
    right_arm: Optional[ArmCommand] = None
    left_gripper: Optional[GripperCommand] = None
    right_gripper: Optional[GripperCommand] = None
    timestamp: float = 0.0


class ActionExecutor:
    """
    动作执行器
    
    处理推理输出，生成安全的机器人命令
    """
    
    def __init__(self, config):
        """
        初始化执行器
        
        Args:
            config: InferenceConfig 配置对象
        """
        self.config = config
        
        # 上一次的动作（用于平滑）
        self._last_action: Optional[np.ndarray] = None
        
        # 当前关节状态（用于安全检查）
        self._current_left_joints: Optional[np.ndarray] = None
        self._current_right_joints: Optional[np.ndarray] = None
        
        # 动作队列（从 action_chunk 中取）
        self._action_queue: deque = deque()
        
        # 夹爪状态（用于滞后处理）
        self._left_gripper_state: float = 0.0
        self._right_gripper_state: float = 0.0
        
        # 统计
        self._total_actions = 0
        self._clamped_actions = 0
    
    def update_state(
        self,
        left_joints: Optional[np.ndarray] = None,
        right_joints: Optional[np.ndarray] = None
    ):
        """
        更新当前关节状态
        
        Args:
            left_joints: 左臂当前关节角度
            right_joints: 右臂当前关节角度
        """
        if left_joints is not None:
            self._current_left_joints = np.array(left_joints)
        if right_joints is not None:
            self._current_right_joints = np.array(right_joints)
    
    def set_action_chunk(self, action_chunk: np.ndarray):
        """
        设置新的动作序列
        
        Args:
            action_chunk: (K, action_dim) 模型输出的动作序列
        """
        # 清空旧队列
        self._action_queue.clear()
        
        # 根据 action_steps 决定取多少步
        steps_to_take = min(self.config.action_steps, len(action_chunk))
        
        for i in range(steps_to_take):
            self._action_queue.append(action_chunk[i])
    
    def has_pending_actions(self) -> bool:
        """是否还有待执行的动作"""
        return len(self._action_queue) > 0
    
    def get_next_command(self) -> Optional[RobotCommand]:
        """
        获取下一个机器人命令
        
        Returns:
            RobotCommand 或 None（如果队列为空）
        """
        if not self._action_queue:
            return None
        
        raw_action = self._action_queue.popleft()
        
        # 1. 动作缩放
        scaled_action = self._scale_action(raw_action)
        
        # 2. 动作平滑
        smoothed_action = self._smooth_action(scaled_action)
        
        # 3. 拆分动作
        parsed = self._parse_action(smoothed_action)
        
        # 4. 应用安全限制
        command = self._apply_safety_limits(parsed)
        
        self._total_actions += 1
        
        return command
    
    def _scale_action(self, action: np.ndarray) -> np.ndarray:
        """
        缩放动作
        
        这是部署调参最关键的地方之一！
        """
        if hasattr(self.config, 'action_type') and self.config.action_type == 'absolute':
            # 对于绝对位置控制，直接缩放会导致位置错误 (e.g. 1.0 -> 0.4)
            # 因此禁用直接缩放
            if self.config.action_scale < 0.99:
                 pass  # 可以添加日志，但在高频循环中避免打印
            return action
            
        return action * self.config.action_scale
    
    def _smooth_action(self, action: np.ndarray) -> np.ndarray:
        """
        EMA 平滑
        
        a_t = α * a_pred + (1-α) * a_{t-1}
        """
        if self._last_action is None:
            self._last_action = action
            return action
        
        alpha = self.config.smoothing_alpha
        smoothed = alpha * action + (1 - alpha) * self._last_action
        self._last_action = smoothed
        
        return smoothed
    
    def _parse_action(self, action: np.ndarray) -> Dict:
        """
        解析动作向量，拆分为各部件
        
        动作向量布局（根据配置动态）:
        - [0:7]: 左臂（如果启用）
        - [7:14] 或 [0:7]: 右臂（如果启用）
        - 最后 1-2 维: 夹爪
        """
        parsed = {
            'left_arm': None,
            'right_arm': None,
            'left_gripper': None,
            'right_gripper': None,
        }
        
        idx = 0
        arm_dof = self.config.arm_dof
        
        # 左臂
        if self.config.use_left_arm:
            parsed['left_arm'] = action[idx:idx + arm_dof]
            idx += arm_dof
        
        # 右臂
        if self.config.use_right_arm:
            parsed['right_arm'] = action[idx:idx + arm_dof]
            idx += arm_dof
        
        # 左夹爪
        if self.config.use_left_gripper:
            parsed['left_gripper'] = action[idx]
            idx += 1
        
        # 右夹爪
        if self.config.use_right_gripper:
            parsed['right_gripper'] = action[idx]
            idx += 1
        
        return parsed
    
    def _apply_safety_limits(self, parsed: Dict) -> RobotCommand:
        """
        应用安全限制，生成最终命令
        """
        command = RobotCommand()
        is_absolute = (self.config.action_type == "absolute")
        
        # 处理左臂
        if parsed['left_arm'] is not None:
            if is_absolute:
                target_pos = parsed['left_arm']
                # 如果有当前状态，应用速度限制 (max_delta)
                if self._current_left_joints is not None:
                    diff = target_pos - self._current_left_joints
                    clamped_diff = np.clip(diff, -self.config.max_joint_delta, self.config.max_joint_delta)
                    safe_pos = self._current_left_joints + clamped_diff
                else:
                    safe_pos = target_pos
                
                command.left_arm = ArmCommand(
                    joint_positions=safe_pos,
                    is_delta=False
                )
            else:
                joint_delta = self._clamp_joint_delta(
                    parsed['left_arm'],
                    self.config.left_arm_joint_limits
                )
                command.left_arm = ArmCommand(
                    joint_positions=joint_delta,
                    is_delta=True
                )
        
        # 处理右臂
        if parsed['right_arm'] is not None:
            if is_absolute:
                target_pos = parsed['right_arm']
                # 如果有当前状态，应用速度限制 (max_delta)
                if self._current_right_joints is not None:
                    diff = target_pos - self._current_right_joints
                    clamped_diff = np.clip(diff, -self.config.max_joint_delta, self.config.max_joint_delta)
                    safe_pos = self._current_right_joints + clamped_diff
                else:
                    safe_pos = target_pos
                    
                command.right_arm = ArmCommand(
                    joint_positions=safe_pos,
                    is_delta=False
                )
            else:
                joint_delta = self._clamp_joint_delta(
                    parsed['right_arm'],
                    self.config.right_arm_joint_limits
                )
                command.right_arm = ArmCommand(
                    joint_positions=joint_delta,
                    is_delta=True
                )
        
        # 处理左夹爪
        if parsed['left_gripper'] is not None:
            gripper_cmd = self._process_gripper(
                parsed['left_gripper'],
                self._left_gripper_state,
                'left'
            )
            self._left_gripper_state = gripper_cmd.position
            command.left_gripper = gripper_cmd
        
        # 处理右夹爪
        if parsed['right_gripper'] is not None:
            gripper_cmd = self._process_gripper(
                parsed['right_gripper'],
                self._right_gripper_state,
                'right'
            )
            self._right_gripper_state = gripper_cmd.position
            command.right_gripper = gripper_cmd
        
        return command
    
    def _clamp_joint_delta(
        self,
        delta: np.ndarray,
        joint_limits: List[Tuple[float, float]]
    ) -> np.ndarray:
        """
        限制关节增量
        
        Args:
            delta: 关节增量
            joint_limits: 关节限位
        
        Returns:
            限制后的增量
        """
        max_delta = self.config.max_joint_delta
        
        # 逐关节限制增量
        clamped = np.clip(delta, -max_delta, max_delta)
        
        # 检查是否有被截断
        if not np.allclose(delta, clamped):
            self._clamped_actions += 1
            if self.config.verbose:
                print(f"[ActionExecutor] Action clamped: max_delta={np.max(np.abs(delta)):.4f}")
        
        return clamped
    
    def _process_gripper(
        self,
        raw_value: float,
        last_state: float,
        side: str
    ) -> GripperCommand:
        """
        处理夹爪命令（带滞后）
        
        Args:
            raw_value: 原始预测值 [0, 1]
            last_state: 上一次状态
            side: "left" 或 "right"
        
        Returns:
            GripperCommand
        """
        threshold = self.config.gripper_threshold
        hysteresis = self.config.gripper_hysteresis
        
        # 应用滞后（防止抖动）
        if last_state > 0.5:  # 当前是闭合
            # 需要更低才打开
            new_state = 1.0 if raw_value > (threshold - hysteresis) else 0.0
        else:  # 当前是打开
            # 需要更高才闭合
            new_state = 1.0 if raw_value > (threshold + hysteresis) else 0.0
        
        return GripperCommand(
            position=new_state,
            is_binary=True
        )
    
    def reset(self):
        """重置执行器状态"""
        self._last_action = None
        self._action_queue.clear()
        self._left_gripper_state = 0.0
        self._right_gripper_state = 0.0
        self._total_actions = 0
        self._clamped_actions = 0
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return {
            'total_actions': self._total_actions,
            'clamped_actions': self._clamped_actions,
            'clamp_rate': self._clamped_actions / max(1, self._total_actions),
        }


class ActionInterpolator:
    """
    动作插值器
    
    用于在控制频率高于推理频率时进行插值
    """
    
    def __init__(self, config):
        self.config = config
        self._start_action: Optional[np.ndarray] = None
        self._end_action: Optional[np.ndarray] = None
        self._progress: float = 0.0
        self._steps: int = 1
    
    def set_target(self, start: np.ndarray, end: np.ndarray, steps: int):
        """
        设置插值目标
        
        Args:
            start: 起始动作
            end: 目标动作
            steps: 插值步数
        """
        self._start_action = start
        self._end_action = end
        self._steps = max(1, steps)
        self._progress = 0.0
    
    def step(self) -> Optional[np.ndarray]:
        """
        获取下一个插值动作
        
        Returns:
            插值后的动作，或 None（如果已完成）
        """
        if self._start_action is None or self._end_action is None:
            return None
        
        if self._progress >= 1.0:
            return None
        
        # 线性插值
        t = self._progress
        action = (1 - t) * self._start_action + t * self._end_action
        
        self._progress += 1.0 / self._steps
        
        return action
    
    def is_done(self) -> bool:
        """是否插值完成"""
        return self._progress >= 1.0
