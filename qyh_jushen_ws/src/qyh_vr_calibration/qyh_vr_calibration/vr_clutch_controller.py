#!/usr/bin/env python3
"""
VR Clutch Controller - 实现离合器模式的VR遥操作

工作原理 (v3.0 - 基于原点偏移):
1. 当grip按钮按下时(>threshold), 记录VR原点位姿和机器人原点位姿
2. 每帧计算VR相对于原点的偏移（不是帧间增量），直接映射到机器人目标
3. 当grip按钮松开时(<threshold), 保持机器人最后位置

优点:
- 无累积误差：始终基于原点计算偏移
- 更稳定：抖动不会累积
- 直观：VR手柄从原点移动多少，机器人就移动多少(乘以缩放)

架构:
- VR→ROS坐标变换在 vr_bridge_node (C++) 中完成
- 此控制器接收的VR数据已经是ROS坐标系 (X前Y左Z上)
- 位置偏移在 base_link(世界)坐标系下
- 旋转偏移相对于grip按下时的姿态
"""

import numpy as np
from enum import Enum
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from typing import Optional, Tuple


class ClutchState(Enum):
    """Clutch状态"""
    IDLE = 0        # Clutch未接合，机器人保持位置
    ENGAGING = 1    # Clutch刚接合，初始化
    TRACKING = 2    # Clutch接合中，跟踪VR增量
    RELEASING = 3   # Clutch刚释放


@dataclass
class ClutchConfig:
    """Clutch配置参数"""
    engage_threshold: float = 0.8      # grip > 0.8 时接合clutch
    release_threshold: float = 0.2     # grip < 0.2 时释放clutch
    
    # 位置缩放: 机械臂长度(~1.2m) / 手臂长度(~0.6m) ≈ 2.0
    position_scale: float = 2.0        # VR位移到机器人位移的缩放
    rotation_scale: float = 1.0        # VR旋转到机器人旋转的缩放
    
    max_position_delta: float = 0.05   # 单帧最大位移变化 (m) - 用于平滑
    max_rotation_delta: float = 0.1    # 单帧最大旋转变化 (rad) - 用于平滑
    
    # 低通滤波系数 (0-1)，越小滤波越强
    smoothing_factor: float = 0.3


class VRClutchController:
    """
    单臂的Clutch控制器 - 基于原点偏移的VR遥操作
    
    原理:
    - 按下grip时，记录VR原点位姿和机器人原点位姿
    - 跟踪时，计算VR当前位姿相对于VR原点的偏移
    - 将偏移（乘以缩放）加到机器人原点，得到机器人目标位姿
    
    输入: VR数据已经是ROS坐标系 (通过 vr_bridge_node 变换)
    """
    
    def __init__(self, config: Optional[ClutchConfig] = None, name: str = ""):
        self.config = config or ClutchConfig()
        self.name = name
        self.state = ClutchState.IDLE
        
        # Grip按下时的VR原点位姿（ROS坐标系）
        self.vr_origin_pos: Optional[np.ndarray] = None
        self.vr_origin_ori: Optional[Rotation] = None
        
        # Grip按下时的机器人原点位姿（base_link坐标系）
        self.robot_origin_pos: Optional[np.ndarray] = None
        self.robot_origin_ori: Optional[Rotation] = None
        
        # 当前机器人目标（用于平滑输出）
        self.robot_target_pos: Optional[np.ndarray] = None
        self.robot_target_ori: Optional[np.ndarray] = None

    def update(
        self,
        vr_pos: np.ndarray,
        vr_ori: np.ndarray,
        robot_current_pos: np.ndarray,
        robot_current_ori: np.ndarray,
        grip_value: float
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], ClutchState]:
        """
        更新Clutch控制器
        
        Args:
            vr_pos: VR手柄位置 [x,y,z] - ROS坐标系 (已变换)
            vr_ori: VR手柄姿态 [x,y,z,w] - ROS坐标系 (已变换)
            robot_current_pos: 机器人末端位置 - base_link坐标系
            robot_current_ori: 机器人末端姿态 - base_link坐标系
            grip_value: Grip按钮值 [0.0, 1.0]
            
        Returns:
            (target_pos, target_ori, state): 目标位姿(base_link坐标系)和状态
        """
        vr_pos = np.asarray(vr_pos, dtype=np.float64)
        vr_ori = np.asarray(vr_ori, dtype=np.float64)
        robot_current_pos = np.asarray(robot_current_pos, dtype=np.float64)
        robot_current_ori = np.asarray(robot_current_ori, dtype=np.float64)

        # 输入数据已经是 ROS 坐标系 (由 vr_bridge_node 变换)
        vr_rot = Rotation.from_quat(vr_ori)

        clutch_pressed = grip_value > self.config.engage_threshold
        clutch_released = grip_value < self.config.release_threshold

        # 状态机
        if self.state == ClutchState.IDLE:
            if clutch_pressed:
                self._engage_clutch(
                    vr_pos, vr_rot,
                    robot_current_pos, robot_current_ori
                )
                self.state = ClutchState.ENGAGING
                return (self.robot_target_pos.copy(),
                        self.robot_target_ori.copy(), self.state)
            return None, None, self.state

        elif self.state == ClutchState.ENGAGING:
            self.state = ClutchState.TRACKING

        if self.state == ClutchState.TRACKING:
            if clutch_released:
                self.state = ClutchState.RELEASING
                if self.robot_target_pos is not None:
                    return (self.robot_target_pos.copy(),
                            self.robot_target_ori.copy(), self.state)
                return None, None, self.state

            # 基于原点偏移计算目标位姿
            self._compute_target_from_offset(vr_pos, vr_rot)

            return (self.robot_target_pos.copy(),
                    self.robot_target_ori.copy(), self.state)

        elif self.state == ClutchState.RELEASING:
            if clutch_pressed:
                self._engage_clutch(
                    vr_pos, vr_rot,
                    robot_current_pos, robot_current_ori
                )
                self.state = ClutchState.TRACKING
                return (self.robot_target_pos.copy(),
                        self.robot_target_ori.copy(), self.state)
            self.state = ClutchState.IDLE
            return None, None, self.state

        return None, None, self.state

    def _engage_clutch(
        self,
        vr_pos: np.ndarray,
        vr_rot: Rotation,
        robot_pos: np.ndarray,
        robot_ori: np.ndarray
    ):
        """
        初始化Clutch跟踪 - 记录VR和机器人的原点位姿
        
        按下grip的瞬间:
        - 记录VR原点位姿
        - 记录机器人原点位姿
        - 之后所有目标都基于这两个原点计算
        """
        # 记录VR原点
        self.vr_origin_pos = vr_pos.copy()
        self.vr_origin_ori = vr_rot
        
        # 记录机器人原点
        self.robot_origin_pos = robot_pos.copy()
        self.robot_origin_ori = Rotation.from_quat(robot_ori)
        
        # 初始化目标为当前机器人位置
        self.robot_target_pos = robot_pos.copy()
        self.robot_target_ori = robot_ori.copy()
        
        # 调试输出
        print(f"[{self.name}] Clutch ENGAGE (origin-based):")
        print(f"  VR origin: [{vr_pos[0]:.3f}, {vr_pos[1]:.3f}, {vr_pos[2]:.3f}]")
        print(f"  Robot origin: [{robot_pos[0]:.3f}, {robot_pos[1]:.3f}, "
              f"{robot_pos[2]:.3f}]")

    def _compute_target_from_offset(
        self,
        vr_pos: np.ndarray,
        vr_rot: Rotation
    ):
        """
        基于原点偏移计算机器人目标位姿
        
        公式:
        - 位置: robot_target = robot_origin + (vr_current - vr_origin) * scale
        - 姿态: robot_target = robot_origin * (vr_origin.inv() * vr_current) * scale
        
        这样不会累积误差，VR回到原点位置时机器人也回到原点
        """
        alpha = self.config.smoothing_factor
        
        # === 位置：计算VR相对于原点的偏移 ===
        vr_offset_pos = vr_pos - self.vr_origin_pos
        
        # 缩放后加到机器人原点
        robot_offset_pos = vr_offset_pos * self.config.position_scale
        raw_target_pos = self.robot_origin_pos + robot_offset_pos
        
        # 平滑：限制单帧变化量
        if self.robot_target_pos is not None:
            delta_pos = raw_target_pos - self.robot_target_pos
            delta_norm = np.linalg.norm(delta_pos)
            if delta_norm > self.config.max_position_delta:
                delta_pos = delta_pos / delta_norm * self.config.max_position_delta
            self.robot_target_pos = self.robot_target_pos + delta_pos * alpha
        else:
            self.robot_target_pos = raw_target_pos
        
        # === 姿态：计算VR相对于原点的旋转偏移 ===
        # vr_offset_rot = vr_origin.inv() * vr_current
        # 表示从VR原点姿态到当前姿态的旋转
        vr_offset_rot = self.vr_origin_ori.inv() * vr_rot
        
        # 缩放旋转（将旋转向量缩放）
        offset_rotvec = vr_offset_rot.as_rotvec() * self.config.rotation_scale
        scaled_offset_rot = Rotation.from_rotvec(offset_rotvec)
        
        # 应用到机器人原点姿态（右乘：在末端局部坐标系下旋转）
        raw_target_rot = self.robot_origin_ori * scaled_offset_rot
        raw_target_ori = raw_target_rot.as_quat()
        
        # 平滑：限制单帧旋转变化
        if self.robot_target_ori is not None:
            current_rot = Rotation.from_quat(self.robot_target_ori)
            delta_rot = current_rot.inv() * raw_target_rot
            delta_angle = np.linalg.norm(delta_rot.as_rotvec())
            
            if delta_angle > self.config.max_rotation_delta:
                limited_rotvec = (delta_rot.as_rotvec() / delta_angle 
                                  * self.config.max_rotation_delta)
                delta_rot = Rotation.from_rotvec(limited_rotvec)
            
            # 应用平滑后的增量
            smoothed_rotvec = delta_rot.as_rotvec() * alpha
            smoothed_rot = Rotation.from_rotvec(smoothed_rotvec)
            new_rot = current_rot * smoothed_rot
            self.robot_target_ori = new_rot.as_quat()
        else:
            self.robot_target_ori = raw_target_ori

    def reset(self):
        """重置控制器状态"""
        self.state = ClutchState.IDLE
        self.vr_origin_pos = None
        self.vr_origin_ori = None
        self.robot_origin_pos = None
        self.robot_origin_ori = None
        self.robot_target_pos = None
        self.robot_target_ori = None

    def is_engaged(self) -> bool:
        """检查clutch是否接合"""
        return self.state in (ClutchState.ENGAGING, ClutchState.TRACKING)

    def get_state_name(self) -> str:
        """获取状态名称"""
        return self.state.name
