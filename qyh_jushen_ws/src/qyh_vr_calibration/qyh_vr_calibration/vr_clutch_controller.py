#!/usr/bin/env python3
"""
VR Clutch Controller - 实现离合器模式的VR遥操作

工作原理:
1. 当grip按钮按下时(>threshold), 记录VR当前位姿和机器人目标位姿
2. 每帧计算VR相对于上一帧的增量，累加到机器人目标
3. 当grip按钮松开时(<threshold), 保持机器人最后位置

架构 (v2.0):
- VR→ROS坐标变换在 vr_bridge_node (C++) 中完成
- 坐标变换: ros_x=-vr_z, ros_y=-vr_x, ros_z=vr_y
- 握持补偿: 35° pitch (可调)
- 此控制器接收的VR数据已经是ROS坐标系 (X前Y左Z上)
- 位置增量在 base_link(世界)坐标系下
- 旋转增量在末端(局部)坐标系下
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
    
    max_position_delta: float = 0.02   # 单帧最大位移 (m)
    max_rotation_delta: float = 0.05   # 单帧最大旋转 (rad)
    
    # 死区：过滤手部抖动（小于此值的增量被忽略）
    position_deadzone: float = 0.002   # 位置死区 2mm
    rotation_deadzone: float = 0.01    # 旋转死区 ~0.5度
    
    # 低通滤波系数 (0-1)，越小滤波越强
    smoothing_factor: float = 0.5


class VRClutchController:
    """
    单臂的Clutch控制器 - 以base_link坐标系为基准的增量跟踪
    
    输入: VR数据已经是ROS坐标系 (通过 vr_bridge_node 变换)
    位置增量: 在 base_link(世界)坐标系下
    旋转增量: 在末端(局部)坐标系下
    """
    
    def __init__(self, config: Optional[ClutchConfig] = None, name: str = ""):
        self.config = config or ClutchConfig()
        self.name = name
        self.state = ClutchState.IDLE
        
        # 上一帧VR位姿（ROS坐标系）
        self.prev_vr_pos: Optional[np.ndarray] = None
        self.prev_vr_ori: Optional[Rotation] = None
        
        # 当前机器人目标（在base_link坐标系下）
        self.robot_target_pos: Optional[np.ndarray] = None
        self.robot_target_ori: Optional[np.ndarray] = None
        
        # 累积缓冲区（用于死区判断）
        self.accumulated_pos_delta: np.ndarray = np.zeros(3)
        self.accumulated_rot_delta: np.ndarray = np.zeros(3)

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
        vr_pos_ros = vr_pos
        vr_rot_ros = Rotation.from_quat(vr_ori)

        clutch_pressed = grip_value > self.config.engage_threshold
        clutch_released = grip_value < self.config.release_threshold

        # 状态机
        if self.state == ClutchState.IDLE:
            if clutch_pressed:
                self._engage_clutch(
                    vr_pos_ros, vr_rot_ros,
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

            # 在base_link坐标系下计算增量
            self._accumulate_delta(vr_pos_ros, vr_rot_ros)

            self.prev_vr_pos = vr_pos_ros.copy()
            self.prev_vr_ori = vr_rot_ros

            return (self.robot_target_pos.copy(),
                    self.robot_target_ori.copy(), self.state)

        elif self.state == ClutchState.RELEASING:
            if clutch_pressed:
                self._engage_clutch(
                    vr_pos_ros, vr_rot_ros,
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
        """初始化Clutch跟踪"""
        self.prev_vr_pos = vr_pos.copy()
        self.prev_vr_ori = vr_rot
        self.robot_target_pos = robot_pos.copy()
        self.robot_target_ori = robot_ori.copy()
        self.accumulated_pos_delta = np.zeros(3)
        self.accumulated_rot_delta = np.zeros(3)

    def _accumulate_delta(
        self,
        vr_pos: np.ndarray,
        vr_rot: Rotation
    ):
        """
        计算VR帧间增量

        位置增量：在 base_link 坐标系下（世界坐标）
        - 用户向前推手 → 机器人向 X+ 移动，与末端姿态无关

        姿态增量：相对于末端当前姿态（局部坐标）
        - 用户旋转手腕 → 末端跟着旋转，符合操作直觉
        """
        if self.prev_vr_pos is None:
            return

        alpha = self.config.smoothing_factor

        # === 位置增量（在 base_link 坐标系下）===
        # 位置以世界坐标为基准，与末端姿态无关
        delta_pos = vr_pos - self.prev_vr_pos
        robot_delta_pos = delta_pos * self.config.position_scale * alpha

        self.accumulated_pos_delta += robot_delta_pos

        accum_norm = np.linalg.norm(self.accumulated_pos_delta)
        if accum_norm >= self.config.position_deadzone:
            if accum_norm > self.config.max_position_delta:
                apply_delta = (self.accumulated_pos_delta / accum_norm
                               * self.config.max_position_delta)
                self.accumulated_pos_delta -= apply_delta
            else:
                apply_delta = self.accumulated_pos_delta
                self.accumulated_pos_delta = np.zeros(3)

            # 直接在 base_link 坐标系下相加
            self.robot_target_pos += apply_delta

        # === 姿态增量（相对于末端坐标系）===
        # VR 手柄的旋转增量（已在 ROS 坐标系下）
        delta_rot = vr_rot * self.prev_vr_ori.inv()
        rotvec = delta_rot.as_rotvec()
        rotvec *= self.config.rotation_scale * alpha

        self.accumulated_rot_delta += rotvec

        accum_angle = np.linalg.norm(self.accumulated_rot_delta)
        if accum_angle >= self.config.rotation_deadzone:
            if accum_angle > self.config.max_rotation_delta:
                apply_rotvec = (self.accumulated_rot_delta / accum_angle
                                * self.config.max_rotation_delta)
                self.accumulated_rot_delta -= apply_rotvec
            else:
                apply_rotvec = self.accumulated_rot_delta
                self.accumulated_rot_delta = np.zeros(3)

            # *** 关键：在末端坐标系下应用旋转 ***
            # target_new = target_old * delta_rot
            # 这样旋转是相对于末端当前姿态的
            apply_rot = Rotation.from_rotvec(apply_rotvec)
            target_rot = Rotation.from_quat(self.robot_target_ori)
            # 右乘：在末端局部坐标系下旋转
            new_target_rot = target_rot * apply_rot
            self.robot_target_ori = new_target_rot.as_quat()

    def reset(self):
        """重置控制器状态"""
        self.state = ClutchState.IDLE
        self.prev_vr_pos = None
        self.prev_vr_ori = None
        self.robot_target_pos = None
        self.robot_target_ori = None
        self.accumulated_pos_delta = np.zeros(3)
        self.accumulated_rot_delta = np.zeros(3)

    def is_engaged(self) -> bool:
        """检查clutch是否接合"""
        return self.state in (ClutchState.ENGAGING, ClutchState.TRACKING)

    def get_state_name(self) -> str:
        """获取状态名称"""
        return self.state.name