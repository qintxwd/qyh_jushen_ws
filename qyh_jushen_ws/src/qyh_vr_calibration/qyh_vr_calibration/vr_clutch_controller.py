#!/usr/bin/env python3
"""
VR Clutch Controller - 实现离合器模式的VR遥操作

工作原理:
1. 当grip按钮按下时(>threshold), 记录VR当前位姿和机器人目标位姿
2. 每帧计算VR相对于上一帧的增量，累加到机器人目标
3. 当grip按钮松开时(<threshold), 保持机器人最后位置

关键点:
- 逐帧累积增量，而不是相对参考点计算（避免限幅导致卡住）
- VR增量 = 机器人增量（无需复杂坐标变换）
- 只需要坐标轴映射（VR和机器人的XYZ对应关系）
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
    
    # 位置缩放: 机械臂长度(1.2m) / 手臂长度(0.52m) ≈ 2.3
    position_scale: float = 2.3        # VR位移到机器人位移的缩放
    rotation_scale: float = 1.0        # VR旋转到机器人旋转的缩放
    
    max_position_delta: float = 0.02   # 单帧最大位移 (m)，约50Hz时对应1m/s
    max_rotation_delta: float = 0.05   # 单帧最大旋转 (rad)，约50Hz时对应2.5rad/s
    
    # 死区：过滤手部抖动（小于此值的增量被忽略）
    position_deadzone: float = 0.002   # 位置死区 2mm
    rotation_deadzone: float = 0.01    # 旋转死区 ~0.5度
    
    # 低通滤波系数 (0-1)，越小滤波越强，0=不更新，1=不滤波
    smoothing_factor: float = 0.5
    
    # VR到机器人的坐标系变换（欧拉角，单位：度）
    # 这个旋转将VR控制器的"前方向"对齐到机器人末端的期望方向
    # PICO VR: Y-up, -Z-forward (控制器指向方向是-Z)
    # 机器人夹爪: 通常X-forward或Z-down
    vr_to_robot_rotation: Tuple[float, float, float] = (-90.0, 0.0, 0.0)
    
    # 坐标轴映射: VR轴 -> 机器人轴
    # VR坐标系: X右, Y上, -Z前
    # 机器人坐标系(ROS): X前, Y左, Z上
    # 映射: Robot_X = -VR_Z, Robot_Y = -VR_X, Robot_Z = VR_Y
    # axis_mapping[i] = VR轴索引, 表示机器人第i轴来自VR哪个轴
    axis_mapping: Tuple[int, int, int] = (2, 0, 1)   # Robot XYZ <- VR ZXY
    axis_signs: Tuple[int, int, int] = (-1, -1, 1)   # Z和X需要反号


class VRClutchController:
    """单臂的Clutch控制器 - 逐帧增量跟踪"""
    
    def __init__(self, config: Optional[ClutchConfig] = None, name: str = ""):
        self.config = config or ClutchConfig()
        self.name = name  # 用于日志，如 "left" 或 "right"
        self.state = ClutchState.IDLE
        
        # 上一帧VR位姿（用于计算帧间增量）
        self.prev_vr_pos: Optional[np.ndarray] = None
        self.prev_vr_ori: Optional[np.ndarray] = None  # quaternion [x,y,z,w]
        
        # 当前机器人目标（累积更新）
        self.robot_target_pos: Optional[np.ndarray] = None
        self.robot_target_ori: Optional[np.ndarray] = None
        
        # 累积缓冲区（用于死区判断，解决慢速移动问题）
        self.accumulated_pos_delta: np.ndarray = np.zeros(3)
        self.accumulated_rot_delta: np.ndarray = np.zeros(3)  # rotvec
        
        # 构建VR到机器人的坐标变换矩阵
        # 根据axis_mapping和axis_signs构建旋转矩阵
        # axis_mapping = [2, 0, 1] 表示: Robot_X=VR_Z, Robot_Y=VR_X, Robot_Z=VR_Y
        # axis_signs = [-1, -1, 1] 表示: Robot_X=-VR_Z, Robot_Y=-VR_X, Robot_Z=+VR_Y
        self._build_coord_transform()

    def _build_coord_transform(self):
        """构建VR到机器人的坐标变换矩阵"""
        # 构建3x3旋转矩阵，将VR坐标系的向量变换到机器人坐标系
        # 矩阵的每一行表示机器人坐标系的一个轴在VR坐标系中的表示
        transform = np.zeros((3, 3))
        for robot_axis in range(3):
            vr_axis = self.config.axis_mapping[robot_axis]
            sign = self.config.axis_signs[robot_axis]
            transform[robot_axis, vr_axis] = sign
        
        # 验证是否为正交矩阵（行列式应为±1）
        det = np.linalg.det(transform)
        if abs(abs(det) - 1.0) > 0.01:
            raise ValueError(f"Invalid axis mapping: det={det}")
        
        # 转换为Rotation对象
        self.vr_to_robot_rot = Rotation.from_matrix(transform)

    def update(
        self,
        vr_pos: np.ndarray,
        vr_ori: np.ndarray,
        robot_current_pos: np.ndarray,
        robot_current_ori: np.ndarray,
        grip_value: float
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], ClutchState]:
        """
        更新Clutch控制器（逐帧增量跟踪）
        
        Args:
            vr_pos: VR手柄位置 [x, y, z] (meters)
            vr_ori: VR手柄姿态 [x, y, z, w] (quaternion)
            robot_current_pos: 机器人当前末端位置 [x, y, z] (meters)
            robot_current_ori: 机器人当前末端姿态 [x, y, z, w] (quaternion)
            grip_value: Grip按钮值 [0.0, 1.0]
            
        Returns:
            (target_pos, target_ori, state): 
                - 机器人目标位姿 (None表示不发送指令)
                - 当前Clutch状态
        """
        vr_pos = np.asarray(vr_pos, dtype=np.float64)
        vr_ori = np.asarray(vr_ori, dtype=np.float64)
        robot_current_pos = np.asarray(robot_current_pos, dtype=np.float64)
        robot_current_ori = np.asarray(robot_current_ori, dtype=np.float64)
        
        # 检测clutch按钮状态
        clutch_pressed = grip_value > self.config.engage_threshold
        clutch_released = grip_value < self.config.release_threshold
        
        # 状态机
        if self.state == ClutchState.IDLE:
            if clutch_pressed:
                # 按下grip，初始化跟踪
                self._engage_clutch(vr_pos, vr_ori, robot_current_pos, robot_current_ori)
                self.state = ClutchState.ENGAGING
                return self.robot_target_pos.copy(), self.robot_target_ori.copy(), self.state
            else:
                # 未接合，不发送指令
                return None, None, self.state
        
        elif self.state == ClutchState.ENGAGING:
            # 下一帧转为跟踪状态
            self.state = ClutchState.TRACKING
            # Fall through to TRACKING
        
        if self.state == ClutchState.TRACKING:
            if clutch_released:
                # 松开grip，停止跟踪
                self.state = ClutchState.RELEASING
                # 返回最后目标位置
                if self.robot_target_pos is not None:
                    return self.robot_target_pos.copy(), self.robot_target_ori.copy(), self.state
                return None, None, self.state
            
            # 计算这一帧VR的增量，累加到机器人目标
            self._accumulate_delta(vr_pos, vr_ori)
            
            # 更新上一帧VR位姿
            self.prev_vr_pos = vr_pos.copy()
            self.prev_vr_ori = vr_ori.copy()
            
            return self.robot_target_pos.copy(), self.robot_target_ori.copy(), self.state
        
        elif self.state == ClutchState.RELEASING:
            if clutch_pressed:
                # 重新按下，建立新的跟踪起点
                self._engage_clutch(vr_pos, vr_ori, robot_current_pos, robot_current_ori)
                self.state = ClutchState.TRACKING
                return self.robot_target_pos.copy(), self.robot_target_ori.copy(), self.state
            else:
                # 完全松开，回到IDLE
                self.state = ClutchState.IDLE
                return None, None, self.state
        
        return None, None, self.state

    def _engage_clutch(
        self,
        vr_pos: np.ndarray,
        vr_ori: np.ndarray,
        robot_pos: np.ndarray,
        robot_ori: np.ndarray
    ):
        """初始化Clutch跟踪"""
        # 记录当前VR位姿作为"上一帧"
        self.prev_vr_pos = vr_pos.copy()
        self.prev_vr_ori = vr_ori.copy()
        
        # 初始化机器人目标为当前位置
        self.robot_target_pos = robot_pos.copy()
        self.robot_target_ori = robot_ori.copy()
        
        # 清空累积缓冲区
        self.accumulated_pos_delta = np.zeros(3)
        self.accumulated_rot_delta = np.zeros(3)

    def _accumulate_delta(self, vr_pos: np.ndarray, vr_ori: np.ndarray):
        """
        计算VR帧间增量，累加到机器人目标
        
        抗抖动策略：累积式死区
        - 每帧增量先累积到缓冲区
        - 当缓冲区累积量超过死区阈值时，才应用到机器人目标
        - 这样慢速移动也能生效（多帧累积），但抖动会相互抵消
        """
        if self.prev_vr_pos is None:
            return
        
        # === 位置增量 ===
        vr_delta_pos = vr_pos - self.prev_vr_pos
        
        # 应用坐标轴映射
        mapped_delta = np.zeros(3)
        for i in range(3):
            src_axis = self.config.axis_mapping[i]
            sign = self.config.axis_signs[i]
            mapped_delta[i] = vr_delta_pos[src_axis] * sign
        
        # 应用位置缩放和低通滤波
        alpha = self.config.smoothing_factor
        robot_delta_pos = mapped_delta * self.config.position_scale * alpha
        
        # 累积到缓冲区
        self.accumulated_pos_delta += robot_delta_pos
        
        # 检查累积量是否超过死区
        accum_norm = np.linalg.norm(self.accumulated_pos_delta)
        if accum_norm >= self.config.position_deadzone:
            # 限制单次应用的最大位移
            if accum_norm > self.config.max_position_delta:
                apply_delta = (self.accumulated_pos_delta / accum_norm 
                               * self.config.max_position_delta)
                # 保留超出部分在缓冲区
                self.accumulated_pos_delta -= apply_delta
            else:
                apply_delta = self.accumulated_pos_delta
                self.accumulated_pos_delta = np.zeros(3)
            
            # 应用到机器人目标
            self.robot_target_pos += apply_delta
        
        # === 姿态增量 ===
        prev_rot = Rotation.from_quat(self.prev_vr_ori)
        curr_rot = Rotation.from_quat(vr_ori)
        vr_delta_rot = curr_rot * prev_rot.inv()
        
        # 将VR坐标系的增量旋转变换到机器人坐标系
        # robot_delta = R_vr2robot * vr_delta * R_vr2robot.inv()
        # 这样做的原因: 旋转增量需要在正确的坐标系中应用
        robot_delta_rot = (self.vr_to_robot_rot 
                          * vr_delta_rot 
                          * self.vr_to_robot_rot.inv())
        
        # 获取旋转向量并应用缩放
        rotvec = robot_delta_rot.as_rotvec()
        rotvec *= self.config.rotation_scale * alpha
        
        # 累积到缓冲区
        self.accumulated_rot_delta += rotvec
        
        # 检查累积量是否超过死区
        accum_angle = np.linalg.norm(self.accumulated_rot_delta)
        if accum_angle >= self.config.rotation_deadzone:
            # 限制单次应用的最大旋转
            if accum_angle > self.config.max_rotation_delta:
                apply_rotvec = (self.accumulated_rot_delta / accum_angle
                                * self.config.max_rotation_delta)
                self.accumulated_rot_delta -= apply_rotvec
            else:
                apply_rotvec = self.accumulated_rot_delta
                self.accumulated_rot_delta = np.zeros(3)
            
            # 应用到机器人目标姿态
            apply_rot = Rotation.from_rotvec(apply_rotvec)
            target_rot = Rotation.from_quat(self.robot_target_ori)
            new_target_rot = apply_rot * target_rot
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
