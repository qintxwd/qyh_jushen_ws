"""
推理配置类

集中管理所有推理相关参数，便于调参和部署
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
import os


@dataclass
class InferenceConfig:
    """
    ACT 推理配置
    
    包含所有部署时需要调整的参数，按照实战经验设置默认值
    """
    
    # ==================== 模型配置 ====================
    model_path: str = ""                    # 模型文件路径 (policy.pt)
    normalization_path: str = ""            # normalization 参数路径 (mean.npy, std.npy)
    device: str = "cuda"                    # 推理设备 (cuda/cpu)
    
    # ==================== 控制参数 ====================
    # 历史观测窗口大小
    observation_horizon: int = 10           # N - 历史帧数
    # 动作预测窗口大小
    action_horizon: int = 20                # K - 预测未来步数
    # 每次执行的动作步数
    action_steps: int = 3                   # M - 每次推理后执行几步（滑动窗口）
    
    # ==================== 频率配置 ====================
    control_frequency: float = 20.0         # 控制频率 (Hz)
    inference_frequency: float = 10.0       # 推理频率 (Hz)，通常比控制频率低
    
    # ==================== 动作缩放与平滑 ====================
    # 动作缩放因子（关键调参项！）
    action_scale: float = 0.4               # 初始建议 0.3-0.5，稳定后可提高
    # 动作平滑因子 (EMA: a_t = α * a_pred + (1-α) * a_{t-1})
    smoothing_alpha: float = 0.3            # 0.2-0.5，越小越平滑
    
    # ==================== 安全限制 ====================
    # 关节速度限制 (rad/s)
    max_joint_velocity: float = 1.0
    # 单步关节变化限制 (rad)
    max_joint_delta: float = 0.05           # 单步最大变化量
    # 夹爪阈值（二值化）
    gripper_threshold: float = 0.6
    # 夹爪滞后区间（防抖）
    gripper_hysteresis: float = 0.1
    
    # ==================== 关节配置 ====================
    # 机械臂关节数
    arm_dof: int = 7
    # 左臂关节限位 (min, max) - 弧度
    left_arm_joint_limits: List[tuple] = field(default_factory=lambda: [
        (-3.14, 3.14),   # J1
        (-2.09, 2.09),   # J2
        (-3.14, 3.14),   # J3
        (-2.09, 2.09),   # J4
        (-3.14, 3.14),   # J5
        (-2.09, 2.09),   # J6
        (-3.14, 3.14),   # J7
    ])
    # 右臂关节限位
    right_arm_joint_limits: List[tuple] = field(default_factory=lambda: [
        (-3.14, 3.14),
        (-2.09, 2.09),
        (-3.14, 3.14),
        (-2.09, 2.09),
        (-3.14, 3.14),
        (-2.09, 2.09),
        (-3.14, 3.14),
    ])
    
    # ==================== 话题配置 ====================
    # 输入话题 - 关节状态
    left_arm_state_topic: str = "/left_arm/joint_states"
    right_arm_state_topic: str = "/right_arm/joint_states"
    
    # 输入话题 - 相机 (与 action.yaml 中 collection.cameras 保持一致)
    head_camera_topic: str = "/head_camera/color/image_raw"
    head_camera_depth_topic: str = "/head_camera/depth/image_raw"
    left_wrist_camera_topic: str = "/left_camera/color/image_raw"
    right_wrist_camera_topic: str = "/right_camera/color/image_raw"
    
    # 输入话题 - 夹爪状态 (与 action.yaml 中 collection.grippers 保持一致)
    left_gripper_state_topic: str = "/left/gripper_state"
    right_gripper_state_topic: str = "/right/gripper_state"
    
    # 输出话题 - 机械臂 servo_j 命令 (Float64MultiArray, 7 joints)
    left_arm_servo_j_topic: str = "/teleop/left/servo_j"
    right_arm_servo_j_topic: str = "/teleop/right/servo_j"
    
    # 输出服务 - 夹爪控制 (qyh_gripper_msgs/srv/MoveGripper)
    left_gripper_service: str = "/left/move_gripper"
    right_gripper_service: str = "/right/move_gripper"
    
    # ==================== 模式配置 ====================
    # 使用哪些输入
    use_left_arm: bool = False              # 是否使用左臂
    use_right_arm: bool = True              # 是否使用右臂
    use_left_gripper: bool = False          # 是否控制左夹爪
    use_right_gripper: bool = True          # 是否控制右夹爪
    use_head_camera: bool = True            # 是否使用头部相机
    use_wrist_cameras: bool = False         # 是否使用腕部相机
    use_depth: bool = False                 # 是否使用深度图
    
    # 动作输出类型
    action_type: str = "delta"              # "delta" (增量) 或 "absolute" (绝对)
    
    # ==================== 调试配置 ====================
    verbose: bool = False                   # 详细日志
    visualize: bool = False                 # 可视化预测轨迹
    record_inference: bool = False          # 记录推理数据
    
    def __post_init__(self):
        """后处理：设置派生参数"""
        self.control_period = 1.0 / self.control_frequency
        self.inference_period = 1.0 / self.inference_frequency
        
        # 计算动作维度
        self.action_dim = 0
        if self.use_left_arm:
            self.action_dim += self.arm_dof
        if self.use_right_arm:
            self.action_dim += self.arm_dof
        if self.use_left_gripper:
            self.action_dim += 1
        if self.use_right_gripper:
            self.action_dim += 1
    
    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'InferenceConfig':
        """从 YAML 文件加载配置"""
        import yaml
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        return cls(**data)
    
    @classmethod
    def from_ros_params(cls, node) -> 'InferenceConfig':
        """从 ROS2 参数加载配置"""
        config = cls()
        
        # 声明并获取参数
        params_map = {
            'model_path': config.model_path,
            'normalization_path': config.normalization_path,
            'device': config.device,
            'observation_horizon': config.observation_horizon,
            'action_horizon': config.action_horizon,
            'action_steps': config.action_steps,
            'control_frequency': config.control_frequency,
            'inference_frequency': config.inference_frequency,
            'action_scale': config.action_scale,
            'smoothing_alpha': config.smoothing_alpha,
            'max_joint_velocity': config.max_joint_velocity,
            'max_joint_delta': config.max_joint_delta,
            'gripper_threshold': config.gripper_threshold,
            'gripper_hysteresis': config.gripper_hysteresis,
            'use_left_arm': config.use_left_arm,
            'use_right_arm': config.use_right_arm,
            'use_left_gripper': config.use_left_gripper,
            'use_right_gripper': config.use_right_gripper,
            'use_head_camera': config.use_head_camera,
            'use_wrist_cameras': config.use_wrist_cameras,
            'use_depth': config.use_depth,
            'action_type': config.action_type,
            'verbose': config.verbose,
            'left_arm_state_topic': config.left_arm_state_topic,
            'right_arm_state_topic': config.right_arm_state_topic,
            'head_camera_topic': config.head_camera_topic,
            'head_camera_depth_topic': config.head_camera_depth_topic,
            'left_wrist_camera_topic': config.left_wrist_camera_topic,
            'right_wrist_camera_topic': config.right_wrist_camera_topic,
            'left_gripper_state_topic': config.left_gripper_state_topic,
            'right_gripper_state_topic': config.right_gripper_state_topic,
            'left_arm_servo_j_topic': config.left_arm_servo_j_topic,
            'right_arm_servo_j_topic': config.right_arm_servo_j_topic,
            'left_gripper_service': config.left_gripper_service,
            'right_gripper_service': config.right_gripper_service,
        }
        
        for param_name, default_value in params_map.items():
            node.declare_parameter(param_name, default_value)
            setattr(config, param_name, node.get_parameter(param_name).value)
        
        config.__post_init__()
        return config
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'model_path': self.model_path,
            'device': self.device,
            'observation_horizon': self.observation_horizon,
            'action_horizon': self.action_horizon,
            'action_steps': self.action_steps,
            'control_frequency': self.control_frequency,
            'action_scale': self.action_scale,
            'smoothing_alpha': self.smoothing_alpha,
            'action_dim': self.action_dim,
            'use_left_arm': self.use_left_arm,
            'use_right_arm': self.use_right_arm,
            'action_type': self.action_type,
        }


# ==================== 预设配置模板 ====================

def get_right_arm_pickup_config() -> InferenceConfig:
    """
    右手单臂夹取配置
    
    适用于：头部相机 + 右臂 + 右夹爪 的简单夹取任务
    """
    return InferenceConfig(
        use_left_arm=False,
        use_right_arm=True,
        use_left_gripper=False,
        use_right_gripper=True,
        use_head_camera=True,
        use_wrist_cameras=False,
        use_depth=False,
        action_scale=0.3,           # 初始保守值
        smoothing_alpha=0.3,
        action_steps=3,
    )


def get_dual_arm_config() -> InferenceConfig:
    """
    双臂协同配置
    
    适用于：需要双手协作的复杂任务
    """
    return InferenceConfig(
        use_left_arm=True,
        use_right_arm=True,
        use_left_gripper=True,
        use_right_gripper=True,
        use_head_camera=True,
        use_wrist_cameras=True,
        use_depth=True,
        action_scale=0.2,           # 双臂需要更保守
        smoothing_alpha=0.4,        # 更强的平滑
        action_steps=2,
    )


def get_debug_config() -> InferenceConfig:
    """
    调试配置
    
    适用于：初次部署测试
    """
    return InferenceConfig(
        use_right_arm=True,
        use_right_gripper=True,
        use_head_camera=True,
        action_scale=0.1,           # 非常保守
        smoothing_alpha=0.5,        # 强平滑
        control_frequency=10.0,     # 低频
        verbose=True,
        visualize=True,
    )
