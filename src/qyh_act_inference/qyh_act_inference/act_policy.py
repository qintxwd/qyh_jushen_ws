"""
ACT Policy 模型加载和推理

核心推理模块，负责：
1. 加载训练好的 ACT 模型
2. 处理观测数据（图像 + 状态）
3. 执行推理，输出动作序列
"""

import os
import numpy as np
from typing import Optional, Dict, List, Tuple, Any
from collections import deque
from dataclasses import dataclass

# 延迟导入 torch，避免非 GPU 环境报错
torch = None


@dataclass
class Observation:
    """单帧观测数据"""
    # 关节状态
    left_arm_joints: Optional[np.ndarray] = None     # (7,) 左臂关节角度
    right_arm_joints: Optional[np.ndarray] = None    # (7,) 右臂关节角度
    left_gripper: Optional[float] = None             # 左夹爪状态 [0, 1]
    right_gripper: Optional[float] = None            # 右夹爪状态 [0, 1]
    
    # 图像
    head_image: Optional[np.ndarray] = None          # (H, W, 3) RGB
    left_wrist_image: Optional[np.ndarray] = None    # (H, W, 3) RGB
    right_wrist_image: Optional[np.ndarray] = None   # (H, W, 3) RGB
    head_depth: Optional[np.ndarray] = None          # (H, W) 深度图
    
    # 时间戳
    timestamp: float = 0.0


class ACTPolicy:
    """
    ACT (Action Chunking Transformer) 策略模型
    
    负责加载模型并执行推理
    """
    
    def __init__(self, config):
        """
        初始化 ACT Policy
        
        Args:
            config: InferenceConfig 配置对象
        """
        self.config = config
        self.device = None
        self.model = None
        self.is_loaded = False
        
        # Normalization 参数
        self.state_mean = None
        self.state_std = None
        self.action_mean = None
        self.action_std = None
        
        # 观测缓冲区
        self.obs_buffer: deque = deque(maxlen=config.observation_horizon)
        
        # 图像预处理参数
        self.image_size = (224, 224)  # ACT 默认输入尺寸
        self.image_mean = np.array([0.485, 0.456, 0.406])
        self.image_std = np.array([0.229, 0.224, 0.225])
    
    def load(self) -> bool:
        """
        加载模型和 normalization 参数
        
        Returns:
            是否加载成功
        """
        global torch
        
        try:
            # 延迟导入 PyTorch
            import torch as _torch
            torch = _torch
            
            # 设置设备
            if self.config.device == "cuda" and torch.cuda.is_available():
                self.device = torch.device("cuda")
                print(f"[ACTPolicy] Using CUDA: {torch.cuda.get_device_name(0)}")
            else:
                self.device = torch.device("cpu")
                print("[ACTPolicy] Using CPU")
            
            # 加载模型
            if not os.path.exists(self.config.model_path):
                print(f"[ACTPolicy] Model not found: {self.config.model_path}")
                return False
            
            print(f"[ACTPolicy] Loading model from: {self.config.model_path}")
            
            # 加载模型（支持多种格式）
            checkpoint = torch.load(
                self.config.model_path, 
                map_location=self.device,
                weights_only=False
            )
            
            # 判断是完整模型还是 state_dict
            if isinstance(checkpoint, dict):
                # 获取配置
                model_config = checkpoint.get('config', {})
                
                # 尝试多种 state_dict 键名
                state_dict = None
                for key in ['model_state_dict', 'model', 'state_dict']:
                    if key in checkpoint:
                        state_dict = checkpoint[key]
                        break
                
                if state_dict is not None:
                    # 需要先构建模型架构
                    self.model = self._build_model(model_config)
                    self.model.load_state_dict(state_dict)
                else:
                    # 假设整个 dict 就是 state_dict
                    self.model = self._build_model({})
                    self.model.load_state_dict(checkpoint)
            else:
                # 完整模型（包含架构）
                self.model = checkpoint
            
            self.model.to(self.device)
            self.model.eval()
            
            # 加载 normalization 参数
            self._load_normalization()
            
            self.is_loaded = True
            print("[ACTPolicy] Model loaded successfully")
            return True
            
        except Exception as e:
            print(f"[ACTPolicy] Failed to load model: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _build_model(self, model_config: Dict) -> Any:
        """
        构建 ACT 模型架构
        
        使用 model_builder 模块，自动尝试从 qyh_act_training 导入模型架构
        """
        try:
            from .model_builder import build_act_model
            return build_act_model(model_config)
        except ImportError as e:
            print(f"[ACTPolicy] Cannot import model_builder: {e}")
            print("[ACTPolicy] Warning: Using placeholder model")
            return PlaceholderModel(self.config)
    
    def _load_normalization(self):
        """加载 normalization 参数"""
        norm_path = self.config.normalization_path
        
        if not norm_path or not os.path.exists(norm_path):
            print("[ACTPolicy] No normalization path, using identity normalization")
            return
        
        try:
            # 尝试加载 npz 格式
            if norm_path.endswith('.npz'):
                data = np.load(norm_path, allow_pickle=True)
                
                # 兼容两种键名格式:
                # - qyh_act_training 导出: qpos_mean, qpos_std
                # - 旧格式: state_mean, state_std
                if 'qpos_mean' in data:
                    self.state_mean = data['qpos_mean']
                    self.state_std = data['qpos_std']
                elif 'state_mean' in data:
                    self.state_mean = data.get('state_mean', None)
                    self.state_std = data.get('state_std', None)
                
                self.action_mean = data.get('action_mean', None)
                self.action_std = data.get('action_std', None)
            else:
                # 尝试加载目录下的多个文件
                norm_dir = norm_path
                if os.path.exists(os.path.join(norm_dir, 'qpos_mean.npy')):
                    self.state_mean = np.load(os.path.join(norm_dir, 'qpos_mean.npy'))
                    self.state_std = np.load(os.path.join(norm_dir, 'qpos_std.npy'))
                elif os.path.exists(os.path.join(norm_dir, 'state_mean.npy')):
                    self.state_mean = np.load(os.path.join(norm_dir, 'state_mean.npy'))
                    self.state_std = np.load(os.path.join(norm_dir, 'state_std.npy'))
                if os.path.exists(os.path.join(norm_dir, 'action_mean.npy')):
                    self.action_mean = np.load(os.path.join(norm_dir, 'action_mean.npy'))
                    self.action_std = np.load(os.path.join(norm_dir, 'action_std.npy'))
            
            print(f"[ACTPolicy] Loaded normalization from: {norm_path}")
            if self.state_mean is not None:
                print(f"  State shape: {self.state_mean.shape}")
            if self.action_mean is not None:
                print(f"  Action shape: {self.action_mean.shape}")
                
        except Exception as e:
            print(f"[ACTPolicy] Failed to load normalization: {e}")
    
    def reset(self):
        """重置观测缓冲区"""
        self.obs_buffer.clear()
    
    def add_observation(self, obs: Observation):
        """
        添加一帧观测到缓冲区
        
        Args:
            obs: Observation 对象
        """
        self.obs_buffer.append(obs)
    
    def is_ready(self) -> bool:
        """
        检查是否有足够的观测数据进行推理
        
        Returns:
            缓冲区是否已满
        """
        return len(self.obs_buffer) >= self.config.observation_horizon
    
    def infer(self) -> Optional[np.ndarray]:
        """
        执行推理
        
        Returns:
            action_chunk: (K, action_dim) 动作序列，或 None 如果推理失败
        """
        if not self.is_loaded:
            print("[ACTPolicy] Model not loaded!")
            return None
        
        if not self.is_ready():
            print(f"[ACTPolicy] Not enough observations: {len(self.obs_buffer)}/{self.config.observation_horizon}")
            return None
        
        try:
            # 准备输入
            state_seq, image_seq = self._prepare_input()
            
            # 推理
            with torch.no_grad():
                # 根据模型类型调用不同的接口
                if hasattr(self.model, 'get_action'):
                    action_chunk = self.model.get_action(state_seq, image_seq)
                elif hasattr(self.model, 'forward'):
                    action_chunk = self.model(state_seq, image_seq)
                else:
                    action_chunk = self.model(state_seq, image_seq)
            
            # 转换为 numpy
            if isinstance(action_chunk, torch.Tensor):
                action_chunk = action_chunk.cpu().numpy()
            
            # 反归一化
            action_chunk = self._denormalize_action(action_chunk)
            
            return action_chunk
            
        except Exception as e:
            print(f"[ACTPolicy] Inference failed: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _prepare_input(self) -> Tuple[Any, Any]:
        """
        准备模型输入
        
        Returns:
            (state_tensor, image_tensor)
        """
        # 收集状态序列
        states = []
        images = []
        
        for obs in self.obs_buffer:
            # 构建状态向量
            state = self._build_state_vector(obs)
            states.append(state)
            
            # 处理图像
            img = self._process_image(obs)
            if img is not None:
                images.append(img)
        
        # 转换为 tensor
        state_seq = np.stack(states, axis=0)  # (N, state_dim)
        state_seq = self._normalize_state(state_seq)
        state_tensor = torch.from_numpy(state_seq).float().unsqueeze(0).to(self.device)  # (1, N, state_dim)
        
        if images:
            image_seq = np.stack(images, axis=0)  # (N, C, H, W)
            image_tensor = torch.from_numpy(image_seq).float().unsqueeze(0).to(self.device)  # (1, N, C, H, W)
        else:
            image_tensor = None
        
        return state_tensor, image_tensor
    
    def _build_state_vector(self, obs: Observation) -> np.ndarray:
        """
        从观测构建状态向量
        
        根据 config 中的配置决定包含哪些状态
        """
        state_parts = []
        
        if self.config.use_left_arm and obs.left_arm_joints is not None:
            state_parts.append(obs.left_arm_joints)
        
        if self.config.use_right_arm and obs.right_arm_joints is not None:
            state_parts.append(obs.right_arm_joints)
        
        if self.config.use_left_gripper and obs.left_gripper is not None:
            state_parts.append(np.array([obs.left_gripper]))
        
        if self.config.use_right_gripper and obs.right_gripper is not None:
            state_parts.append(np.array([obs.right_gripper]))
        
        if state_parts:
            return np.concatenate(state_parts)
        else:
            return np.zeros(self.config.action_dim)
    
    def _process_image(self, obs: Observation) -> Optional[np.ndarray]:
        """
        处理图像：resize + normalize
        
        Returns:
            (C, H, W) 归一化后的图像，或 None
        """
        import cv2
        
        image = None
        
        # 根据配置选择图像
        if self.config.use_head_camera and obs.head_image is not None:
            image = obs.head_image
        elif self.config.use_wrist_cameras:
            if obs.right_wrist_image is not None:
                image = obs.right_wrist_image
            elif obs.left_wrist_image is not None:
                image = obs.left_wrist_image
        
        if image is None:
            return None
        
        # Resize
        image = cv2.resize(image, self.image_size)
        
        # Normalize: (H, W, C) -> (C, H, W), [0, 255] -> [0, 1] -> normalized
        image = image.astype(np.float32) / 255.0
        image = (image - self.image_mean) / self.image_std
        image = image.transpose(2, 0, 1)  # (C, H, W)
        
        return image
    
    def _normalize_state(self, state: np.ndarray) -> np.ndarray:
        """归一化状态"""
        if self.state_mean is not None and self.state_std is not None:
            return (state - self.state_mean) / (self.state_std + 1e-8)
        return state
    
    def _denormalize_action(self, action: np.ndarray) -> np.ndarray:
        """反归一化动作"""
        if self.action_mean is not None and self.action_std is not None:
            return action * self.action_std + self.action_mean
        return action


class PlaceholderModel:
    """
    占位模型，用于测试流程
    
    返回零动作
    """
    
    def __init__(self, config):
        self.config = config
        self.action_dim = config.action_dim or 8  # 7 joints + 1 gripper
        self.action_horizon = config.action_horizon
    
    def to(self, device):
        return self
    
    def eval(self):
        return self
    
    def __call__(self, state_seq, image_seq=None):
        """返回零动作"""
        batch_size = state_seq.shape[0] if hasattr(state_seq, 'shape') else 1
        return np.zeros((batch_size, self.action_horizon, self.action_dim))
    
    def get_action(self, state_seq, image_seq=None):
        return self(state_seq, image_seq)
