"""
ACT 模型构建器

从 qyh_act_training 导入模型架构，用于推理时重建模型
"""

import os
import sys
from typing import Dict, Any, Optional

# 尝试从训练包导入模型
_ACT_TRAINING_PATH = os.path.expanduser("~/qyh-robot-system/qyh_act_training")
if os.path.exists(_ACT_TRAINING_PATH):
    sys.path.insert(0, _ACT_TRAINING_PATH)


def build_act_model(config: Dict[str, Any]) -> Any:
    """
    根据配置构建 ACT 模型
    
    Args:
        config: 模型配置字典（从 checkpoint['config'] 获取）
        
    Returns:
        构建好的 ACTPolicy 模型
    """
    try:
        # 尝试从训练包导入
        from qyh_act_training.models import ACTPolicy, ACTConfig
        
        # 将字典配置转换为 ACTConfig
        if isinstance(config, dict):
            act_config = ACTConfig(
                # 维度
                qpos_dim=config.get('qpos_dim', 7),
                action_dim=config.get('action_dim', 8),
                
                # 序列长度
                observation_horizon=config.get('observation_horizon', 10),
                action_horizon=config.get('action_horizon', 20),
                
                # 网络结构
                hidden_dim=config.get('hidden_dim', 512),
                num_heads=config.get('num_heads', 8),
                num_encoder_layers=config.get('num_encoder_layers', 4),
                num_decoder_layers=config.get('num_decoder_layers', 4),
                ff_dim=config.get('ff_dim', config.get('hidden_dim', 512) * 4),
                dropout=config.get('dropout', 0.1),
                
                # VAE
                latent_dim=config.get('latent_dim', 32),
                kl_weight=config.get('kl_weight', 10.0),
                
                # 视觉
                camera_names=config.get('camera_names', ['head']),
                vision_backbone=config.get('vision_backbone', 'resnet18'),
                vision_pretrained=False,  # 推理时不需要预训练
                freeze_vision=True,
            )
        else:
            act_config = config
        
        # 构建模型
        model = ACTPolicy(act_config)
        print(f"[ModelBuilder] Built ACTPolicy from qyh_act_training")
        print(f"  qpos_dim={act_config.qpos_dim}, action_dim={act_config.action_dim}")
        print(f"  cameras={act_config.camera_names}")
        
        return model
        
    except ImportError as e:
        print(f"[ModelBuilder] Cannot import from qyh_act_training: {e}")
        print("[ModelBuilder] Falling back to local placeholder model")
        return _build_placeholder_model(config)


def _build_placeholder_model(config: Dict[str, Any]) -> Any:
    """
    占位模型（当训练包不可用时使用）
    
    注意：这只是一个简单的占位，实际使用需要与训练模型架构完全一致
    """
    import torch
    import torch.nn as nn
    
    class PlaceholderACTModel(nn.Module):
        """简单占位模型"""
        
        def __init__(self, config):
            super().__init__()
            self.config = config
            
            qpos_dim = config.get('qpos_dim', 7)
            action_dim = config.get('action_dim', 8)
            action_horizon = config.get('action_horizon', 20)
            hidden_dim = config.get('hidden_dim', 512)
            
            # 简单的 MLP
            self.encoder = nn.Sequential(
                nn.Linear(qpos_dim, hidden_dim),
                nn.ReLU(),
                nn.Linear(hidden_dim, hidden_dim),
                nn.ReLU(),
            )
            
            self.decoder = nn.Linear(hidden_dim, action_horizon * action_dim)
            
            self.action_horizon = action_horizon
            self.action_dim = action_dim
        
        def forward(self, qpos, images=None, actions=None, is_training=False):
            """
            前向传播
            
            Args:
                qpos: [B, T, qpos_dim]
                images: 图像字典
                actions: 目标动作（训练时用）
                is_training: 是否训练模式
                
            Returns:
                如果 is_training: (action_pred, loss_dict)
                否则: action_pred [B, K, action_dim]
            """
            B = qpos.shape[0]
            
            # 取最后一帧
            last_qpos = qpos[:, -1, :]  # [B, qpos_dim]
            
            # 编码
            h = self.encoder(last_qpos)  # [B, hidden_dim]
            
            # 解码
            out = self.decoder(h)  # [B, K * action_dim]
            action_pred = out.view(B, self.action_horizon, self.action_dim)
            
            if is_training and actions is not None:
                loss = nn.functional.mse_loss(action_pred, actions)
                return action_pred, {'loss': loss, 'recon_loss': loss}
            
            return action_pred
        
        def get_action(self, qpos, images=None):
            """推理时获取动作"""
            with torch.no_grad():
                return self.forward(qpos, images, is_training=False)
    
    print("[ModelBuilder] WARNING: Using placeholder model!")
    print("[ModelBuilder] This model structure may not match the trained model!")
    
    return PlaceholderACTModel(config)


# 导出
__all__ = ['build_act_model']
