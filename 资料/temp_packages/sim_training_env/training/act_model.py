#!/usr/bin/env python3
"""
ACT (Action Chunking Transformer) 模型实现

基于论文：
- ACT: https://arxiv.org/abs/2304.13705
- Mobile ALOHA: https://arxiv.org/abs/2401.02117

模型架构：
1. 视觉编码器：ResNet18 提取图像特征
2. 状态编码器：MLP 编码关节状态
3. Transformer编码器：融合多模态特征
4. Transformer解码器：预测动作序列（action chunks）
"""

import torch
import torch.nn as nn
import torchvision.models as models
from typing import Tuple, Optional


class ResNetEncoder(nn.Module):
    """ResNet18视觉编码器"""
    
    def __init__(self, pretrained: bool = True, output_dim: int = 512):
        """
        Args:
            pretrained: 是否使用预训练权重
            output_dim: 输出特征维度
        """
        super().__init__()
        
        # 加载预训练的ResNet18
        resnet = models.resnet18(pretrained=pretrained)
        
        # 移除最后的全连接层和平均池化层
        self.backbone = nn.Sequential(*list(resnet.children())[:-2])
        
        # ResNet18最后一层输出512通道
        self.adaptive_pool = nn.AdaptiveAvgPool2d((1, 1))
        
        # 可选的投影层
        self.projection = nn.Linear(512, output_dim) if output_dim != 512 else nn.Identity()
        
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, 3, H, W) 输入图像
        
        Returns:
            features: (B, output_dim) 图像特征
        """
        # 提取特征
        x = self.backbone(x)  # (B, 512, H', W')
        
        # 全局平均池化
        x = self.adaptive_pool(x)  # (B, 512, 1, 1)
        x = x.flatten(1)  # (B, 512)
        
        # 投影
        x = self.projection(x)  # (B, output_dim)
        
        return x


class StateEncoder(nn.Module):
    """状态编码器（MLP）"""
    
    def __init__(self, state_dim: int = 14, hidden_dim: int = 256, output_dim: int = 512):
        """
        Args:
            state_dim: 状态维度（关节数量）
            hidden_dim: 隐藏层维度
            output_dim: 输出特征维度
        """
        super().__init__()
        
        self.encoder = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, state_dim) 关节状态
        
        Returns:
            features: (B, output_dim) 状态特征
        """
        return self.encoder(x)


class ACTModel(nn.Module):
    """ACT (Action Chunking Transformer) 模型"""
    
    def __init__(
        self,
        state_dim: int = 14,
        action_dim: int = 14,
        chunk_size: int = 10,
        hidden_dim: int = 512,
        num_encoder_layers: int = 4,
        num_decoder_layers: int = 4,
        num_heads: int = 8,
        dropout: float = 0.1,
        use_vision: bool = True
    ):
        """
        Args:
            state_dim: 状态维度（关节数量）
            action_dim: 动作维度（与state_dim相同）
            chunk_size: 动作块大小（一次预测多少步）
            hidden_dim: Transformer隐藏维度
            num_encoder_layers: Transformer编码器层数
            num_decoder_layers: Transformer解码器层数
            num_heads: 多头注意力头数
            dropout: Dropout概率
            use_vision: 是否使用视觉输入
        """
        super().__init__()
        
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.chunk_size = chunk_size
        self.hidden_dim = hidden_dim
        self.use_vision = use_vision
        
        # 视觉编码器
        if use_vision:
            self.vision_encoder = ResNetEncoder(pretrained=True, output_dim=hidden_dim)
        
        # 状态编码器
        self.state_encoder = StateEncoder(
            state_dim=state_dim,
            hidden_dim=hidden_dim // 2,
            output_dim=hidden_dim
        )
        
        # Transformer
        self.transformer = nn.Transformer(
            d_model=hidden_dim,
            nhead=num_heads,
            num_encoder_layers=num_encoder_layers,
            num_decoder_layers=num_decoder_layers,
            dim_feedforward=hidden_dim * 4,
            dropout=dropout,
            batch_first=True
        )
        
        # 位置编码
        self.encoder_pos_embedding = nn.Parameter(
            torch.randn(1, 2 if use_vision else 1, hidden_dim) * 0.02
        )
        self.decoder_pos_embedding = nn.Parameter(
            torch.randn(1, chunk_size, hidden_dim) * 0.02
        )
        
        # 动作查询（可学习的decoder输入）
        self.action_queries = nn.Parameter(
            torch.randn(1, chunk_size, hidden_dim) * 0.02
        )
        
        # 动作预测头
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )
    
    def forward(
        self,
        qpos: torch.Tensor,
        images: Optional[torch.Tensor] = None,
        actions: Optional[torch.Tensor] = None
    ) -> torch.Tensor:
        """
        前向传播
        
        Args:
            qpos: (B, state_dim) 当前关节状态
            images: (B, 3, H, W) 输入图像（可选）
            actions: (B, chunk_size, action_dim) 目标动作（训练时使用）
        
        Returns:
            predicted_actions: (B, chunk_size, action_dim) 预测的动作序列
        """
        batch_size = qpos.shape[0]
        
        # 编码器输入
        encoder_tokens = []
        
        # 编码关节状态
        state_features = self.state_encoder(qpos)  # (B, hidden_dim)
        state_features = state_features.unsqueeze(1)  # (B, 1, hidden_dim)
        encoder_tokens.append(state_features)
        
        # 编码图像（如果有）
        if self.use_vision and images is not None:
            vision_features = self.vision_encoder(images)  # (B, hidden_dim)
            vision_features = vision_features.unsqueeze(1)  # (B, 1, hidden_dim)
            encoder_tokens.append(vision_features)
        
        # 拼接encoder tokens
        encoder_input = torch.cat(encoder_tokens, dim=1)  # (B, N, hidden_dim)
        encoder_input = encoder_input + self.encoder_pos_embedding[:, :encoder_input.shape[1], :]
        
        # 解码器输入（可学习的动作查询）
        decoder_input = self.action_queries.expand(batch_size, -1, -1)  # (B, chunk_size, hidden_dim)
        decoder_input = decoder_input + self.decoder_pos_embedding
        
        # Transformer
        transformer_output = self.transformer(
            src=encoder_input,
            tgt=decoder_input
        )  # (B, chunk_size, hidden_dim)
        
        # 预测动作
        predicted_actions = self.action_head(transformer_output)  # (B, chunk_size, action_dim)
        
        return predicted_actions
    
    def get_action(
        self,
        qpos: torch.Tensor,
        images: Optional[torch.Tensor] = None
    ) -> torch.Tensor:
        """
        推理时获取下一个动作
        
        Args:
            qpos: (B, state_dim) 当前关节状态
            images: (B, 3, H, W) 输入图像（可选）
        
        Returns:
            action: (B, action_dim) 下一步动作（chunk的第一个）
        """
        with torch.no_grad():
            action_chunk = self.forward(qpos, images)
            return action_chunk[:, 0, :]  # 返回第一个动作


def create_act_model(config: dict) -> ACTModel:
    """
    根据配置创建ACT模型
    
    Args:
        config: 模型配置字典
    
    Returns:
        model: ACT模型实例
    """
    return ACTModel(
        state_dim=config.get('state_dim', 14),
        action_dim=config.get('action_dim', 14),
        chunk_size=config.get('chunk_size', 10),
        hidden_dim=config.get('hidden_dim', 512),
        num_encoder_layers=config.get('num_encoder_layers', 4),
        num_decoder_layers=config.get('num_decoder_layers', 4),
        num_heads=config.get('num_heads', 8),
        dropout=config.get('dropout', 0.1),
        use_vision=config.get('use_vision', True)
    )
