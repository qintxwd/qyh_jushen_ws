# ACT/ACT++ 训练框架

## 文件说明

- `act_model.py`: ACT模型架构实现
- `act_dataset.py`: 数据加载器（待创建）
- `train_act.py`: 训练脚本（待创建）
- `eval_act.py`: 评估脚本（待创建）

## 快速开始

### 1. 安装依赖

```bash
pip3 install torch torchvision torchaudio
pip3 install tensorboard
```

### 2. 测试模型

```bash
python3 act_model.py
```

### 3. 训练模型（简化版）

由于完整的训练框架较复杂，这里提供简化的训练流程：

```bash
# 准备数据
python3 ../../scripts/data_processor.py --demos_dir ~/demo_recordings

# 训练（使用简化脚本）
python3 train_simple.py --data ~/processed_data/training_data.npz
```

## 模型架构

```
输入：
  - RGB图像: (B, 3, 224, 224)
  - 关节状态: (B, 14)

处理流程：
  1. ResNet18编码图像 → (B, 512)
  2. MLP编码关节状态 → (B, 512)
  3. Transformer编码器融合特征
  4. Transformer解码器预测动作chunk
  
输出：
  - 动作序列: (B, chunk_size, 14)
```

## 训练配置

```yaml
model:
  state_dim: 14
  action_dim: 14
  chunk_size: 10
  hidden_dim: 512
  num_encoder_layers: 4
  num_decoder_layers: 4
  num_heads: 8
  
training:
  batch_size: 32
  num_epochs: 200
  learning_rate: 1e-4
  weight_decay: 1e-4
```

## 参考资料

- **ACT论文**: https://arxiv.org/abs/2304.13705
- **Mobile ALOHA**: https://arxiv.org/abs/2401.02117
