#!/usr/bin/env python3
"""
数据处理管线：将录制的演示数据转换为ACT/ACT++训练格式

输入：~/demo_recordings/demo_XXXX_timestamp/ 目录
输出：processed_data/ 目录中的 .npz 文件

数据格式：
- observations/images: (T, H, W, 3) RGB图像
- observations/qpos: (T, 14) 关节位置
- actions: (T, 14) 下一步关节位置（shifted qpos）
- episode_ends: (num_episodes,) 每个episode结束的索引
"""

import os
import json
import yaml
import numpy as np
from pathlib import Path
from PIL import Image
import cv2
from tqdm import tqdm
import argparse


class DemoDataProcessor:
    """处理演示数据的主类"""
    
    def __init__(self, demos_dir, output_dir, target_fps=30):
        """
        Args:
            demos_dir: 演示数据根目录（~/demo_recordings/）
            output_dir: 输出目录
            target_fps: 目标帧率（用于重采样）
        """
        self.demos_dir = Path(demos_dir).expanduser()
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_fps = target_fps
        
    def find_all_demos(self):
        """查找所有演示目录"""
        demo_dirs = sorted([d for d in self.demos_dir.iterdir() 
                           if d.is_dir() and d.name.startswith('demo_')])
        print(f"找到 {len(demo_dirs)} 个演示数据")
        return demo_dirs
    
    def load_metadata(self, demo_dir):
        """加载元数据"""
        metadata_file = demo_dir / "metadata.yaml"
        if not metadata_file.exists():
            return None
        with open(metadata_file, 'r') as f:
            return yaml.safe_load(f)
    
    def load_images(self, demo_dir, modality='rgb'):
        """加载图像数据"""
        img_dir = demo_dir / modality
        if not img_dir.exists():
            return []
        
        img_files = sorted(img_dir.glob('*.jpg' if modality == 'rgb' else '*.png'))
        images = []
        
        for img_file in tqdm(img_files, desc=f"Loading {modality}", leave=False):
            if modality == 'rgb':
                img = cv2.imread(str(img_file))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:  # depth
                img = cv2.imread(str(img_file), cv2.IMREAD_ANYDEPTH)
            images.append(img)
        
        return np.array(images)
    
    def load_joint_states(self, demo_dir):
        """加载关节状态数据"""
        joint_dir = demo_dir / "joint_states"
        if not joint_dir.exists():
            return None
        
        json_files = sorted(joint_dir.glob('*.json'))
        joint_states = []
        timestamps = []
        
        for json_file in tqdm(json_files, desc="Loading joint states", leave=False):
            with open(json_file, 'r') as f:
                data = json.load(f)
                # 提取14个关节的位置（右臂7个 + 左臂7个）
                positions = data['position']  # 应该是14维
                joint_states.append(positions)
                timestamps.append(data['timestamp'])
        
        return np.array(joint_states), np.array(timestamps)
    
    def resample_data(self, data, original_fps, target_fps):
        """
        重采样数据到目标帧率
        
        Args:
            data: (T, ...) 数组
            original_fps: 原始帧率
            target_fps: 目标帧率
        """
        if original_fps == target_fps:
            return data
        
        original_length = len(data)
        target_length = int(original_length * target_fps / original_fps)
        
        # 使用线性插值
        indices = np.linspace(0, original_length - 1, target_length)
        
        if data.ndim == 1:
            return np.interp(indices, np.arange(original_length), data)
        else:
            # 多维数据：对每个维度进行插值
            resampled = []
            for idx in indices:
                lower = int(np.floor(idx))
                upper = int(np.ceil(idx))
                weight = idx - lower
                
                if upper >= original_length:
                    upper = original_length - 1
                
                if lower == upper:
                    resampled.append(data[lower])
                else:
                    # 线性插值
                    interp_val = data[lower] * (1 - weight) + data[upper] * weight
                    resampled.append(interp_val)
            
            return np.array(resampled)
    
    def align_timestamps(self, rgb_images, joint_states, joint_timestamps, metadata):
        """
        对齐图像和关节状态的时间戳
        
        假设：
        - RGB图像按顺序保存，fps从metadata中获取
        - 关节状态有精确时间戳
        """
        # 获取RGB的fps
        rgb_fps = metadata.get('rgb_fps', 30)
        
        # 计算RGB的时间戳（假设从0开始）
        rgb_length = len(rgb_images)
        rgb_timestamps = np.arange(rgb_length) / rgb_fps
        
        # 将关节状态重采样到RGB的时间戳
        aligned_joint_states = []
        
        for rgb_t in rgb_timestamps:
            # 找到最近的关节状态
            idx = np.searchsorted(joint_timestamps, rgb_t)
            if idx >= len(joint_states):
                idx = len(joint_states) - 1
            elif idx > 0:
                # 线性插值
                t_lower = joint_timestamps[idx - 1]
                t_upper = joint_timestamps[idx]
                weight = (rgb_t - t_lower) / (t_upper - t_lower + 1e-8)
                joint_state = joint_states[idx - 1] * (1 - weight) + joint_states[idx] * weight
                aligned_joint_states.append(joint_state)
            else:
                aligned_joint_states.append(joint_states[0])
        
        return np.array(aligned_joint_states)
    
    def create_action_labels(self, qpos):
        """
        创建动作标签（下一步的关节位置）
        
        对于ACT，action就是未来的关节位置
        简单版本：action[t] = qpos[t+1]
        """
        actions = np.zeros_like(qpos)
        actions[:-1] = qpos[1:]  # 将位置向前移一步
        actions[-1] = qpos[-1]   # 最后一步保持不变
        return actions
    
    def apply_augmentation(self, images, apply_prob=0.5):
        """
        应用数据增强（用于训练时）
        
        Args:
            images: (T, H, W, 3) numpy数组
            apply_prob: 应用增强的概率
        """
        if np.random.rand() > apply_prob:
            return images
        
        augmented = images.copy()
        
        # 随机亮度调整
        if np.random.rand() < 0.5:
            brightness_factor = np.random.uniform(0.8, 1.2)
            augmented = np.clip(augmented * brightness_factor, 0, 255).astype(np.uint8)
        
        # 随机对比度调整
        if np.random.rand() < 0.5:
            contrast_factor = np.random.uniform(0.8, 1.2)
            mean = augmented.mean(axis=(1, 2), keepdims=True)
            augmented = np.clip((augmented - mean) * contrast_factor + mean, 0, 255).astype(np.uint8)
        
        return augmented
    
    def process_single_demo(self, demo_dir, demo_idx, apply_augmentation=False):
        """
        处理单个演示数据
        
        Returns:
            dict: 包含 observations, actions 等的字典
        """
        print(f"\n处理演示 {demo_idx}: {demo_dir.name}")
        
        # 加载元数据
        metadata = self.load_metadata(demo_dir)
        if metadata is None:
            print(f"  ⚠️  未找到元数据，跳过")
            return None
        
        # 加载RGB图像
        rgb_images = self.load_images(demo_dir, 'rgb')
        if len(rgb_images) == 0:
            print(f"  ⚠️  未找到RGB图像，跳过")
            return None
        
        # 加载关节状态
        joint_states, joint_timestamps = self.load_joint_states(demo_dir)
        if joint_states is None:
            print(f"  ⚠️  未找到关节状态，跳过")
            return None
        
        # 对齐时间戳
        aligned_qpos = self.align_timestamps(rgb_images, joint_states, joint_timestamps, metadata)
        
        # 确保长度一致
        min_length = min(len(rgb_images), len(aligned_qpos))
        rgb_images = rgb_images[:min_length]
        aligned_qpos = aligned_qpos[:min_length]
        
        # 创建动作标签
        actions = self.create_action_labels(aligned_qpos)
        
        # 数据增强（可选）
        if apply_augmentation:
            rgb_images = self.apply_augmentation(rgb_images)
        
        print(f"  ✓ 处理完成: {len(rgb_images)} 帧, 图像形状 {rgb_images.shape}")
        
        return {
            'observations': {
                'images': rgb_images,  # (T, H, W, 3)
                'qpos': aligned_qpos   # (T, 14)
            },
            'actions': actions,        # (T, 14)
            'metadata': metadata
        }
    
    def process_all_demos(self, output_filename='training_data.npz', apply_augmentation=False):
        """
        处理所有演示并保存为单个npz文件
        
        输出格式（ACT标准格式）：
        - observations/images: (N, H, W, 3) 所有图像拼接
        - observations/qpos: (N, 14) 所有关节状态拼接
        - actions: (N, 14) 所有动作拼接
        - episode_ends: (num_episodes,) 每个episode结束的索引
        """
        demo_dirs = self.find_all_demos()
        
        if len(demo_dirs) == 0:
            print("❌ 未找到演示数据")
            return
        
        all_images = []
        all_qpos = []
        all_actions = []
        episode_ends = []
        
        total_frames = 0
        
        for idx, demo_dir in enumerate(demo_dirs):
            result = self.process_single_demo(demo_dir, idx, apply_augmentation)
            
            if result is None:
                continue
            
            all_images.append(result['observations']['images'])
            all_qpos.append(result['observations']['qpos'])
            all_actions.append(result['actions'])
            
            total_frames += len(result['observations']['images'])
            episode_ends.append(total_frames - 1)  # 0-indexed
        
        if len(all_images) == 0:
            print("❌ 没有成功处理的演示数据")
            return
        
        # 拼接所有数据
        all_images = np.concatenate(all_images, axis=0)
        all_qpos = np.concatenate(all_qpos, axis=0)
        all_actions = np.concatenate(all_actions, axis=0)
        episode_ends = np.array(episode_ends)
        
        # 保存为npz
        output_file = self.output_dir / output_filename
        np.savez_compressed(
            output_file,
            **{
                'observations/images': all_images,
                'observations/qpos': all_qpos,
                'actions': all_actions,
                'episode_ends': episode_ends
            }
        )
        
        print(f"\n{'='*60}")
        print(f"✓ 数据处理完成！")
        print(f"  总帧数: {total_frames}")
        print(f"  总episodes: {len(episode_ends)}")
        print(f"  图像形状: {all_images.shape}")
        print(f"  输出文件: {output_file}")
        print(f"  文件大小: {output_file.stat().st_size / 1024 / 1024:.2f} MB")
        print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description='处理演示数据为ACT训练格式')
    parser.add_argument('--demos_dir', type=str, 
                       default='~/demo_recordings',
                       help='演示数据根目录')
    parser.add_argument('--output_dir', type=str,
                       default='~/processed_data',
                       help='输出目录')
    parser.add_argument('--output_file', type=str,
                       default='training_data.npz',
                       help='输出文件名')
    parser.add_argument('--target_fps', type=int, default=30,
                       help='目标帧率')
    parser.add_argument('--augment', action='store_true',
                       help='应用数据增强')
    
    args = parser.parse_args()
    
    processor = DemoDataProcessor(
        demos_dir=args.demos_dir,
        output_dir=args.output_dir,
        target_fps=args.target_fps
    )
    
    processor.process_all_demos(
        output_filename=args.output_file,
        apply_augmentation=args.augment
    )


if __name__ == '__main__':
    main()
