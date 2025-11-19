#!/usr/bin/env python3
"""
数据可视化工具：用于检查处理后的训练数据

可以：
1. 显示数据集统计信息
2. 可视化某个episode的图像序列
3. 绘制关节轨迹
4. 检查数据质量
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import cv2


class DataVisualizer:
    """数据可视化工具"""
    
    def __init__(self, data_file):
        """
        Args:
            data_file: .npz文件路径
        """
        self.data_file = Path(data_file).expanduser()
        self.data = None
        self.load_data()
    
    def load_data(self):
        """加载npz数据"""
        if not self.data_file.exists():
            raise FileNotFoundError(f"数据文件不存在: {self.data_file}")
        
        print(f"加载数据: {self.data_file}")
        self.data = np.load(self.data_file, allow_pickle=True)
        
        # 打印所有键
        print(f"数据键: {list(self.data.keys())}")
    
    def print_statistics(self):
        """打印数据集统计信息"""
        images = self.data['observations/images']
        qpos = self.data['observations/qpos']
        actions = self.data['actions']
        episode_ends = self.data['episode_ends']
        
        num_episodes = len(episode_ends)
        total_frames = len(images)
        
        print("\n" + "="*60)
        print("数据集统计信息")
        print("="*60)
        print(f"总帧数: {total_frames}")
        print(f"总episodes: {num_episodes}")
        print(f"平均每个episode帧数: {total_frames / num_episodes:.1f}")
        print(f"\n图像:")
        print(f"  形状: {images.shape}")
        print(f"  数据类型: {images.dtype}")
        print(f"  值范围: [{images.min()}, {images.max()}]")
        print(f"\n关节状态 (qpos):")
        print(f"  形状: {qpos.shape}")
        print(f"  数据类型: {qpos.dtype}")
        print(f"  值范围: [{qpos.min():.3f}, {qpos.max():.3f}]")
        print(f"\n动作 (actions):")
        print(f"  形状: {actions.shape}")
        print(f"  数据类型: {actions.dtype}")
        print(f"  值范围: [{actions.min():.3f}, {actions.max():.3f}]")
        print(f"\nEpisode结束索引: {episode_ends}")
        
        # 计算每个episode的长度
        episode_lengths = []
        start = 0
        for end in episode_ends:
            episode_lengths.append(end - start + 1)
            start = end + 1
        
        print(f"\n每个episode的长度: {episode_lengths}")
        print("="*60 + "\n")
    
    def get_episode_data(self, episode_idx):
        """
        获取指定episode的数据
        
        Args:
            episode_idx: episode索引（0-based）
        
        Returns:
            dict: 包含该episode的images, qpos, actions
        """
        episode_ends = self.data['episode_ends']
        
        if episode_idx >= len(episode_ends):
            raise ValueError(f"Episode索引超出范围: {episode_idx} >= {len(episode_ends)}")
        
        # 计算起始和结束索引
        if episode_idx == 0:
            start = 0
        else:
            start = episode_ends[episode_idx - 1] + 1
        end = episode_ends[episode_idx]
        
        return {
            'images': self.data['observations/images'][start:end+1],
            'qpos': self.data['observations/qpos'][start:end+1],
            'actions': self.data['actions'][start:end+1]
        }
    
    def visualize_episode_images(self, episode_idx, sample_rate=5):
        """
        可视化某个episode的图像序列
        
        Args:
            episode_idx: episode索引
            sample_rate: 采样率（每隔几帧显示一次）
        """
        episode_data = self.get_episode_data(episode_idx)
        images = episode_data['images']
        
        num_frames = len(images)
        sampled_indices = list(range(0, num_frames, sample_rate))
        num_samples = len(sampled_indices)
        
        # 计算子图布局
        cols = min(5, num_samples)
        rows = (num_samples + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(cols*3, rows*3))
        if num_samples == 1:
            axes = [axes]
        else:
            axes = axes.flatten() if rows > 1 else axes
        
        for i, idx in enumerate(sampled_indices):
            if i >= len(axes):
                break
            axes[i].imshow(images[idx])
            axes[i].set_title(f"Frame {idx}/{num_frames-1}")
            axes[i].axis('off')
        
        # 隐藏多余的子图
        for i in range(num_samples, len(axes)):
            axes[i].axis('off')
        
        plt.suptitle(f"Episode {episode_idx} 图像序列 (每{sample_rate}帧)")
        plt.tight_layout()
        plt.show()
    
    def plot_joint_trajectories(self, episode_idx):
        """
        绘制关节轨迹
        
        Args:
            episode_idx: episode索引
        """
        episode_data = self.get_episode_data(episode_idx)
        qpos = episode_data['qpos']  # (T, 14)
        actions = episode_data['actions']  # (T, 14)
        
        num_frames = len(qpos)
        time_steps = np.arange(num_frames)
        
        # 14个关节分成两组（右臂7个，左臂7个）
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # 右臂关节
        for i in range(7):
            axes[0].plot(time_steps, qpos[:, i], label=f'r-j{i+1}', alpha=0.7)
        axes[0].set_xlabel('Time Step')
        axes[0].set_ylabel('Joint Position (rad)')
        axes[0].set_title(f'Episode {episode_idx} - 右臂关节轨迹')
        axes[0].legend(ncol=4, fontsize=8)
        axes[0].grid(True, alpha=0.3)
        
        # 左臂关节
        for i in range(7):
            axes[1].plot(time_steps, qpos[:, i+7], label=f'l-j{i+1}', alpha=0.7)
        axes[1].set_xlabel('Time Step')
        axes[1].set_ylabel('Joint Position (rad)')
        axes[1].set_title(f'Episode {episode_idx} - 左臂关节轨迹')
        axes[1].legend(ncol=4, fontsize=8)
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def create_video(self, episode_idx, output_file='episode_video.mp4', fps=30):
        """
        创建episode的视频文件
        
        Args:
            episode_idx: episode索引
            output_file: 输出视频文件路径
            fps: 视频帧率
        """
        episode_data = self.get_episode_data(episode_idx)
        images = episode_data['images']
        
        if len(images) == 0:
            print("错误: 没有图像数据")
            return
        
        height, width = images[0].shape[:2]
        
        # 创建VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
        
        for img in images:
            # OpenCV使用BGR格式
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            out.write(img_bgr)
        
        out.release()
        print(f"视频已保存: {output_file}")
    
    def check_data_quality(self):
        """检查数据质量"""
        print("\n" + "="*60)
        print("数据质量检查")
        print("="*60)
        
        images = self.data['observations/images']
        qpos = self.data['observations/qpos']
        actions = self.data['actions']
        
        # 检查NaN值
        has_nan_images = np.isnan(images).any()
        has_nan_qpos = np.isnan(qpos).any()
        has_nan_actions = np.isnan(actions).any()
        
        print(f"图像包含NaN: {has_nan_images}")
        print(f"关节状态包含NaN: {has_nan_qpos}")
        print(f"动作包含NaN: {has_nan_actions}")
        
        # 检查关节范围（通常在 [-π, π]）
        qpos_out_of_range = np.abs(qpos) > np.pi
        if qpos_out_of_range.any():
            print(f"⚠️  警告: {qpos_out_of_range.sum()} 个关节值超出 [-π, π] 范围")
        else:
            print(f"✓ 所有关节值在合理范围内")
        
        # 检查图像亮度
        mean_brightness = images.mean()
        print(f"平均图像亮度: {mean_brightness:.2f} (正常范围: 50-200)")
        
        if mean_brightness < 30:
            print("⚠️  警告: 图像过暗")
        elif mean_brightness > 220:
            print("⚠️  警告: 图像过亮")
        else:
            print("✓ 图像亮度正常")
        
        print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='可视化训练数据')
    parser.add_argument('data_file', type=str,
                       help='训练数据文件 (.npz)')
    parser.add_argument('--stats', action='store_true',
                       help='显示统计信息')
    parser.add_argument('--check', action='store_true',
                       help='检查数据质量')
    parser.add_argument('--episode', type=int, default=0,
                       help='要可视化的episode索引')
    parser.add_argument('--images', action='store_true',
                       help='显示图像序列')
    parser.add_argument('--joints', action='store_true',
                       help='显示关节轨迹')
    parser.add_argument('--video', type=str,
                       help='生成视频文件')
    parser.add_argument('--sample_rate', type=int, default=5,
                       help='图像采样率')
    
    args = parser.parse_args()
    
    visualizer = DataVisualizer(args.data_file)
    
    if args.stats:
        visualizer.print_statistics()
    
    if args.check:
        visualizer.check_data_quality()
    
    if args.images:
        visualizer.visualize_episode_images(args.episode, args.sample_rate)
    
    if args.joints:
        visualizer.plot_joint_trajectories(args.episode)
    
    if args.video:
        visualizer.create_video(args.episode, args.video)
    
    # 如果没有指定任何操作，默认显示统计信息
    if not any([args.stats, args.check, args.images, args.joints, args.video]):
        visualizer.print_statistics()
        visualizer.check_data_quality()


if __name__ == '__main__':
    main()
