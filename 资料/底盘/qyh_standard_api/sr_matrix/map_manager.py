"""
地图管理器
负责获取、保存和处理地图数据
"""

import os
import json
from typing import Optional, Dict, List, Tuple
from pathlib import Path

try:
    from PIL import Image
    import numpy as np
    HAS_PIL = True
except ImportError:
    HAS_PIL = False
    print("警告: PIL/Pillow 未安装，部分图像处理功能不可用")

from .matrix_client import MatrixClient


class MapManager:
    """地图管理器"""
    
    def __init__(self, client: MatrixClient, save_path: str = "./maps"):
        """
        初始化地图管理器
        
        Args:
            client: MatrixClient 实例
            save_path: 地图保存路径
        """
        self.client = client
        self.save_path = Path(save_path)
        self.save_path.mkdir(parents=True, exist_ok=True)
        
        # 缓存
        self._map_list_cache: List[Dict] = []
        self._current_map_cache: Optional[Dict] = None
    
    def list_maps(self, refresh: bool = False) -> List[Dict]:
        """
        获取地图列表
        
        Args:
            refresh: 是否强制刷新缓存
            
        Returns:
            地图列表
        """
        if not refresh and self._map_list_cache:
            return self._map_list_cache
        
        self._map_list_cache = self.client.get_map_list()
        return self._map_list_cache
    
    def get_current_map_info(self, refresh: bool = False) -> Optional[Dict]:
        """
        获取当前地图信息
        
        Args:
            refresh: 是否强制刷新
            
        Returns:
            当前地图信息
        """
        if not refresh and self._current_map_cache:
            return self._current_map_cache
        
        self._current_map_cache = self.client.get_current_map()
        return self._current_map_cache
    
    def download_map(self, map_name: str = None, map_id: str = None) -> Optional[str]:
        """
        下载地图并保存到本地
        
        Args:
            map_name: 地图名称
            map_id: 地图 ID
            
        Returns:
            保存的文件路径，失败返回 None
        """
        # 获取地图数据
        map_data = self.client.get_map_data(map_name=map_name, map_id=map_id)
        if map_data:
            # 保存元数据
            filename = map_name or map_id or "unknown"
            meta_path = self.save_path / f"{filename}_meta.json"
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(map_data, f, indent=2, ensure_ascii=False)
            print(f"地图元数据已保存: {meta_path}")
        
        # 获取地图图片
        image_data = self.client.get_map_image(map_name=map_name, map_id=map_id)
        if image_data:
            filename = map_name or map_id or "unknown"
            image_path = self.save_path / f"{filename}.png"
            with open(image_path, "wb") as f:
                f.write(image_data)
            print(f"地图图片已保存: {image_path}")
            return str(image_path)
        
        return None
    
    def download_current_map(self) -> Optional[str]:
        """
        下载当前地图
        
        Returns:
            保存的文件路径
        """
        current = self.get_current_map_info(refresh=True)
        if current:
            map_name = current.get("name") or current.get("mapName")
            map_id = current.get("id") or current.get("mapId")
            return self.download_map(map_name=map_name, map_id=map_id)
        else:
            print("无法获取当前地图信息")
            return None
    
    def download_all_maps(self) -> List[str]:
        """
        下载所有地图
        
        Returns:
            保存的文件路径列表
        """
        maps = self.list_maps(refresh=True)
        saved_paths = []
        
        for map_info in maps:
            map_name = map_info.get("name") or map_info.get("mapName")
            map_id = map_info.get("id") or map_info.get("mapId")
            
            print(f"正在下载地图: {map_name or map_id}")
            path = self.download_map(map_name=map_name, map_id=map_id)
            if path:
                saved_paths.append(path)
        
        return saved_paths
    
    def get_sites_for_map(self, map_name: str = None) -> List[Dict]:
        """
        获取指定地图的站点列表
        
        Args:
            map_name: 地图名称，None 表示当前地图
            
        Returns:
            站点列表
        """
        return self.client.get_sites(map_name=map_name)
    
    def save_sites(self, map_name: str = None) -> Optional[str]:
        """
        保存站点信息到文件
        
        Args:
            map_name: 地图名称
            
        Returns:
            保存的文件路径
        """
        sites = self.get_sites_for_map(map_name)
        if sites:
            filename = map_name or "current"
            sites_path = self.save_path / f"{filename}_sites.json"
            with open(sites_path, "w", encoding="utf-8") as f:
                json.dump(sites, f, indent=2, ensure_ascii=False)
            print(f"站点信息已保存: {sites_path}")
            return str(sites_path)
        return None
    
    # ==================== 图像处理 ====================
    
    def load_map_image(self, map_name: str) -> Optional['Image.Image']:
        """
        加载本地地图图片
        
        Args:
            map_name: 地图名称
            
        Returns:
            PIL Image 对象
        """
        if not HAS_PIL:
            print("PIL 未安装")
            return None
        
        image_path = self.save_path / f"{map_name}.png"
        if image_path.exists():
            return Image.open(image_path)
        return None
    
    def map_to_occupancy_grid(self, map_name: str) -> Optional[Tuple[np.ndarray, Dict]]:
        """
        将地图图片转换为 ROS 风格的 OccupancyGrid
        
        Args:
            map_name: 地图名称
            
        Returns:
            (grid_array, meta_info) 元组
            grid_array: numpy 数组, -1=未知, 0=空闲, 100=占用
            meta_info: 元数据 (分辨率, 原点等)
        """
        if not HAS_PIL:
            print("需要 PIL 库")
            return None
        
        image = self.load_map_image(map_name)
        if image is None:
            return None
        
        # 转换为灰度图
        gray = image.convert('L')
        img_array = np.array(gray)
        
        # 转换为 OccupancyGrid 格式
        # 黑色 (0) -> 100 (占用)
        # 白色 (255) -> 0 (空闲)
        # 灰色 -> -1 (未知) 或按比例
        grid = np.zeros_like(img_array, dtype=np.int8)
        
        # 简单阈值转换
        grid[img_array < 50] = 100      # 黑色 -> 占用
        grid[img_array > 200] = 0       # 白色 -> 空闲
        grid[(img_array >= 50) & (img_array <= 200)] = -1  # 灰色 -> 未知
        
        # 加载元数据
        meta_path = self.save_path / f"{map_name}_meta.json"
        meta_info = {}
        if meta_path.exists():
            with open(meta_path, "r", encoding="utf-8") as f:
                meta_info = json.load(f)
        
        # 添加图像尺寸
        meta_info["width"] = grid.shape[1]
        meta_info["height"] = grid.shape[0]
        
        return grid, meta_info
