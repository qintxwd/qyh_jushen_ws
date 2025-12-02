#!/usr/bin/env python3
"""
Standard Robots Matrix API 客户端

此模块提供与 Standard Robots Matrix 系统交互的 Python API，
支持地图、账户、配置等功能的访问。

跨平台支持：Windows/Linux/ARM

使用示例：
    from matrix.client import MatrixClient
    
    client = MatrixClient("192.168.71.50")
    maps = client.get_map_list()
    client.download_map("standard", "./maps")
"""

import requests
import json
import os
from pathlib import Path
from typing import Optional, Dict, List, Any, Union
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


@dataclass
class MapInfo:
    """地图信息"""
    name: str
    md5: str
    modify_time: str


@dataclass
class MapMeta:
    """地图元数据"""
    version: str
    resolution: float  # mm/pixel
    width: int  # pixels
    height: int  # pixels
    origin_x: float  # mm
    origin_y: float  # mm
    length_unit: str
    angle_unit: str


@dataclass
class MapNode:
    """导航节点"""
    id: int
    x: float  # mm
    y: float  # mm
    yaw: float  # rad
    desc: str = ""


# Edge 类型常量
EDGE_TYPE_LINE = 1      # 直线
EDGE_TYPE_ARC = 2       # 圆弧
EDGE_TYPE_BEZIER = 3    # 三次贝塞尔曲线


@dataclass
class MapEdge:
    """
    路径边
    
    绘制规则:
    - type=1 (LINE): 直线，从 (sx,sy) 到 (ex,ey)
    - type=3 (BEZIER): 三次贝塞尔曲线
      P0=(sx,sy), P1=(cx,cy), P2=(dx,dy), P3=(ex,ey)
      B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
    """
    id: int
    s_node: int  # 起始节点 ID
    e_node: int  # 结束节点 ID
    sx: float  # 起点 x (mm)
    sy: float  # 起点 y (mm)
    ex: float  # 终点 x (mm)
    ey: float  # 终点 y (mm)
    cost: float
    edge_type: int = EDGE_TYPE_LINE  # 1=直线, 3=贝塞尔曲线
    # 贝塞尔曲线控制点
    cx: float = 0  # 控制点1 x (mm)
    cy: float = 0  # 控制点1 y (mm)
    dx: float = 0  # 控制点2 x (mm)
    dy: float = 0  # 控制点2 y (mm)
    desc: str = ""
    
    def is_bezier(self) -> bool:
        """是否为贝塞尔曲线"""
        return self.edge_type == EDGE_TYPE_BEZIER
    
    def get_bezier_points(self):
        """获取贝塞尔曲线的四个控制点"""
        return (
            (self.sx, self.sy),  # P0: 起点
            (self.cx, self.cy),  # P1: 控制点1
            (self.dx, self.dy),  # P2: 控制点2
            (self.ex, self.ey),  # P3: 终点
        )


@dataclass
class MapStation:
    """
    工位/站点
    
    站点方向 (pos_yaw):
    - 单位: 毫弧度 (1/1000 rad)
    - 例如: 3141.6 ≈ π rad ≈ 180°
    - 方向: 从 X 轴正方向逆时针测量
    
    转换为弧度: yaw_rad = pos_yaw / 1000.0
    转换为角度: yaw_deg = pos_yaw / 1000.0 * 180 / π
    """
    id: int
    name: str
    station_type: str
    pos_x: float  # mm
    pos_y: float  # mm
    pos_yaw: float  # 毫弧度 (1/1000 rad)
    desc: str = ""
    
    def get_yaw_rad(self) -> float:
        """获取方向角（弧度）"""
        return self.pos_yaw / 1000.0
    
    def get_yaw_deg(self) -> float:
        """获取方向角（度）"""
        import math
        return self.pos_yaw / 1000.0 * 180.0 / math.pi


class MatrixClient:
    """
    Standard Robots Matrix REST API 客户端
    
    Args:
        host: Matrix 服务器 IP 地址
        port: HTTP 端口，默认 80
        timeout: 请求超时时间，默认 30 秒
    """
    
    def __init__(self, host: str, port: int = 80, timeout: int = 30):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.base_url = f"http://{host}:{port}" if port != 80 else f"http://{host}"
        self.session = requests.Session()
    
    def _get(self, endpoint: str) -> requests.Response:
        """发送 GET 请求"""
        url = f"{self.base_url}{endpoint}"
        logger.debug(f"GET {url}")
        resp = self.session.get(url, timeout=self.timeout)
        resp.raise_for_status()
        return resp
    
    def _post(self, endpoint: str, data: Any = None) -> requests.Response:
        """发送 POST 请求"""
        url = f"{self.base_url}{endpoint}"
        logger.debug(f"POST {url}")
        resp = self.session.post(url, json=data, timeout=self.timeout)
        resp.raise_for_status()
        return resp
    
    # ==================== 地图 API ====================
    
    def get_map_list(self) -> List[MapInfo]:
        """
        获取地图列表
        
        Returns:
            地图信息列表
        """
        resp = self._get("/api/v0/map")
        data = resp.json()
        maps = data.get('maps', [])
        return [MapInfo(
            name=m['name'],
            md5=m['md5'],
            modify_time=m['modify_time']
        ) for m in maps]
    
    def get_map_list_v2(self) -> List[Dict]:
        """
        获取地图列表 (v2 API，包含更多信息)
        
        Returns:
            地图详细信息列表
        """
        resp = self._get("/api/v2/map")
        return resp.json()
    
    def get_map_data(self, map_name: str) -> Dict:
        """
        获取地图完整数据（JSON 格式）
        
        Args:
            map_name: 地图名称
            
        Returns:
            包含 meta, data, private 的完整地图数据
        """
        resp = self._get(f"/api/v0/map/{map_name}/data")
        return resp.json()
    
    def get_map_image(self, map_name: str, scale: float = 1.0) -> bytes:
        """
        获取地图图片（PNG 格式）
        
        Args:
            map_name: 地图名称
            scale: 缩放比例，1.0 表示原始大小
            
        Returns:
            PNG 图片二进制数据
        """
        resp = self._get(f"/api/v0/map/{map_name}/image?scale={scale}")
        return resp.content
    
    def get_map_meta(self, map_name: str) -> MapMeta:
        """
        获取地图元数据
        
        Args:
            map_name: 地图名称
            
        Returns:
            地图元数据
        """
        data = self.get_map_data(map_name)
        meta = data.get('meta', {})
        return MapMeta(
            version=meta.get('version', ''),
            resolution=meta.get('resolution', 1),
            width=meta.get('size.x', 0),
            height=meta.get('size.y', 0),
            origin_x=meta.get('zero_offset.x', 0) * meta.get('resolution', 1),
            origin_y=meta.get('zero_offset.y', 0) * meta.get('resolution', 1),
            length_unit=meta.get('length_unit', 'mm'),
            angle_unit=meta.get('angle_unit', 'rad')
        )
    
    def get_map_nodes(self, map_name: str) -> List[MapNode]:
        """
        获取地图导航节点
        
        Args:
            map_name: 地图名称
            
        Returns:
            导航节点列表
        """
        data = self.get_map_data(map_name)
        nodes = data.get('data', {}).get('node', [])
        return [MapNode(
            id=n['id'],
            x=n['x'],
            y=n['y'],
            yaw=n.get('yaw', 0),
            desc=n.get('desc', '')
        ) for n in nodes]
    
    def get_map_edges(self, map_name: str) -> List[MapEdge]:
        """
        获取地图路径边
        
        Args:
            map_name: 地图名称
            
        Returns:
            路径边列表
            
        Note:
            - type=1: 直线
            - type=3: 贝塞尔曲线，使用 cx,cy,dx,dy 作为控制点
        """
        data = self.get_map_data(map_name)
        edges = data.get('data', {}).get('edge', [])
        return [MapEdge(
            id=e['id'],
            s_node=e.get('s_node', 0),
            e_node=e.get('e_node', 0),
            sx=e.get('sx', 0),
            sy=e.get('sy', 0),
            ex=e.get('ex', 0),
            ey=e.get('ey', 0),
            cost=e.get('cost', 0),
            edge_type=e.get('type', 1),
            cx=e.get('cx', 0),
            cy=e.get('cy', 0),
            dx=e.get('dx', 0),
            dy=e.get('dy', 0),
            desc=e.get('desc', '')
        ) for e in edges]
    
    def get_map_stations(self, map_name: str) -> List[MapStation]:
        """
        获取地图工位/站点
        
        Args:
            map_name: 地图名称
            
        Returns:
            站点列表
        """
        data = self.get_map_data(map_name)
        stations = data.get('data', {}).get('station', [])
        return [MapStation(
            id=s['id'],
            name=s.get('name', ''),
            station_type=s.get('type', ''),
            pos_x=s.get('pos.x', 0),
            pos_y=s.get('pos.y', 0),
            pos_yaw=s.get('pos.yaw', 0),
            desc=s.get('desc', '')
        ) for s in stations]
    
    def download_map(self, map_name: str, output_dir: Union[str, Path]) -> Dict[str, Path]:
        """
        下载地图到本地目录
        
        Args:
            map_name: 地图名称
            output_dir: 输出目录
            
        Returns:
            包含下载文件路径的字典 {'json': Path, 'image': Path}
        """
        output_dir = Path(output_dir)
        map_dir = output_dir / map_name
        map_dir.mkdir(parents=True, exist_ok=True)
        
        result = {}
        
        # 下载 JSON 数据
        data = self.get_map_data(map_name)
        json_path = map_dir / f"{map_name}.json"
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        result['json'] = json_path
        logger.info(f"Downloaded map JSON: {json_path}")
        
        # 下载图片
        img_data = self.get_map_image(map_name)
        img_path = map_dir / f"{map_name}.png"
        with open(img_path, 'wb') as f:
            f.write(img_data)
        result['image'] = img_path
        logger.info(f"Downloaded map image: {img_path}")
        
        return result
    
    def download_all_maps(self, output_dir: Union[str, Path]) -> Dict[str, Dict[str, Path]]:
        """
        下载所有地图
        
        Args:
            output_dir: 输出目录
            
        Returns:
            所有地图的下载结果
        """
        result = {}
        maps = self.get_map_list()
        for map_info in maps:
            logger.info(f"Downloading map: {map_info.name}")
            result[map_info.name] = self.download_map(map_info.name, output_dir)
        return result
    
    # ==================== 账户 API ====================
    
    def get_accounts(self) -> List[Dict]:
        """
        获取账户列表
        
        Returns:
            账户信息列表
        """
        resp = self._get("/api/v0/accounts")
        return resp.json()
    
    def get_account(self, username: str) -> Dict:
        """
        获取指定账户信息
        
        Args:
            username: 用户名
            
        Returns:
            账户信息
        """
        resp = self._get(f"/api/v0/account/{username}")
        return resp.json()
    
    # ==================== 系统信息 API ====================
    
    def guess_current_map(self) -> Optional[str]:
        """
        猜测当前地图（基于修改时间）
        
        由于获取精确的当前地图需要 WebSocket + Protobuf，
        这个方法通过比较地图修改时间来猜测。
        
        Returns:
            猜测的当前地图名称，或 None
        """
        maps = self.get_map_list_v2()
        if not maps:
            return None
        
        # 按 modify_time 排序，最新的可能是当前地图
        sorted_maps = sorted(
            maps, 
            key=lambda x: x.get('modify_time', 0), 
            reverse=True
        )
        
        return sorted_maps[0]['name'] if sorted_maps else None
    
    def get_current_map(self) -> str:
        """
        获取当前加载的地图名称
        
        Returns:
            地图名称
        """
        resp = self._get("/api/v0/map/current")
        data = resp.json()
        return data.get('name', '')
    
    def set_current_map(self, map_name: str) -> bool:
        """
        设置当前地图
        
        Args:
            map_name: 地图名称
            
        Returns:
            是否成功
        """
        resp = self._post(f"/api/v0/map/{map_name}/load")
        return resp.status_code == 200


# 便捷函数

def download_maps(host: str, output_dir: str = "./maps") -> Dict:
    """
    便捷函数：下载所有地图
    
    Args:
        host: Matrix 服务器 IP
        output_dir: 输出目录
        
    Returns:
        下载结果
    """
    client = MatrixClient(host)
    return client.download_all_maps(output_dir)


def list_maps(host: str) -> List[str]:
    """
    便捷函数：列出所有地图名称
    
    Args:
        host: Matrix 服务器 IP
        
    Returns:
        地图名称列表
    """
    client = MatrixClient(host)
    return [m.name for m in client.get_map_list()]


if __name__ == '__main__':
    # 示例用法
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.71.50"
    
    client = MatrixClient(host)
    
    print("=== 地图列表 ===")
    for m in client.get_map_list():
        print(f"  {m.name} (修改时间: {m.modify_time})")
    
    print("\n=== standard 地图详情 ===")
    try:
        meta = client.get_map_meta("standard")
        print(f"  分辨率: {meta.resolution} mm/pixel")
        print(f"  尺寸: {meta.width}x{meta.height} pixels")
        
        nodes = client.get_map_nodes("standard")
        print(f"  节点数: {len(nodes)}")
        
        edges = client.get_map_edges("standard")
        print(f"  边数: {len(edges)}")
        
        stations = client.get_map_stations("standard")
        print(f"  站点数: {len(stations)}")
        if stations:
            print(f"  第一个站点: {stations[0].name} ({stations[0].pos_x}, {stations[0].pos_y})")
    except Exception as e:
        print(f"  获取地图详情失败: {e}")
