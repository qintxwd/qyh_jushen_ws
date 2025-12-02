"""
Standard Robots Matrix API 模块

提供与 Standard Robots Matrix 系统交互的 Python API。
"""

from .client import (
    MatrixClient,
    MapInfo,
    MapMeta,
    MapNode,
    MapEdge,
    MapStation,
    download_maps,
    list_maps,
    # Edge 类型常量
    EDGE_TYPE_LINE,
    EDGE_TYPE_ARC,
    EDGE_TYPE_BEZIER,
)

__all__ = [
    'MatrixClient',
    'MapInfo',
    'MapMeta',
    'MapNode',
    'MapEdge',
    'MapStation',
    'download_maps',
    'list_maps',
    'EDGE_TYPE_LINE',
    'EDGE_TYPE_ARC',
    'EDGE_TYPE_BEZIER',
]

__version__ = '0.1.0'
