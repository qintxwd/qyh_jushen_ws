#!/usr/bin/env python3
"""
Standard Robots Matrix API 总结和临时解决方案

已实现功能：
1. 地图列表获取 ✓
2. 地图数据下载 (JSON + PNG) ✓
3. 地图数据解析 (nodes, edges, stations) ✓
4. 地图渲染 ✓

待实现功能：
1. 获取当前地图 - 需要 WebSocket + Protobuf
2. 实时状态订阅 - 需要 WebSocket + Protobuf

关于 Edge 绘制：
- type=1: 直线，使用 (sx,sy) 到 (ex,ey)
- type=3: 三次贝塞尔曲线，控制点是 (cx,cy) 和 (dx,dy)
  曲线公式: B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
  其中: P0=(sx,sy), P1=(cx,cy), P2=(dx,dy), P3=(ex,ey)

关于站点方向：
- pos.yaw 单位是毫弧度 (1/1000 rad)
- 例如: 3141.6 毫弧度 ≈ 3.1416 rad ≈ 180°
- 方向是从 X 轴正方向逆时针测量

临时解决方案 - 获取当前地图：
由于需要 Protobuf 解析，暂时提供以下方案：
1. 从 v2 API 获取地图列表，按 modify_time 排序，最新的可能是当前地图
2. 或者让用户手动指定当前地图名称
"""

import json
import requests
from pathlib import Path
from datetime import datetime

BASE = "http://192.168.71.50"

def get_maps_with_timestamps():
    """获取带时间戳的地图列表"""
    r = requests.get(f"{BASE}/api/v2/map", timeout=10)
    r.raise_for_status()
    return r.json()

def guess_current_map():
    """
    尝试猜测当前地图
    根据 top_timestamp（置顶时间）或 modify_time 判断
    """
    maps = get_maps_with_timestamps()
    
    print("地图列表 (按 modify_time 排序):")
    print("-" * 70)
    
    # 按 modify_time 排序
    sorted_maps = sorted(maps, key=lambda x: x.get('modify_time', 0), reverse=True)
    
    for m in sorted_maps:
        name = m['name']
        modify_time = m.get('modify_time', 0)
        top_timestamp = m.get('top_timestamp', 0)
        
        # 转换时间戳
        if modify_time > 1e12:  # 毫秒
            modify_time /= 1000
        modify_dt = datetime.fromtimestamp(modify_time) if modify_time > 0 else "N/A"
        
        top_marker = " ★ (置顶)" if top_timestamp > 0 else ""
        print(f"  {name}: modify={modify_dt}{top_marker}")
    
    print("-" * 70)
    
    # 返回置顶的地图，或者最新修改的地图
    topped = [m for m in maps if m.get('top_timestamp', 0) > 0]
    if topped:
        return topped[0]['name']
    
    if sorted_maps:
        return sorted_maps[0]['name']
    
    return None

def main():
    print("=" * 70)
    print("Standard Robots Matrix - 猜测当前地图")
    print("=" * 70)
    
    try:
        current = guess_current_map()
        print(f"\n猜测的当前地图: {current}")
        print("\n注意: 这只是基于修改时间的猜测。")
        print("要获取精确的当前地图，需要实现 WebSocket + Protobuf 客户端。")
    except Exception as e:
        print(f"错误: {e}")

if __name__ == '__main__':
    main()
