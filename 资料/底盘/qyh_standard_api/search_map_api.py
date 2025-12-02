#!/usr/bin/env python3
"""搜索获取当前地图和曲线绘制的 API"""
import re
from pathlib import Path

def search_js():
    js_path = Path(__file__).parent / 'web_src' / 'js' / 'app.d3121814.js'
    with open(js_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    print("=" * 70)
    print("搜索当前地图 API")
    print("=" * 70)
    
    # 搜索 current map 相关
    patterns = [
        r'current[_\-]?map[^;]{0,100}',
        r'currentMap[^;]{0,100}',
        r'cur[_\-]?map[^;]{0,100}',
        r'activeMap[^;]{0,100}',
        r'loadedMap[^;]{0,100}',
        r'/map/current[^"\']*',
        r'GET_CUR_MAP[^;]{0,100}',
    ]
    
    for pattern in patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            print(f"\n【{pattern}】")
            for m in list(set(matches))[:5]:
                print(f"  {m[:100]}")
    
    print("\n" + "=" * 70)
    print("搜索曲线绘制代码")
    print("=" * 70)
    
    # 搜索贝塞尔曲线相关
    bezier_patterns = [
        r'bezier[^;]{0,150}',
        r'quadraticCurveTo[^;]{0,150}',
        r'bezierCurveTo[^;]{0,150}',
        r'ctx\.curve[^;]{0,100}',
        r'drawEdge[^}]{0,300}',
        r'lineTo[^;]{0,80}',
    ]
    
    for pattern in bezier_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            print(f"\n【{pattern}】找到 {len(matches)} 处")
            for m in matches[:3]:
                print(f"  {m[:150]}")
    
    print("\n" + "=" * 70)
    print("搜索 edge type 绘制逻辑")
    print("=" * 70)
    
    # 搜索 edge type 判断
    edge_type_patterns = [
        r'edge\.type[^;]{0,100}',
        r'\.type===?3[^;]{0,100}',
        r'type===?1[^;]{0,50}',
    ]
    
    for pattern in edge_type_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            print(f"\n【{pattern}】找到 {len(matches)} 处")
            for m in list(set(matches))[:5]:
                print(f"  {m[:100]}")
    
    print("\n" + "=" * 70)
    print("搜索 WebSocket 获取状态")
    print("=" * 70)
    
    # 搜索实时状态获取
    state_patterns = [
        r'RESPONSE_ALL_STATE[^;]{0,100}',
        r'robotState[^;]{0,80}',
        r'mapName[^;]{0,80}',
    ]
    
    for pattern in state_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            print(f"\n【{pattern}】")
            for m in list(set(matches))[:3]:
                print(f"  {m[:100]}")

if __name__ == '__main__':
    search_js()
