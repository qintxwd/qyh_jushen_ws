#!/usr/bin/env python3
"""
分析如何获取当前地图名称
从 JS 代码中寻找线索
"""
import re
from pathlib import Path

def search():
    js_path = Path(__file__).parent / 'web_src' / 'js' / 'app.d3121814.js'
    with open(js_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    print("=" * 70)
    print("搜索获取当前地图的方法")
    print("=" * 70)
    
    # 搜索 systemState.mapName 相关代码
    patterns = [
        (r'systemState\.mapName[^,;]{0,80}', "systemState.mapName"),
        (r'mapName[^,;]{0,50}', "mapName"),
        (r'loadedMapName[^,;]{0,50}', "loadedMapName"),
        (r'displayedMapName[^,;]{0,50}', "displayedMapName"),
    ]
    
    for pattern, desc in patterns:
        matches = re.findall(pattern, content)
        if matches:
            print(f"\n【{desc}】 ({len(matches)} 处)")
            unique = list(set(matches))[:10]
            for m in unique:
                print(f"  {m}")
    
    # 搜索 RESPONSE_SYSTEM_STATE 相关
    print("\n" + "=" * 70)
    print("搜索 SystemState 响应处理")
    print("=" * 70)
    
    # 找到 systemState 结构
    sys_state_patterns = [
        r'systemState\s*=\s*\{[^}]{0,500}',
        r'SystemState\s*\{[^}]{0,500}',
        r'"mapName"[^,}]{0,50}',
    ]
    
    for pattern in sys_state_patterns:
        matches = re.findall(pattern, content)
        if matches:
            print(f"\n【{pattern}】")
            for m in matches[:3]:
                print(f"  {m[:200]}")
    
    # 搜索 v2 API 中是否有状态
    print("\n" + "=" * 70)
    print("搜索可能的状态 API")
    print("=" * 70)
    
    api_patterns = [
        r'/api/v\d/[a-z]+/state',
        r'/api/v\d/state[a-z]*',
        r'/api/v\d/current[a-z]*',
        r'fetch\([^)]*state[^)]*\)',
        r'axios\.[a-z]+\([^)]*state[^)]*\)',
    ]
    
    for pattern in api_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        if matches:
            print(f"\n【{pattern}】")
            for m in list(set(matches))[:5]:
                print(f"  {m}")

if __name__ == '__main__':
    search()
