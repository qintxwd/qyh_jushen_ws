#!/usr/bin/env python3
"""分析地图 JSON 数据结构"""
import json
from pathlib import Path

def analyze_map(json_path):
    """分析单个地图"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    map_name = json_path.stem
    print(f"\n{'='*70}")
    print(f"地图: {map_name}")
    print(f"{'='*70}")
    
    # Meta 信息
    if 'meta' in data:
        meta = data['meta']
        print(f"\n【Meta 信息】")
        print(f"  版本: {meta.get('version', '?')}")
        print(f"  分辨率: {meta.get('resolution', '?')} mm/pixel")
        print(f"  尺寸: {meta.get('size.x', '?')} x {meta.get('size.y', '?')} pixels")
        print(f"  原点偏移: ({meta.get('zero_offset.x', '?')}, {meta.get('zero_offset.y', '?')})")
        print(f"  长度单位: {meta.get('length_unit', '?')}")
        print(f"  角度单位: {meta.get('angle_unit', '?')}")
    
    # Data 信息
    if 'data' in data:
        d = data['data']
        print(f"\n【Data 内容】")
        
        for key in d.keys():
            val = d[key]
            if isinstance(val, list):
                print(f"  {key}: List[{len(val)}]")
                if val and isinstance(val[0], dict):
                    print(f"    示例 keys: {list(val[0].keys())[:10]}")
            elif isinstance(val, dict):
                print(f"  {key}: Dict keys={list(val.keys())[:5]}")
            else:
                print(f"  {key}: {type(val).__name__} = {str(val)[:50]}")
        
        # 详细分析 poses
        if 'poses' in d and d['poses']:
            poses = d['poses']
            print(f"\n  【Poses 详情】")
            print(f"    共 {len(poses)} 个位置点")
            if poses:
                first = poses[0]
                print(f"    示例: {json.dumps(first, ensure_ascii=False)[:200]}")
        
        # 详细分析 paths
        if 'paths' in d and d['paths']:
            paths = d['paths']
            print(f"\n  【Paths 详情】")
            print(f"    共 {len(paths)} 条路径")
            if paths:
                first = paths[0]
                print(f"    示例: {json.dumps(first, ensure_ascii=False)[:200]}")
        
        # 详细分析 grids
        if 'grids' in d and d['grids']:
            grids = d['grids']
            print(f"\n  【Grids 详情】")
            print(f"    共 {len(grids)} 个栅格")
            if grids:
                first = grids[0]
                if isinstance(first, dict):
                    print(f"    示例 keys: {list(first.keys())}")
    
    # Private 信息
    if 'private' in data:
        priv = data['private']
        print(f"\n【Private 内容】")
        for key in priv.keys():
            val = priv[key]
            if isinstance(val, list):
                print(f"  {key}: List[{len(val)}]")
            elif isinstance(val, dict):
                print(f"  {key}: Dict keys={list(val.keys())[:5]}")
            else:
                print(f"  {key}: {str(val)[:100]}")

def main():
    maps_dir = Path(__file__).parent / "maps"
    
    for map_dir in sorted(maps_dir.iterdir()):
        if map_dir.is_dir():
            json_files = list(map_dir.glob("*.json"))
            if json_files:
                analyze_map(json_files[0])

if __name__ == '__main__':
    main()
