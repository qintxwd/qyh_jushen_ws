#!/usr/bin/env python3
"""
Standard Robots Matrix 地图下载器
"""
import requests
import json
import os
from pathlib import Path

BASE = "http://192.168.71.50"

def get_map_list():
    """获取地图列表"""
    r = requests.get(f"{BASE}/api/v0/map", timeout=10)
    r.raise_for_status()
    return r.json()

def get_map_data(map_name):
    """获取地图 JSON 数据"""
    r = requests.get(f"{BASE}/api/v0/map/{map_name}/data", timeout=30)
    r.raise_for_status()
    return r.json()

def get_map_image(map_name, scale=1):
    """获取地图图片"""
    r = requests.get(f"{BASE}/api/v0/map/{map_name}/image?scale={scale}", timeout=30)
    r.raise_for_status()
    return r.content

def main():
    # 创建输出目录
    output_dir = Path(__file__).parent / "maps"
    output_dir.mkdir(exist_ok=True)
    
    print("=" * 70)
    print("Standard Robots Matrix 地图下载器")
    print("=" * 70)
    
    # 获取地图列表
    print("\n【获取地图列表】")
    map_info = get_map_list()
    maps = map_info.get('maps', [])
    
    print(f"  找到 {len(maps)} 个地图:")
    for m in maps:
        print(f"    - {m['name']} (md5: {m['md5'][:8]}..., 修改时间: {m['modify_time']})")
    
    # 选择第一个地图下载
    if not maps:
        print("  没有找到地图!")
        return
    
    # 下载所有地图
    for map_entry in maps:
        map_name = map_entry['name']
        map_dir = output_dir / map_name
        map_dir.mkdir(exist_ok=True)
        
        print(f"\n【下载地图: {map_name}】")
        
        # 1. 下载地图 JSON 数据
        try:
            print(f"  正在下载 JSON 数据...")
            map_data = get_map_data(map_name)
            
            json_file = map_dir / f"{map_name}.json"
            with open(json_file, 'w', encoding='utf-8') as f:
                json.dump(map_data, f, indent=2, ensure_ascii=False)
            print(f"  ✓ JSON 数据已保存: {json_file}")
            
            # 显示地图信息
            if isinstance(map_data, dict):
                print(f"    地图数据 keys: {list(map_data.keys())[:10]}")
                
                # 检查是否有栅格地图
                if 'grid' in map_data:
                    grid = map_data['grid']
                    if isinstance(grid, dict):
                        print(f"    栅格地图: {grid.get('width', '?')}x{grid.get('height', '?')}")
                        print(f"    分辨率: {grid.get('resolution', '?')}")
                        
                # 检查是否有路径点
                if 'poses' in map_data:
                    poses = map_data['poses']
                    if isinstance(poses, list):
                        print(f"    路径点数量: {len(poses)}")
                        
                # 检查是否有路径
                if 'paths' in map_data:
                    paths = map_data['paths']
                    if isinstance(paths, list):
                        print(f"    路径数量: {len(paths)}")
                        
        except Exception as e:
            print(f"  ✗ JSON 下载失败: {e}")
        
        # 2. 下载地图图片
        try:
            print(f"  正在下载地图图片...")
            img_data = get_map_image(map_name, scale=1)
            
            # 检查图片类型
            if img_data[:8] == b'\x89PNG\r\n\x1a\n':
                img_ext = 'png'
            elif img_data[:2] == b'\xff\xd8':
                img_ext = 'jpg'
            else:
                img_ext = 'bin'
            
            img_file = map_dir / f"{map_name}.{img_ext}"
            with open(img_file, 'wb') as f:
                f.write(img_data)
            print(f"  ✓ 图片已保存: {img_file} ({len(img_data)} bytes)")
            
        except Exception as e:
            print(f"  ✗ 图片下载失败: {e}")
    
    print("\n" + "=" * 70)
    print(f"地图已保存到: {output_dir}")
    print("=" * 70)

if __name__ == '__main__':
    main()
