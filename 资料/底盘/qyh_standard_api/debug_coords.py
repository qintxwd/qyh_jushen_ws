#!/usr/bin/env python3
"""分析 yuanchang 地图坐标"""
import json
from pathlib import Path

# 加载数据
json_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.json'
with open(json_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

meta = data['meta']
print("=" * 60)
print("Meta 信息:")
print("=" * 60)
print(f"  图片尺寸: {meta.get('size.x')} x {meta.get('size.y')} pixels")
print(f"  原点偏移: ({meta.get('zero_offset.x')}, {meta.get('zero_offset.y')}) pixels")
print(f"  分辨率: {meta.get('resolution')} mm/pixel")

# 计算原点在真实世界中的位置
res = meta.get('resolution', 2)
origin_px_x = meta.get('zero_offset.x', 0)
origin_px_y = meta.get('zero_offset.y', 0)
print(f"\n  原点(0,0)在图片中的位置: ({origin_px_x}, {origin_px_y}) pixels")

print("\n" + "=" * 60)
print("Station 信息:")
print("=" * 60)
stations = data['data']['station']
for s in stations:
    pos_x = s['pos.x']
    pos_y = s['pos.y']
    
    # 当前的坐标转换
    px_x_current = origin_px_x + pos_x / res
    px_y_current = origin_px_y - pos_y / res  # y 翻转
    
    print(f"  {s['name']}:")
    print(f"    世界坐标: ({pos_x:.1f}, {pos_y:.1f}) mm")
    print(f"    当前转换后像素: ({px_x_current:.1f}, {px_y_current:.1f})")

print("\n" + "=" * 60)
print("Edge 信息:")
print("=" * 60)
edges = data['data']['edge']
for e in edges:
    print(f"  Edge {e['id']}:")
    print(f"    起点: ({e['sx']:.1f}, {e['sy']:.1f}) mm")
    print(f"    终点: ({e['ex']:.1f}, {e['ey']:.1f}) mm")
    
    # 转换
    sx_px = origin_px_x + e['sx'] / res
    sy_px = origin_px_y - e['sy'] / res
    ex_px = origin_px_x + e['ex'] / res
    ey_px = origin_px_y - e['ey'] / res
    
    print(f"    起点像素: ({sx_px:.1f}, {sy_px:.1f})")
    print(f"    终点像素: ({ex_px:.1f}, {ey_px:.1f})")

# 检查图片大小
print("\n" + "=" * 60)
print("坐标范围检查:")
print("=" * 60)
img_w = meta.get('size.x')
img_h = meta.get('size.y')

for s in stations:
    px_x = origin_px_x + s['pos.x'] / res
    px_y = origin_px_y - s['pos.y'] / res
    in_range = 0 <= px_x < img_w and 0 <= px_y < img_h
    print(f"  {s['name']}: px=({px_x:.0f}, {px_y:.0f}), 在范围内: {in_range}")
