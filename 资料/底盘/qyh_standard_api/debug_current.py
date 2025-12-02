#!/usr/bin/env python3
"""调试坐标转换"""
import json
from pathlib import Path

map_dir = Path(r'd:\work\yc\qyh_jushen_ws\qyh_standard_api\maps\yuanchang')
with open(map_dir / 'yuanchang.json', 'r', encoding='utf-8') as f:
    data = json.load(f)

meta = data['meta']
map_data = data['data']

print(f"图片尺寸: ({meta['size.x']}, {meta['size.y']})")
print(f"原点: ({meta['zero_offset.x']}, {meta['zero_offset.y']})")
print(f"分辨率: {meta['resolution']} mm/pixel")
print()

# 站点
stations = map_data.get('station', [])
print(f"站点数: {len(stations)}")
for s in stations:
    x_mm = s.get('pos.x', s.get('x', 0))
    y_mm = s.get('pos.y', s.get('y', 0))
    name = s.get('name', 'unknown')
    
    res = meta['resolution']
    origin_x = meta['zero_offset.x']
    origin_y = meta['zero_offset.y']
    
    px_x = origin_x + x_mm / res
    px_y = origin_y - y_mm / res
    
    in_bounds = 0 <= px_x < meta['size.x'] and 0 <= px_y < meta['size.y']
    status = "✓" if in_bounds else "✗ OUT OF BOUNDS"
    
    print(f"  {name}: ({x_mm}, {y_mm}) mm -> ({px_x:.1f}, {px_y:.1f}) px {status}")

print()

# 边
edges = map_data.get('edge', [])
print(f"边数: {len(edges)}")
for e in edges:
    sx, sy = e['sx'], e['sy']
    ex, ey = e['ex'], e['ey']
    
    res = meta['resolution']
    origin_x = meta['zero_offset.x']
    origin_y = meta['zero_offset.y']
    
    px_sx = origin_x + sx / res
    px_sy = origin_y - sy / res
    px_ex = origin_x + ex / res
    px_ey = origin_y - ey / res
    
    s_in = 0 <= px_sx < meta['size.x'] and 0 <= px_sy < meta['size.y']
    e_in = 0 <= px_ex < meta['size.x'] and 0 <= px_ey < meta['size.y']
    status = "✓" if (s_in and e_in) else "✗ OUT OF BOUNDS"
    
    print(f"  ({sx}, {sy}) -> ({ex}, {ey}) mm")
    print(f"    ({px_sx:.1f}, {px_sy:.1f}) -> ({px_ex:.1f}, {px_ey:.1f}) px {status}")
