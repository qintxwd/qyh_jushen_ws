#!/usr/bin/env python3
"""分析 edge 数据，找出曲线绘制规则"""
import json
from pathlib import Path

def analyze_edges(map_name):
    json_path = Path(__file__).parent / 'maps' / map_name / f'{map_name}.json'
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    edges = data.get('data', {}).get('edge', [])
    print(f"\n{'='*70}")
    print(f"地图: {map_name} - Edge 分析")
    print(f"{'='*70}")
    print(f"总边数: {len(edges)}")
    
    # 分类统计
    straight_edges = []  # cx=cy=dx=dy=0 的直线
    curved_edges = []    # 有控制点的曲线
    arc_edges = []       # 有 radius 的圆弧
    
    for e in edges:
        cx, cy = e.get('cx', 0), e.get('cy', 0)
        dx, dy = e.get('dx', 0), e.get('dy', 0)
        radius = e.get('radius', 0)
        
        if radius != 0:
            arc_edges.append(e)
        elif cx != 0 or cy != 0 or dx != 0 or dy != 0:
            curved_edges.append(e)
        else:
            straight_edges.append(e)
    
    print(f"\n直线边: {len(straight_edges)}")
    print(f"曲线边 (有控制点): {len(curved_edges)}")
    print(f"圆弧边 (有radius): {len(arc_edges)}")
    
    # 显示曲线边示例
    if curved_edges:
        print(f"\n【曲线边示例】")
        for e in curved_edges[:3]:
            print(f"  Edge {e['id']}: ({e['sx']:.1f},{e['sy']:.1f}) -> ({e['ex']:.1f},{e['ey']:.1f})")
            print(f"    控制点1: ({e['cx']:.1f}, {e['cy']:.1f})")
            print(f"    控制点2: ({e['dx']:.1f}, {e['dy']:.1f})")
            print(f"    type={e.get('type', '?')}, direction={e.get('direction', '?')}")
    
    # 显示圆弧边示例
    if arc_edges:
        print(f"\n【圆弧边示例】")
        for e in arc_edges[:3]:
            print(f"  Edge {e['id']}: ({e['sx']:.1f},{e['sy']:.1f}) -> ({e['ex']:.1f},{e['ey']:.1f})")
            print(f"    radius={e['radius']:.1f}")
            print(f"    rotate_direction={e.get('rotate_direction', '?')}")
    
    # 分析 edge type
    print(f"\n【Edge Type 统计】")
    type_counts = {}
    for e in edges:
        t = e.get('type', 'unknown')
        type_counts[t] = type_counts.get(t, 0) + 1
    for t, count in sorted(type_counts.items()):
        print(f"  type={t}: {count} 条")
    
    # 分析站点方向
    stations = data.get('data', {}).get('station', [])
    print(f"\n【站点分析】")
    print(f"总站点数: {len(stations)}")
    if stations:
        s = stations[0]
        print(f"站点字段: {list(s.keys())}")
        print(f"\n站点示例:")
        for s in stations[:3]:
            pos_x = s.get('pos.x', s.get('x', 0))
            pos_y = s.get('pos.y', s.get('y', 0))
            pos_yaw = s.get('pos.yaw', s.get('yaw', 0))
            name = s.get('name', f"站点{s.get('id', '?')}")
            print(f"  {name}: pos=({pos_x:.1f}, {pos_y:.1f}), yaw={pos_yaw:.3f} rad ({pos_yaw*180/3.14159:.1f}°)")

# 分析两个地图
analyze_edges('yuanchang')
analyze_edges('standard')
