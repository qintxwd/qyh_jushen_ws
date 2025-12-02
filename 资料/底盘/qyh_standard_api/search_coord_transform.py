#!/usr/bin/env python3
"""
搜索 JS 中的坐标转换代码
"""
import re

with open('web_src/js/app.d3121814.js', encoding='utf-8', errors='ignore') as f:
    c = f.read()

print("=" * 60)
print("搜索坐标转换相关代码")
print("=" * 60)

# 搜索坐标转换
patterns = [
    (r'zero_offset[^;]{0,100}', "zero_offset"),
    (r'resolution[^;]{0,100}', "resolution"),
    (r'worldToScreen[^}]{0,200}', "worldToScreen"),
    (r'screenToWorld[^}]{0,200}', "screenToWorld"),
    (r'toPixel[^}]{0,200}', "toPixel"),
    (r'toWorld[^}]{0,200}', "toWorld"),
    (r'convertCoord[^}]{0,200}', "convertCoord"),
    (r'\.x\s*/\s*\w+\.resolution', "x/resolution"),
]

for pattern, name in patterns:
    matches = re.findall(pattern, c)
    if matches:
        print(f"\n【{name}】({len(matches)} matches)")
        for m in list(set(matches))[:3]:
            print(f"  {m[:150]}")

# 特别搜索 station 绘制相关
print("\n" + "=" * 60)
print("搜索 station 绘制")
print("=" * 60)

station_patterns = [
    r'drawStation[^}]{0,300}',
    r'station\.pos\.x[^;]{0,50}',
    r'"pos\.x"[^;]{0,50}',
]

for p in station_patterns:
    matches = re.findall(p, c)
    if matches:
        print(f"\n【{p}】")
        for m in list(set(matches))[:3]:
            print(f"  {m[:150]}")
