#!/usr/bin/env python3
"""搜索 scale 使用"""
import re

with open('web_src/js/app.d3121814.js', encoding='utf-8', errors='ignore') as f:
    c = f.read()

# 搜索 scale 相关代码
patterns = [
    r'/image\?scale=[^"\']{0,50}',
    r'scale\s*[:=]\s*1/[^,;]{0,30}',
    r'1/t[^,;]{0,30}scale',
]

for p in patterns:
    matches = re.findall(p, c)
    if matches:
        print(f"\n【{p}】")
        for m in list(set(matches))[:5]:
            print(f"  {m}")
