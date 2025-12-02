#!/usr/bin/env python3
"""
测试不同 scale 下载的图片
"""
import requests
from PIL import Image
from io import BytesIO

BASE = "http://192.168.71.50"
map_name = "yuanchang"

scales = [1, 0.5, 0.25, 0.125]

print("测试不同 scale 参数的图片大小:")
print("=" * 50)

for scale in scales:
    try:
        url = f"{BASE}/api/v0/map/{map_name}/image?scale={scale}"
        r = requests.get(url, timeout=30)
        r.raise_for_status()
        
        img = Image.open(BytesIO(r.content))
        print(f"scale={scale}: {img.size[0]} x {img.size[1]} ({len(r.content)} bytes)")
    except Exception as e:
        print(f"scale={scale}: Error - {e}")
