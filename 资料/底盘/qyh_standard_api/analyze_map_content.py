#!/usr/bin/env python3
"""
分析图片内容，找出正确的坐标系
"""
import json
from pathlib import Path
from PIL import Image
import numpy as np

# 加载数据
json_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.json'
img_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.png'

with open(json_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

meta = data['meta']
img = Image.open(img_path).convert('L')  # 转灰度
img_array = np.array(img)

img_w, img_h = img.size
print(f"图片尺寸: {img_w} x {img_h}")

# 找出地图内容的边界（非白色区域）
# 白色是 255，地图内容是深色
threshold = 250
non_white = np.where(img_array < threshold)

if len(non_white[0]) > 0:
    min_y = non_white[0].min()
    max_y = non_white[0].max()
    min_x = non_white[1].min()
    max_x = non_white[1].max()
    
    print(f"\n地图内容边界:")
    print(f"  X: {min_x} ~ {max_x} (宽度: {max_x - min_x})")
    print(f"  Y: {min_y} ~ {max_y} (高度: {max_y - min_y})")
    
    # 地图内容的中心
    center_x = (min_x + max_x) // 2
    center_y = (min_y + max_y) // 2
    print(f"  内容中心: ({center_x}, {center_y})")

# 元数据中的原点
origin_x = meta.get('zero_offset.x', 0)
origin_y = meta.get('zero_offset.y', 0)
print(f"\n元数据原点: ({origin_x}, {origin_y})")

# 站点信息
stations = data['data']['station']
print(f"\n站点世界坐标:")
for s in stations:
    print(f"  {s['name']}: ({s['pos.x']:.1f}, {s['pos.y']:.1f}) mm")

# 计算站点之间的距离
s1 = stations[0]
s2 = stations[1]
dx = s2['pos.x'] - s1['pos.x']
dy = s2['pos.y'] - s1['pos.y']
dist_mm = (dx**2 + dy**2)**0.5
print(f"\n站点间距离: {dist_mm:.1f} mm = {dist_mm/1000:.2f} m")

# 从图2估算：两站点在图片中大约相距多少像素
# 如果分辨率是 2mm/pixel，那么 5544mm ≈ 2772 pixels
res = meta.get('resolution', 2)
expected_dist_px = dist_mm / res
print(f"预期像素距离 (res={res}): {expected_dist_px:.1f} pixels")

# 图片宽度 3952，如果两点距离 2772 pixels，那么：
# 如果一个点在 x=1388，另一个点应该在 x=1388-2766=-1378 (确实超出范围)

# 或者分辨率理解错了？
# 假设两点都在图片内，估算分辨率
# 假设 s1 在图片中心 (1976, 1616)，s2 在左侧
# 那需要多大的分辨率才能让 s2 也在图片内？

# 站点2.x 需要 >= 0，即 origin_x + s2.x/res >= 0
# 2450 + (-7658)/res >= 0
# -7658/res >= -2450
# 7658/res <= 2450
# res >= 7658/2450 = 3.13

print(f"\n要让站点2在图片内，分辨率需要 >= {7658/2450:.2f} mm/pixel")
print(f"当前分辨率: {res} mm/pixel")

# 或者 zero_offset 的理解有问题
# 如果 zero_offset.x = 2450 是从图片右边算呢？
print(f"\n如果 zero_offset.x 是从右边算:")
alt_origin_x = img_w - origin_x  # 3952 - 2450 = 1502
print(f"  origin_x = {alt_origin_x}")
s2_x_alt = alt_origin_x + s2['pos.x'] / res
print(f"  站点2.x = {s2_x_alt:.1f}")
