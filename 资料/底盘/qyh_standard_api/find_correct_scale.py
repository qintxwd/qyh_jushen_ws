#!/usr/bin/env python3
"""
使用图片实际内容反推坐标转换
"""
import json
from pathlib import Path
from PIL import Image, ImageDraw
import numpy as np

# 加载数据
json_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.json'
img_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.png'

with open(json_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

meta = data['meta']
img = Image.open(img_path).convert('RGBA')
img_w, img_h = img.size

stations = data['data']['station']
edges = data['data']['edge']

# 元数据
origin_x_px = meta.get('zero_offset.x', 0)  # 2450
origin_y_px = meta.get('zero_offset.y', 0)  # 1503
res = meta.get('resolution', 2)  # 2 mm/pixel

print("=" * 60)
print("坐标转换公式推导")
print("=" * 60)

# 从图2分析：
# - 原点 (0,0) 在图片右下区域
# - X 轴向右为正
# - Y 轴向上为正
# - 站点1 (n1, 站点1) 在原点左边偏下
# - 站点2 (n2, 站点2) 更靠左

# 世界坐标：
# 站点1: (-2125, -1773) - 原点左边 2125mm，原点下方 1773mm
# 站点2: (-7658, -1418) - 原点左边 7658mm，原点下方 1418mm

# 图片坐标系（左上角为原点，x向右，y向下）
# 如果世界 (0,0) 在图片 (origin_x_px, origin_y_px) = (2450, 1503)
# 
# 世界 x -> 图片 x: px_x = origin_x_px + world_x / res
# 世界 y -> 图片 y: 因为图片 y 向下，世界 y 向上
#                   py_y = origin_y_px - world_y / res

# 问题：当前转换让站点2超出范围
# 可能原因：
# 1. 分辨率不是 2，而是根据 image_scale 缩放了
# 2. zero_offset 的含义不同

print(f"\nimage_scale 列表: {meta.get('image_scale', [])}")

# 从 API 获取图片时，scale 参数是什么意思？
# /api/v0/map/{name}/image?scale=1
# 如果 image_scale = [1, 2, 4, 8]，可能表示可用的缩放级别

# 假设：下载的图片是 scale=1，但地图数据是按原始分辨率存储的
# 那实际分辨率 = resolution * scale

# 让我计算：要让两个站点都在图片内，需要什么分辨率
s1_x, s1_y = stations[0]['pos.x'], stations[0]['pos.y']
s2_x, s2_y = stations[1]['pos.x'], stations[1]['pos.y']

# 站点2 是最左边的点，x = -7658
# 要让 px_x >= 0: origin_x_px + s2_x / actual_res >= 0
# 2450 - 7658 / actual_res >= 0
# actual_res >= 7658 / 2450 = 3.13

# 站点1 是最右边的点，x = -2125
# 要让 px_x < img_w: origin_x_px + s1_x / actual_res < 3952
# 2450 - 2125 / actual_res < 3952 (这个总是满足的)

min_res = 7658 / origin_x_px
print(f"\n最小有效分辨率: {min_res:.2f} mm/pixel")

# 但从图2看，两个站点和路径都完整显示在图片中
# 而且从图2的比例看，分辨率应该更大

# 让我试试：如果分辨率是 4 mm/pixel (image_scale 中的 2?)
test_res = 4
print(f"\n测试分辨率 = {test_res} mm/pixel:")
s1_px_x = origin_x_px + s1_x / test_res
s2_px_x = origin_x_px + s2_x / test_res
print(f"  站点1 x: {s1_px_x:.1f}")
print(f"  站点2 x: {s2_px_x:.1f}")

# 还是不够，试试更大的
for test_res in [2, 3, 4, 5, 6, 8, 10]:
    s1_px = (origin_x_px + s1_x / test_res, origin_y_px - s1_y / test_res)
    s2_px = (origin_x_px + s2_x / test_res, origin_y_px - s2_y / test_res)
    
    in_range_1 = 0 <= s1_px[0] < img_w and 0 <= s1_px[1] < img_h
    in_range_2 = 0 <= s2_px[0] < img_w and 0 <= s2_px[1] < img_h
    
    if in_range_1 and in_range_2:
        print(f"\n✓ 分辨率 {test_res} mm/pixel 有效!")
        print(f"  站点1: ({s1_px[0]:.1f}, {s1_px[1]:.1f})")
        print(f"  站点2: ({s2_px[0]:.1f}, {s2_px[1]:.1f})")
        
        # 生成测试图
        test_img = img.copy()
        draw = ImageDraw.Draw(test_img)
        
        # 画原点
        draw.ellipse([origin_x_px-8, origin_y_px-8, origin_x_px+8, origin_y_px+8], 
                    fill=(0, 255, 0), outline=(0, 0, 0))
        
        # 画边
        for e in edges:
            sx = origin_x_px + e['sx'] / test_res
            sy = origin_y_px - e['sy'] / test_res
            ex = origin_x_px + e['ex'] / test_res
            ey = origin_y_px - e['ey'] / test_res
            draw.line([(sx, sy), (ex, ey)], fill=(0, 100, 255), width=3)
        
        # 画站点
        for s in stations:
            px = origin_x_px + s['pos.x'] / test_res
            py = origin_y_px - s['pos.y'] / test_res
            draw.ellipse([px-10, py-10, px+10, py+10], fill=(255, 100, 0), outline=(0, 0, 0))
        
        out_path = Path(__file__).parent / f'maps/yuanchang/test_res_{test_res}.png'
        test_img.save(out_path)
        print(f"  保存: {out_path}")
