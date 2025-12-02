#!/usr/bin/env python3
"""
修复坐标转换问题
分析 Matrix 地图的坐标系
"""
import json
from pathlib import Path
from PIL import Image, ImageDraw

# 加载数据
json_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.json'
img_path = Path(__file__).parent / 'maps/yuanchang/yuanchang.png'

with open(json_path, 'r', encoding='utf-8') as f:
    data = json.load(f)

meta = data['meta']
img = Image.open(img_path)

print("=" * 60)
print("坐标系分析")
print("=" * 60)

# 图片信息
img_w, img_h = img.size
print(f"实际图片尺寸: {img_w} x {img_h}")
print(f"Meta 中记录的尺寸: {meta.get('size.x')} x {meta.get('size.y')}")

# 原点位置
origin_x = meta.get('zero_offset.x', 0)  # 2450
origin_y = meta.get('zero_offset.y', 0)  # 1503
res = meta.get('resolution', 2)  # 2 mm/pixel

print(f"\n原点 (0,0) 在图片中的位置:")
print(f"  zero_offset.x = {origin_x} (从左边算)")
print(f"  zero_offset.y = {origin_y} (从上边算?)")

# 从图2看，原点在右下区域
# 如果 zero_offset.y 是从图片顶部算，那原点在 y=1503 的位置
# 图片高度 3232，所以原点在靠上的位置

# 尝试不同的坐标转换方式
stations = data['data']['station']
edges = data['data']['edge']

print("\n" + "=" * 60)
print("尝试不同的坐标转换:")
print("=" * 60)

s1 = stations[0]  # 站点1
s2 = stations[1]  # 站点2

print(f"\n站点1: 世界坐标 ({s1['pos.x']:.1f}, {s1['pos.y']:.1f}) mm")
print(f"站点2: 世界坐标 ({s2['pos.x']:.1f}, {s2['pos.y']:.1f}) mm")

# 方式1: 当前方式 (x 向右, y 向上但图片 y 向下)
def convert_v1(x_mm, y_mm):
    px = origin_x + x_mm / res
    py = origin_y - y_mm / res  # y 翻转
    return px, py

# 方式2: y 不翻转
def convert_v2(x_mm, y_mm):
    px = origin_x + x_mm / res
    py = origin_y + y_mm / res
    return px, py

# 方式3: zero_offset.y 是从底部算
def convert_v3(x_mm, y_mm):
    px = origin_x + x_mm / res
    py = (img_h - origin_y) - y_mm / res
    return px, py

# 方式4: 完全翻转
def convert_v4(x_mm, y_mm):
    px = origin_x + x_mm / res
    py = img_h - (origin_y + y_mm / res)
    return px, py

conversions = [
    ("V1: origin_y - y/res", convert_v1),
    ("V2: origin_y + y/res", convert_v2),
    ("V3: (h-origin_y) - y/res", convert_v3),
    ("V4: h - (origin_y + y/res)", convert_v4),
]

for name, conv in conversions:
    p1 = conv(s1['pos.x'], s1['pos.y'])
    p2 = conv(s2['pos.x'], s2['pos.y'])
    
    in_range_1 = 0 <= p1[0] < img_w and 0 <= p1[1] < img_h
    in_range_2 = 0 <= p2[0] < img_w and 0 <= p2[1] < img_h
    
    print(f"\n{name}:")
    print(f"  站点1: ({p1[0]:.0f}, {p1[1]:.0f}), 在范围内: {in_range_1}")
    print(f"  站点2: ({p2[0]:.0f}, {p2[1]:.0f}), 在范围内: {in_range_2}")
    print(f"  两点都在范围内: {in_range_1 and in_range_2}")

# 生成测试图片
print("\n" + "=" * 60)
print("生成测试图片 (每种转换方式一张)")
print("=" * 60)

output_dir = Path(__file__).parent / 'maps/yuanchang'

for i, (name, conv) in enumerate(conversions):
    test_img = img.copy().convert('RGBA')
    draw = ImageDraw.Draw(test_img)
    
    # 画原点
    draw.ellipse([origin_x-5, origin_y-5, origin_x+5, origin_y+5], fill=(0, 255, 0), outline=(0, 0, 0))
    
    # 画边
    for e in edges:
        sx_px, sy_px = conv(e['sx'], e['sy'])
        ex_px, ey_px = conv(e['ex'], e['ey'])
        
        # 裁剪到图片范围内
        if -1000 < sx_px < img_w + 1000 and -1000 < sy_px < img_h + 1000:
            draw.line([(sx_px, sy_px), (ex_px, ey_px)], fill=(0, 100, 255), width=3)
    
    # 画站点
    for s in stations:
        px, py = conv(s['pos.x'], s['pos.y'])
        if 0 <= px < img_w and 0 <= py < img_h:
            draw.ellipse([px-8, py-8, px+8, py+8], fill=(255, 100, 0), outline=(0, 0, 0))
    
    out_path = output_dir / f'test_conv_v{i+1}.png'
    test_img.save(out_path)
    print(f"  保存: {out_path.name}")
