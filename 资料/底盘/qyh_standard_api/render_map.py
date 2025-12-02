#!/usr/bin/env python3
"""
地图可视化工具
将地图 JSON 数据渲染为图片，包含：
- 底图 (PNG)
- 节点 (圆点)
- 边 (直线/贝塞尔曲线)
- 站点 (带方向的标记)
"""
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# Edge 类型
EDGE_TYPE_LINE = 1
EDGE_TYPE_ARC = 2
EDGE_TYPE_BEZIER = 3

# 颜色
COLOR_NODE = (0, 128, 255)  # 蓝色
COLOR_EDGE_LINE = (100, 100, 100)  # 灰色
COLOR_EDGE_BEZIER = (0, 200, 200)  # 青色
COLOR_STATION = (255, 100, 0)  # 橙色
COLOR_STATION_DIR = (255, 0, 0)  # 红色 (方向箭头)


def mm_to_pixel(x_mm, y_mm, meta, image_scale_factor=1):
    """将毫米坐标转换为像素坐标
    
    Args:
        x_mm: 世界坐标 x (毫米)
        y_mm: 世界坐标 y (毫米)
        meta: 地图元数据
        image_scale_factor: 图片缩放因子 (1=原始大小, 2=缩小2倍, etc.)
                           下载时用 scale=1/t 参数，t 就是 image_scale_factor
    
    Note:
        resolution 字段的单位是 cm/pixel (厘米/像素)
        例如: resolution=2 表示 2 cm/pixel = 20 mm/pixel = 0.02 m/pixel
        坐标单位是 mm (毫米)，界面显示单位是 cm (厘米)
    """
    # resolution 的单位是 cm/pixel，需要转换为 mm/pixel
    resolution_cm_per_pixel = meta.get('resolution', 2)  # 单位: cm/pixel
    resolution_mm_per_pixel = resolution_cm_per_pixel * 10  # 转换为 mm/pixel
    
    # 实际分辨率 = 基础分辨率 * 图片缩放因子
    # 例如: base=20mm/pixel, scale_factor=2 -> actual=40mm/pixel
    actual_resolution = resolution_mm_per_pixel * image_scale_factor
    
    # 原点也需要根据缩放调整
    origin_x_px = meta.get('zero_offset.x', 0) / image_scale_factor
    origin_y_px = meta.get('zero_offset.y', 0) / image_scale_factor
    
    # 地图坐标系: 原点在 (zero_offset.x, zero_offset.y) 像素位置
    # x 向右为正，y 向上为正 (但图片 y 向下)
    px_x = origin_x_px + x_mm / actual_resolution
    px_y = origin_y_px - y_mm / actual_resolution  # 注意 y 轴翻转
    
    return px_x, px_y


def draw_bezier_curve(draw, p0, p1, p2, p3, color, width=2, steps=20):
    """绘制三次贝塞尔曲线"""
    points = []
    for i in range(steps + 1):
        t = i / steps
        # 三次贝塞尔公式
        x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + 3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
        y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + 3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
        points.append((x, y))
    
    # 绘制线段
    for i in range(len(points) - 1):
        draw.line([points[i], points[i+1]], fill=color, width=width)


def draw_arrow(draw, x, y, angle_rad, length=15, color=(255, 0, 0), width=2):
    """绘制方向箭头"""
    # angle_rad 是从 x 轴正方向逆时针的角度
    # 但因为 y 轴翻转，需要调整
    
    # 箭头终点
    end_x = x + length * math.cos(angle_rad)
    end_y = y - length * math.sin(angle_rad)  # y 轴翻转
    
    # 画主线
    draw.line([(x, y), (end_x, end_y)], fill=color, width=width)
    
    # 画箭头
    arrow_angle = 2.5  # 约 150 度
    arrow_len = 5
    
    left_angle = angle_rad + arrow_angle
    right_angle = angle_rad - arrow_angle
    
    left_x = end_x - arrow_len * math.cos(left_angle)
    left_y = end_y + arrow_len * math.sin(left_angle)
    right_x = end_x - arrow_len * math.cos(right_angle)
    right_y = end_y + arrow_len * math.sin(right_angle)
    
    draw.line([(end_x, end_y), (left_x, left_y)], fill=color, width=width)
    draw.line([(end_x, end_y), (right_x, right_y)], fill=color, width=width)


def render_map(map_name, output_path=None, image_scale_factor=None,
               auto_expand=True):
    """渲染地图
    
    Args:
        map_name: 地图名称
        output_path: 输出路径 (可选)
        image_scale_factor: 图片缩放因子，如果为 None 则自动检测
        auto_expand: 是否自动扩展画布以包含所有元素
    """
    maps_dir = Path(__file__).parent / 'maps'
    map_dir = maps_dir / map_name
    
    # 加载 JSON 数据
    json_path = map_dir / f'{map_name}.json'
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    meta = data.get('meta', {})
    map_data = data.get('data', {})
    
    # 加载底图
    img_path = map_dir / f'{map_name}.png'
    if img_path.exists():
        base_img = Image.open(img_path).convert('RGBA')
    else:
        # 创建空白图
        width = meta.get('size.x', 1000)
        height = meta.get('size.y', 1000)
        base_img = Image.new('RGBA', (width, height), (255, 255, 255, 255))
    
    # 自动检测图片缩放因子
    if image_scale_factor is None:
        # 比较实际图片尺寸与元数据中的尺寸
        actual_width, actual_height = base_img.size
        meta_width = meta.get('size.x', actual_width)
        meta_height = meta.get('size.y', actual_height)
        
        # 计算缩放因子
        scale_x = meta_width / actual_width if actual_width > 0 else 1
        scale_y = meta_height / actual_height if actual_height > 0 else 1
        
        # 取两者的平均值，并四舍五入到最近的整数
        image_scale_factor = round((scale_x + scale_y) / 2)
        if image_scale_factor < 1:
            image_scale_factor = 1
    
    base_res = meta.get('resolution', 2)  # 单位: cm/pixel
    base_res_mm = base_res * 10  # 转换为 mm/pixel
    actual_res_mm = base_res_mm * image_scale_factor
    
    # 计算所有元素的边界
    if auto_expand:
        all_x_mm = []
        all_y_mm = []
        
        # 收集所有节点坐标
        for n in map_data.get('node', []):
            all_x_mm.append(n['x'])
            all_y_mm.append(n['y'])
        
        # 收集所有边坐标
        for e in map_data.get('edge', []):
            all_x_mm.extend([e['sx'], e['ex']])
            all_y_mm.extend([e['sy'], e['ey']])
            # 贝塞尔控制点
            if e.get('type') == EDGE_TYPE_BEZIER:
                if e.get('cx', 0) != 0 or e.get('cy', 0) != 0:
                    all_x_mm.append(e['cx'])
                    all_y_mm.append(e['cy'])
                if e.get('dx', 0) != 0 or e.get('dy', 0) != 0:
                    all_x_mm.append(e['dx'])
                    all_y_mm.append(e['dy'])
        
        # 收集所有站点坐标
        for s in map_data.get('station', []):
            all_x_mm.append(s.get('pos.x', s.get('x', 0)))
            all_y_mm.append(s.get('pos.y', s.get('y', 0)))
        
        if all_x_mm and all_y_mm:
            # 计算需要的画布扩展量
            origin_x_px = meta.get('zero_offset.x', 0) / image_scale_factor
            origin_y_px = meta.get('zero_offset.y', 0) / image_scale_factor
            
            min_x_mm, max_x_mm = min(all_x_mm), max(all_x_mm)
            min_y_mm, max_y_mm = min(all_y_mm), max(all_y_mm)
            
            # 转换为像素
            min_px_x = origin_x_px + min_x_mm / actual_res_mm
            max_px_x = origin_x_px + max_x_mm / actual_res_mm
            min_px_y = origin_y_px - max_y_mm / actual_res_mm  # y 翻转
            max_px_y = origin_y_px - min_y_mm / actual_res_mm
            
            # 添加边距
            margin = 50
            
            # 计算需要扩展的像素数
            left_expand = max(0, margin - min_px_x)
            right_expand = max(0, (max_px_x + margin) - base_img.width)
            top_expand = max(0, margin - min_px_y)
            bottom_expand = max(0, (max_px_y + margin) - base_img.height)
            
            if left_expand > 0 or right_expand > 0 or top_expand > 0 or bottom_expand > 0:
                # 创建扩展后的画布
                new_width = int(base_img.width + left_expand + right_expand)
                new_height = int(base_img.height + top_expand + bottom_expand)
                
                # 用灰色背景填充扩展区域
                expanded_img = Image.new('RGBA', (new_width, new_height), 
                                         (200, 200, 200, 255))
                
                # 将原图粘贴到正确位置
                expanded_img.paste(base_img, (int(left_expand), int(top_expand)))
                
                # 更新原点位置
                origin_x_px += left_expand
                origin_y_px += top_expand
                
                # 更新 meta 中的原点（用于后续坐标转换）
                meta = dict(meta)  # 复制以避免修改原始数据
                meta['zero_offset.x'] = origin_x_px * image_scale_factor
                meta['zero_offset.y'] = origin_y_px * image_scale_factor
                
                base_img = expanded_img
                
                print(f"  画布已扩展: +{left_expand:.0f}左, +{right_expand:.0f}右, "
                      f"+{top_expand:.0f}上, +{bottom_expand:.0f}下")
    
    img = base_img.copy()
    
    # 创建绘图对象
    draw = ImageDraw.Draw(img)
    
    print(f"\n渲染地图: {map_name}")
    print(f"  图片尺寸: {img.size}")
    print(f"  元数据尺寸: ({meta.get('size.x', 0)}, {meta.get('size.y', 0)})")
    print(f"  基础分辨率: {base_res} cm/pixel = {base_res_mm} mm/pixel")
    print(f"  图片缩放因子: {image_scale_factor}")
    print(f"  实际分辨率: {actual_res_mm} mm/pixel")
    ox = meta.get('zero_offset.x', 0)
    oy = meta.get('zero_offset.y', 0)
    print(f"  原点 (原始): ({ox}, {oy})")
    print(f"  原点 (缩放后): ({ox/image_scale_factor:.1f}, "
          f"{oy/image_scale_factor:.1f})")
    
    # 1. 绘制边
    edges = map_data.get('edge', [])
    print(f"  绘制 {len(edges)} 条边...")
    
    line_count = 0
    bezier_count = 0
    
    for e in edges:
        edge_type = e.get('type', 1)
        
        # 起点和终点
        sx, sy = mm_to_pixel(e['sx'], e['sy'], meta, image_scale_factor)
        ex, ey = mm_to_pixel(e['ex'], e['ey'], meta, image_scale_factor)
        
        if edge_type == EDGE_TYPE_BEZIER:
            # 贝塞尔曲线
            cx, cy = mm_to_pixel(e.get('cx', 0), e.get('cy', 0), meta, image_scale_factor)
            dx, dy = mm_to_pixel(e.get('dx', 0), e.get('dy', 0), meta, image_scale_factor)
            
            # 如果控制点是 0,0，可能需要特殊处理
            if e.get('cx', 0) == 0 and e.get('cy', 0) == 0:
                # 使用起点作为第一个控制点
                cx, cy = sx, sy
            if e.get('dx', 0) == 0 and e.get('dy', 0) == 0:
                # 使用终点作为第二个控制点
                dx, dy = ex, ey
            
            draw_bezier_curve(draw, (sx, sy), (cx, cy), (dx, dy), (ex, ey),
                              COLOR_EDGE_BEZIER, width=2)
            bezier_count += 1
        else:
            # 直线
            draw.line([(sx, sy), (ex, ey)], fill=COLOR_EDGE_LINE, width=2)
            line_count += 1
    
    print(f"    直线: {line_count}, 贝塞尔曲线: {bezier_count}")
    
    # 2. 绘制节点
    nodes = map_data.get('node', [])
    print(f"  绘制 {len(nodes)} 个节点...")
    
    for n in nodes:
        x, y = mm_to_pixel(n['x'], n['y'], meta, image_scale_factor)
        r = 4  # 半径
        draw.ellipse([x-r, y-r, x+r, y+r], fill=COLOR_NODE)
    
    # 3. 绘制站点 (带方向)
    stations = map_data.get('station', [])
    print(f"  绘制 {len(stations)} 个站点...")
    
    for s in stations:
        # 位置可能用不同的 key
        pos_x = s.get('pos.x', s.get('x', 0))
        pos_y = s.get('pos.y', s.get('y', 0))
        pos_yaw = s.get('pos.yaw', s.get('yaw', 0))
        
        x, y = mm_to_pixel(pos_x, pos_y, meta, image_scale_factor)
        
        # 转换角度 (毫弧度 -> 弧度)
        yaw_rad = pos_yaw / 1000.0
        
        # 绘制站点圆圈
        r = 6
        draw.ellipse([x-r, y-r, x+r, y+r], fill=COLOR_STATION, outline=(0, 0, 0))
        
        # 绘制方向箭头
        draw_arrow(draw, x, y, yaw_rad, length=15, color=COLOR_STATION_DIR, width=2)
    
    # 保存结果
    if output_path is None:
        output_path = map_dir / f'{map_name}_rendered.png'
    
    img.save(output_path)
    print(f"  已保存: {output_path}")
    
    return img


def main():
    # 渲染所有地图
    maps_dir = Path(__file__).parent / 'maps'
    
    for map_dir in sorted(maps_dir.iterdir()):
        if map_dir.is_dir():
            map_name = map_dir.name
            try:
                render_map(map_name)
            except Exception as e:
                print(f"  渲染 {map_name} 失败: {e}")


if __name__ == '__main__':
    main()
