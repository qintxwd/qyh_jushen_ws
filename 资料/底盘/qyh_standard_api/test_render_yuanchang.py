#!/usr/bin/env python3
"""测试渲染 yuanchang 地图"""
import sys
sys.path.insert(0, r'd:\work\yc\qyh_jushen_ws\qyh_standard_api')

from render_map import render_map
from pathlib import Path

if __name__ == '__main__':
    # 删除旧文件
    out_path = Path(r'd:\work\yc\qyh_jushen_ws\qyh_standard_api\maps\yuanchang\yuanchang_rendered.png')
    if out_path.exists():
        out_path.unlink()
        print(f"已删除旧文件: {out_path}")
    
    # 重新渲染（使用最大缓冲）
    import io
    import sys
    
    # 重新渲染
    img = render_map('yuanchang')
    
    print(f"\n最终图片尺寸: {img.size}")
    print("\n完成!")
