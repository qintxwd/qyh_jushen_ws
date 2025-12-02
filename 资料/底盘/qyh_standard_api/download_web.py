#!/usr/bin/env python3
"""
下载 Standard Robots Matrix 网站前端资源
用于分析 API 接口
"""

import requests
import os
import re
import yaml
from pathlib import Path

def load_config():
    """加载配置"""
    config_path = Path(__file__).parent / 'config.yaml'
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return {'matrix': {'host': '192.168.71.50', 'port': 80}}

def download_file(url, filepath, timeout=30):
    """下载文件"""
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        
        # 确保目录存在
        os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.', exist_ok=True)
        
        # 保存文件
        with open(filepath, 'wb') as f:
            f.write(r.content)
        
        print(f"  ✓ {filepath} ({len(r.content)} bytes)")
        return r.content.decode('utf-8', errors='ignore')
    except Exception as e:
        print(f"  ✗ {filepath} 失败: {e}")
        return None

def main():
    config = load_config()
    host = config['matrix']['host']
    port = config['matrix'].get('port', 80)
    base_url = f"http://{host}:{port}" if port != 80 else f"http://{host}"
    
    web_src = Path(__file__).parent / 'web_src'
    web_src.mkdir(exist_ok=True)
    os.chdir(web_src)
    
    print(f"=== 下载 Matrix 网站资源 ===")
    print(f"目标: {base_url}")
    print(f"保存到: {web_src}")
    print()
    
    # 1. 下载主页
    print("1. 下载主页...")
    html = download_file(base_url, 'index.html')
    if not html:
        print("无法下载主页，退出")
        return
    
    # 2. 解析资源链接 (支持有引号和无引号的格式)
    print("\n2. 解析资源链接...")
    
    # 找 JS 文件
    js_pattern = r'src=["\']?([^"\'>\s]+\.js)["\']?'
    js_files = re.findall(js_pattern, html)
    
    # 找 CSS 文件
    css_pattern = r'href=["\']?([^"\'>\s]+\.css)["\']?'
    css_files = re.findall(css_pattern, html)
    
    # 找 favicon 和图片
    img_pattern = r'href=["\']?([^"\'>\s]+\.(ico|png|jpg|svg))["\']?'
    images = [m[0] for m in re.findall(img_pattern, html)]
    
    # 找 fonts
    fonts_pattern = r'href=["\']?([^"\'>\s]*fonts[^"\'>\s]*)["\']?'
    fonts = re.findall(fonts_pattern, html)
    
    print(f"   找到 {len(js_files)} 个 JS 文件")
    print(f"   找到 {len(css_files)} 个 CSS 文件")
    print(f"   找到 {len(images)} 个图片/favicon")
    print(f"   找到 {len(fonts)} 个字体文件")
    
    # 3. 创建目录
    for d in ['js', 'css', 'fonts', 'img', 'static/source/companyInformation']:
        os.makedirs(d, exist_ok=True)
    
    # 4. 下载 JS 文件
    print("\n3. 下载 JS 文件...")
    for js in js_files:
        js_path = js.lstrip('/')
        url = f"{base_url}/{js_path}"
        download_file(url, js_path)
    
    # 5. 下载 CSS 文件
    print("\n4. 下载 CSS 文件...")
    for css in css_files:
        css_path = css.lstrip('/')
        url = f"{base_url}/{css_path}"
        download_file(url, css_path)
    
    # 6. 下载 favicon 和图片
    print("\n5. 下载其他资源...")
    for img in images:
        img_path = img.lstrip('/')
        url = f"{base_url}/{img_path}"
        download_file(url, img_path)
    
    # 7. 尝试下载一些常见的 API 文档路径
    print("\n6. 探索常见路径...")
    common_paths = [
        '/api',
        '/api/v1',
        '/swagger',
        '/docs',
        '/openapi.json',
        '/api-docs',
        '/manifest.json',
        '/config.js',
        '/env.js',
        '/settings.json',
    ]
    
    for path in common_paths:
        url = base_url + path
        filepath = path.lstrip('/').replace('/', '_') or 'root'
        try:
            r = requests.get(url, timeout=5)
            if r.status_code == 200 and len(r.content) > 0:
                with open(f'_api_explore_{filepath}.txt', 'wb') as f:
                    f.write(r.content)
                print(f"  ✓ {path} -> _api_explore_{filepath}.txt ({len(r.content)} bytes)")
        except:
            pass
    
    print("\n=== 下载完成 ===")
    print(f"文件保存在: {web_src}")
    
    # 列出下载的文件
    print("\n下载的文件:")
    for root, dirs, files in os.walk('.'):
        for f in files:
            filepath = os.path.join(root, f)
            size = os.path.getsize(filepath)
            print(f"  {filepath} ({size} bytes)")

if __name__ == '__main__':
    main()
