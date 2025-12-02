#!/usr/bin/env python3
"""
下载 Standard Robots Matrix 网站登录后的资源
模拟登录后下载 homePage 相关资源

由于这是 SPA (单页应用)，前端代码实际上在登录前后是一样的，
但我们需要分析登录过程中的 API 调用和 WebSocket 消息。
"""

import requests
import os
import re
import yaml
import json
import hashlib
import base64
from pathlib import Path
from urllib.parse import urljoin, urlparse

def load_config():
    """加载配置"""
    config_path = Path(__file__).parent / 'config.yaml'
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return {'matrix': {'host': '192.168.71.50', 'port': 80}}

def download_file(session, url, filepath, timeout=30):
    """下载文件"""
    try:
        r = session.get(url, timeout=timeout)
        r.raise_for_status()
        
        # 确保目录存在
        dir_path = os.path.dirname(filepath)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        
        # 保存文件
        with open(filepath, 'wb') as f:
            f.write(r.content)
        
        print(f"  ✓ {filepath} ({len(r.content)} bytes)")
        return r.content.decode('utf-8', errors='ignore')
    except Exception as e:
        print(f"  ✗ {filepath} 失败: {e}")
        return None

def explore_api(session, base_url, output_dir):
    """探索 API 端点"""
    print("\n=== 探索 API 端点 ===")
    
    # 各种可能的 API 端点
    api_endpoints = [
        # 地图相关
        '/api/v0/map',
        '/api/v0/map/current',
        '/api/v0/map/list',
        '/api/v0/maps',
        '/api/v0/current_map',
        '/api/v0/currentmap',
        
        # 系统/机器人状态
        '/api/v0/system',
        '/api/v0/system/state',
        '/api/v0/system/status',
        '/api/v0/system/info',
        '/api/v0/robot',
        '/api/v0/robot/state',
        '/api/v0/robot/status',
        '/api/v0/robot/info',
        '/api/v0/status',
        '/api/v0/state',
        '/api/v0/info',
        
        # 任务/任务相关
        '/api/v0/mission',
        '/api/v0/missions',
        '/api/v0/task',
        '/api/v0/tasks',
        
        # 账户相关
        '/api/v0/account',
        '/api/v0/accounts',
        '/api/v0/user',
        '/api/v0/users',
        
        # 配置相关
        '/api/v0/config',
        '/api/v0/configs',
        '/api/v0/settings',
        '/api/v0/param',
        '/api/v0/params',
        
        # 其他
        '/api/v0/version',
        '/api/v0/navigation',
        '/api/v0/position',
        '/api/v0/pose',
        '/api/v0/location',
    ]
    
    results = {}
    api_dir = os.path.join(output_dir, '_api_results')
    os.makedirs(api_dir, exist_ok=True)
    
    for endpoint in api_endpoints:
        try:
            r = session.get(f"{base_url}{endpoint}", timeout=5)
            results[endpoint] = {
                'status': r.status_code,
                'content_type': r.headers.get('content-type', ''),
                'length': len(r.content)
            }
            
            if r.status_code == 200:
                # 保存响应内容
                filename = endpoint.replace('/', '_').strip('_') + '.json'
                filepath = os.path.join(api_dir, filename)
                with open(filepath, 'wb') as f:
                    f.write(r.content)
                print(f"  ✓ {endpoint} -> {filename} ({len(r.content)} bytes)")
                
                # 尝试解析 JSON
                try:
                    data = r.json()
                    results[endpoint]['data'] = data
                except:
                    results[endpoint]['data'] = r.text[:200]
            else:
                print(f"  - {endpoint} -> {r.status_code}")
        except Exception as e:
            results[endpoint] = {'error': str(e)}
            print(f"  ✗ {endpoint} -> {e}")
    
    # 保存汇总
    summary_path = os.path.join(api_dir, '_summary.json')
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False, default=str)
    print(f"\n  API 探索汇总保存到: {summary_path}")
    
    return results

def download_static_resources(session, base_url, html_content, output_dir):
    """下载静态资源"""
    print("\n=== 下载静态资源 ===")
    
    # 找 JS 文件
    js_pattern = r'src=["\']?([^"\'>\s]+\.js)["\']?'
    js_files = list(set(re.findall(js_pattern, html_content)))
    
    # 找 CSS 文件
    css_pattern = r'href=["\']?([^"\'>\s]+\.css)["\']?'
    css_files = list(set(re.findall(css_pattern, html_content)))
    
    # 找图片和 favicon
    img_pattern = r'(?:href|src)=["\']?([^"\'>\s]+\.(ico|png|jpg|jpeg|svg|gif))["\']?'
    images = list(set([m[0] for m in re.findall(img_pattern, html_content)]))
    
    # 找字体
    font_pattern = r'(?:href|src)=["\']?([^"\'>\s]+\.(woff|woff2|ttf|eot|otf))["\']?'
    fonts = list(set([m[0] for m in re.findall(font_pattern, html_content)]))
    
    print(f"  找到 {len(js_files)} 个 JS 文件")
    print(f"  找到 {len(css_files)} 个 CSS 文件")
    print(f"  找到 {len(images)} 个图片")
    print(f"  找到 {len(fonts)} 个字体")
    
    # 下载 JS
    print("\n下载 JS 文件...")
    for js in js_files:
        js_path = js.lstrip('/')
        url = urljoin(base_url + '/', js_path)
        filepath = os.path.join(output_dir, js_path)
        download_file(session, url, filepath)
    
    # 下载 CSS
    print("\n下载 CSS 文件...")
    for css in css_files:
        css_path = css.lstrip('/')
        url = urljoin(base_url + '/', css_path)
        filepath = os.path.join(output_dir, css_path)
        content = download_file(session, url, filepath)
        
        # 从 CSS 中提取更多资源链接
        if content:
            # 找 CSS 中引用的图片和字体
            url_pattern = r'url\(["\']?([^"\')\s]+)["\']?\)'
            css_urls = re.findall(url_pattern, content)
            for css_url in css_urls:
                if not css_url.startswith('data:'):
                    # 解析相对路径
                    css_dir = os.path.dirname(css_path)
                    if css_url.startswith('/'):
                        res_path = css_url.lstrip('/')
                    else:
                        res_path = os.path.normpath(os.path.join(css_dir, css_url)).replace('\\', '/')
                    
                    res_url = urljoin(base_url + '/', res_path)
                    res_filepath = os.path.join(output_dir, res_path)
                    download_file(session, res_url, res_filepath)
    
    # 下载图片
    print("\n下载图片...")
    for img in images:
        img_path = img.lstrip('/')
        url = urljoin(base_url + '/', img_path)
        filepath = os.path.join(output_dir, img_path)
        download_file(session, url, filepath)
    
    # 下载字体
    print("\n下载字体...")
    for font in fonts:
        font_path = font.lstrip('/')
        url = urljoin(base_url + '/', font_path)
        filepath = os.path.join(output_dir, font_path)
        download_file(session, url, filepath)

def download_chunk_files(session, base_url, output_dir):
    """下载 webpack chunk 文件"""
    print("\n=== 下载 Chunk 文件 ===")
    
    # 基于观察到的 chunk 模式，尝试下载更多 chunk
    # 从 JS 代码中我们可以看到 chunk 编号的映射
    js_dir = os.path.join(output_dir, 'js')
    css_dir = os.path.join(output_dir, 'css')
    os.makedirs(js_dir, exist_ok=True)
    os.makedirs(css_dir, exist_ok=True)
    
    # 尝试下载编号为 0-40 的 chunk
    for i in range(41):
        # JS chunks
        for pattern in [f'js/{i}.*.js', f'js/chunk-{i}.*.js']:
            # 尝试常见的 hash 长度
            url = f"{base_url}/js/{i}.js"
            try:
                r = session.get(url, timeout=3)
                if r.status_code == 200:
                    filepath = os.path.join(js_dir, f'{i}.js')
                    with open(filepath, 'wb') as f:
                        f.write(r.content)
                    print(f"  ✓ js/{i}.js ({len(r.content)} bytes)")
            except:
                pass
        
        # CSS chunks
        url = f"{base_url}/css/{i}.css"
        try:
            r = session.get(url, timeout=3)
            if r.status_code == 200:
                filepath = os.path.join(css_dir, f'{i}.css')
                with open(filepath, 'wb') as f:
                    f.write(r.content)
                print(f"  ✓ css/{i}.css ({len(r.content)} bytes)")
        except:
            pass

def analyze_js_for_api(output_dir):
    """分析 JS 文件中的 API 调用"""
    print("\n=== 分析 JS 中的 API 调用 ===")
    
    js_dir = os.path.join(output_dir, 'js')
    if not os.path.exists(js_dir):
        print("  没有找到 JS 目录")
        return
    
    api_patterns = [
        r'/api/v\d+/[a-zA-Z0-9/_-]+',
        r'mapName',
        r'systemState',
        r'currentMap',
        r'displayedMapName',
    ]
    
    findings = {}
    
    for js_file in os.listdir(js_dir):
        if js_file.endswith('.js'):
            filepath = os.path.join(js_dir, js_file)
            try:
                with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                
                file_findings = []
                for pattern in api_patterns:
                    matches = re.findall(pattern, content)
                    if matches:
                        file_findings.extend(list(set(matches)))
                
                if file_findings:
                    findings[js_file] = list(set(file_findings))
            except Exception as e:
                print(f"  分析 {js_file} 失败: {e}")
    
    # 保存分析结果
    analysis_path = os.path.join(output_dir, '_js_api_analysis.json')
    with open(analysis_path, 'w', encoding='utf-8') as f:
        json.dump(findings, f, indent=2, ensure_ascii=False)
    print(f"  JS API 分析保存到: {analysis_path}")
    
    # 打印一些关键发现
    all_apis = set()
    for apis in findings.values():
        for api in apis:
            if api.startswith('/api/'):
                all_apis.add(api)
    
    if all_apis:
        print(f"\n  发现的 API 端点:")
        for api in sorted(all_apis):
            print(f"    {api}")

def main():
    config = load_config()
    host = config['matrix']['host']
    port = config['matrix'].get('port', 80)
    base_url = f"http://{host}:{port}" if port != 80 else f"http://{host}"
    
    output_dir = Path(__file__).parent / 'web_src_home'
    output_dir.mkdir(exist_ok=True)
    
    print("=" * 70)
    print("下载 Matrix 网站资源 (登录后页面)")
    print("=" * 70)
    print(f"目标: {base_url}")
    print(f"保存到: {output_dir}")
    print()
    
    # 创建会话
    session = requests.Session()
    
    # 1. 下载主页 (index.html)
    print("1. 下载主页...")
    html_content = download_file(session, base_url, str(output_dir / 'index.html'))
    if not html_content:
        print("无法下载主页，退出")
        return
    
    # 2. 下载静态资源
    download_static_resources(session, base_url, html_content, str(output_dir))
    
    # 3. 尝试下载更多 chunk 文件
    download_chunk_files(session, base_url, str(output_dir))
    
    # 4. 探索 API 端点
    api_results = explore_api(session, base_url, str(output_dir))
    
    # 5. 分析 JS 中的 API
    analyze_js_for_api(str(output_dir))
    
    # 6. 下载一些特定的静态资源
    print("\n=== 下载其他资源 ===")
    other_resources = [
        '/static/source/companyInformation/companyInformation.json',
        '/manifest.json',
        '/favicon.ico',
    ]
    
    for res in other_resources:
        url = base_url + res
        filepath = str(output_dir) + res.replace('/', os.sep)
        download_file(session, url, filepath)
    
    print("\n" + "=" * 70)
    print("下载完成！")
    print("=" * 70)
    
    # 打印文件统计
    total_files = 0
    total_size = 0
    for root, dirs, files in os.walk(output_dir):
        for f in files:
            filepath = os.path.join(root, f)
            total_files += 1
            total_size += os.path.getsize(filepath)
    
    print(f"总共下载: {total_files} 个文件, {total_size / 1024:.1f} KB")
    print(f"保存位置: {output_dir}")
    
    # 特别关注 mapName 相关信息
    print("\n" + "=" * 70)
    print("关于获取当前地图名的分析")
    print("=" * 70)
    print("""
根据 JS 代码分析:
1. 当前地图名存储在 systemState.mapName 字段中
2. 这个字段通过 WebSocket 接收的 SystemState protobuf 消息更新
3. 显示逻辑: displayedMapName = displayedMapName || systemState.mapName

获取方式:
- WebSocket 连接到 ws://{host}:5002
- 发送登录认证消息 (Protobuf 格式)
- 接收 SystemState 消息，解析 mapName 字段

HTTP API 方式 (如果可用):
- /api/v0/map 只返回地图列表，不是当前地图
- 其他 API 需要进一步探索
""".format(host=host))

if __name__ == '__main__':
    main()
