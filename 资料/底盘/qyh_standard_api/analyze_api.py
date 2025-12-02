#!/usr/bin/env python3
"""
分析 Standard Robots Matrix 前端代码，提取 API 接口
"""

import re
import json
from pathlib import Path

def analyze_js_file(filepath):
    """分析 JS 文件，提取 API 相关信息"""
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    results = {
        'api_endpoints': set(),
        'login_related': [],
        'map_related': [],
        'axios_calls': [],
        'fetch_calls': [],
    }
    
    # 1. 查找 API 端点
    # 匹配 "/api/xxx" 或 "api/xxx" 格式
    api_patterns = [
        r'["\'](/api/[^"\']+)["\']',
        r'["\']api/([^"\']+)["\']',
        r'url:\s*["\']([^"\']*api[^"\']*)["\']',
        r'baseURL:\s*["\']([^"\']+)["\']',
    ]
    
    for pattern in api_patterns:
        matches = re.findall(pattern, content)
        for m in matches:
            results['api_endpoints'].add(m)
    
    # 2. 查找登录相关代码
    login_patterns = [
        r'.{0,100}login.{0,100}',
        r'.{0,100}Login.{0,100}',
        r'.{0,100}auth.{0,100}',
        r'.{0,100}token.{0,100}',
        r'.{0,100}password.{0,100}',
    ]
    
    for pattern in login_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        for m in matches[:5]:  # 只取前5个
            if 'api' in m.lower() or 'url' in m.lower() or 'post' in m.lower():
                results['login_related'].append(m.strip())
    
    # 3. 查找地图相关代码
    map_patterns = [
        r'.{0,100}[Mm]ap.{0,100}',
        r'.{0,100}mapList.{0,100}',
        r'.{0,100}getMap.{0,100}',
        r'.{0,100}currentMap.{0,100}',
    ]
    
    for pattern in map_patterns:
        matches = re.findall(pattern, content)
        for m in matches[:10]:
            if 'api' in m.lower() or 'url' in m.lower() or 'get' in m.lower():
                results['map_related'].append(m.strip())
    
    # 4. 查找 HTTP 请求模式
    http_patterns = [
        r'\.get\s*\(\s*["\']([^"\']+)["\']',
        r'\.post\s*\(\s*["\']([^"\']+)["\']',
        r'\.put\s*\(\s*["\']([^"\']+)["\']',
        r'\.delete\s*\(\s*["\']([^"\']+)["\']',
    ]
    
    for pattern in http_patterns:
        matches = re.findall(pattern, content)
        results['axios_calls'].extend(matches)
    
    return results

def find_specific_apis(filepath):
    """精确查找特定 API"""
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    specific = {
        'login': [],
        'map': [],
        'robot': [],
        'task': [],
        'site': [],
        'navigation': [],
    }
    
    # 按关键词分类搜索
    for key in specific:
        # 搜索包含该关键词的 URL 路径
        pattern = rf'["\'][^"\']*{key}[^"\']*["\']'
        matches = re.findall(pattern, content, re.IGNORECASE)
        for m in matches:
            if '/' in m or 'api' in m.lower():
                specific[key].append(m.strip('"\''))
    
    return specific

def extract_api_routes(filepath):
    """提取路由定义"""
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    routes = []
    
    # 查找类似 {path: "/xxx", method: "GET"} 的模式
    route_pattern = r'\{[^}]*path\s*:\s*["\']([^"\']+)["\'][^}]*\}'
    matches = re.findall(route_pattern, content)
    routes.extend(matches)
    
    # 查找 Vue Router 风格的路由
    vue_route_pattern = r'path\s*:\s*["\']([^"\']+)["\']'
    matches = re.findall(vue_route_pattern, content)
    routes.extend(matches)
    
    return list(set(routes))

def main():
    web_src = Path(__file__).parent / 'web_src'
    app_js = web_src / 'js' / 'app.d3121814.js'
    vendor_js = web_src / 'js' / 'vendor.94d54028.js'
    
    print("=" * 60)
    print("Standard Robots Matrix API 分析")
    print("=" * 60)
    
    if app_js.exists():
        print(f"\n分析: {app_js.name}")
        print("-" * 40)
        
        results = analyze_js_file(app_js)
        
        print("\n【发现的 API 端点】")
        for api in sorted(results['api_endpoints']):
            print(f"  {api}")
        
        print("\n【登录相关代码片段】")
        for item in results['login_related'][:10]:
            print(f"  {item[:100]}...")
        
        print("\n【地图相关代码片段】")
        for item in results['map_related'][:10]:
            print(f"  {item[:100]}...")
        
        print("\n【HTTP 请求】")
        for url in sorted(set(results['axios_calls']))[:30]:
            print(f"  {url}")
        
        # 精确查找
        print("\n" + "=" * 60)
        print("精确 API 搜索")
        print("=" * 60)
        
        specific = find_specific_apis(app_js)
        for key, values in specific.items():
            if values:
                print(f"\n【{key.upper()}】")
                for v in sorted(set(values))[:20]:
                    print(f"  {v}")
        
        # 路由
        print("\n【路由定义】")
        routes = extract_api_routes(app_js)
        for r in sorted(set(routes))[:30]:
            print(f"  {r}")
    
    # 在 JS 中搜索特定字符串
    print("\n" + "=" * 60)
    print("关键 API 搜索")
    print("=" * 60)
    
    if app_js.exists():
        with open(app_js, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # 搜索登录 API
        print("\n【登录 API】")
        login_apis = re.findall(r'["\'][^"\']*(?:login|signin|auth)[^"\']*["\']', content, re.IGNORECASE)
        for api in sorted(set(login_apis))[:20]:
            if '/' in api:
                print(f"  {api}")
        
        # 搜索地图下载 API
        print("\n【地图 API】")
        map_apis = re.findall(r'["\'][^"\']*(?:map|Map)[^"\']*["\']', content)
        for api in sorted(set(map_apis))[:30]:
            if '/' in api or 'api' in api.lower():
                print(f"  {api}")
        
        # 搜索 baseURL
        print("\n【Base URL 配置】")
        base_urls = re.findall(r'baseURL\s*[=:]\s*["\']([^"\']+)["\']', content)
        for url in set(base_urls):
            print(f"  {url}")
        
        # 搜索所有包含 /api/ 的字符串
        print("\n【所有 /api/ 路径】")
        all_apis = re.findall(r'["\']([^"\']*\/api\/[^"\']*)["\']', content)
        for api in sorted(set(all_apis)):
            print(f"  {api}")

if __name__ == '__main__':
    main()
