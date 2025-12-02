#!/usr/bin/env python3
"""
探测 Standard Robots Matrix API 端点
通过分析网页请求来发现实际的 API
"""

import requests
import json
import re
from urllib.parse import urljoin

HOST = "http://192.168.71.50"
TIMEOUT = 5

session = requests.Session()
session.headers.update({
    "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36",
    "Accept": "*/*",
})


def check_endpoint(method, path, data=None, print_response=True):
    """检查端点"""
    url = urljoin(HOST, path)
    try:
        if method == "GET":
            resp = session.get(url, timeout=TIMEOUT)
        else:
            resp = session.post(url, json=data, timeout=TIMEOUT)
        
        status = resp.status_code
        content_type = resp.headers.get("Content-Type", "")
        
        print(f"[{status}] {method} {path}")
        if print_response and status == 200:
            try:
                print(f"       {resp.json()}")
            except:
                print(f"       (HTML/Text: {len(resp.text)} chars)")
        
        return resp
    except Exception as e:
        print(f"[ERR] {method} {path}: {e}")
        return None


def explore():
    print("=" * 60)
    print("Standard Robots Matrix API 探测")
    print("=" * 60)
    
    # 首先获取主页，分析 JS 文件
    print("\n1. 获取主页...")
    resp = check_endpoint("GET", "/")
    
    if resp and resp.status_code == 200:
        html = resp.text
        
        # 查找 JS 文件
        js_files = re.findall(r'src="([^"]+\.js)"', html)
        print(f"   发现 {len(js_files)} 个 JS 文件")
        
        # 查找可能的 API 基础路径
        api_hints = re.findall(r'["\']/(api|rest|v\d)[^"\']*["\']', html)
        print(f"   API 提示: {set(api_hints)}")
    
    # 常见 API 路径探测
    print("\n2. 探测常见 API 路径...")
    
    common_paths = [
        "/api",
        "/rest",
        "/v1",
        "/api/v1",
        "/api/v2",
        "/rpc",
        "/graphql",
        "/socket.io",
        "/ws",
        "/websocket",
        # Matrix 特定
        "/matrix",
        "/matrix/api",
        "/sr",
        "/sr/api",
        "/robot",
        "/robot/api",
    ]
    
    for path in common_paths:
        check_endpoint("GET", path, print_response=True)
    
    # 登录相关探测
    print("\n3. 探测登录端点...")
    
    login_data = {"username": "dev", "password": "_sr_dev_"}
    login_paths = [
        "/api/user/login",
        "/api/auth/login",
        "/api/login",
        "/user/login",
        "/auth/login",
        "/login",
        "/api/v1/login",
        "/api/v2/login",
        "/matrix/login",
        "/matrix/api/login",
        "/rpc/login",
        "/sr/login",
        "/robot/login",
        # 带后缀
        "/api/user/login.json",
        "/api/login.json",
    ]
    
    for path in login_paths:
        resp = check_endpoint("POST", path, data=login_data)
        if resp and resp.status_code == 200:
            try:
                result = resp.json()
                if result.get("code") == 0 or result.get("success") or "token" in str(result).lower():
                    print(f"   *** 可能的登录端点: {path} ***")
            except:
                pass
    
    # 检查是否有 swagger/openapi
    print("\n4. 检查 API 文档...")
    doc_paths = [
        "/swagger",
        "/swagger.json",
        "/swagger/index.html",
        "/swagger-ui",
        "/swagger-ui.html",
        "/api-docs",
        "/api-docs.json",
        "/openapi",
        "/openapi.json",
        "/docs",
        "/redoc",
    ]
    
    for path in doc_paths:
        resp = check_endpoint("GET", path)
        if resp and resp.status_code == 200:
            print(f"   *** 发现 API 文档: {path} ***")
    
    # 检查静态资源中的 API 定义
    print("\n5. 检查 JS 文件中的 API...")
    if resp:
        js_paths = [
            "/static/js/app.js",
            "/static/js/main.js",
            "/assets/index.js",
            "/js/app.js",
            "/js/main.js",
        ]
        
        for path in js_paths:
            resp = check_endpoint("GET", path, print_response=False)
            if resp and resp.status_code == 200:
                # 在 JS 中搜索 API 端点
                js_content = resp.text
                api_endpoints = re.findall(r'["\']/(api|rest|rpc)/[^"\']+["\']', js_content)
                if api_endpoints:
                    print(f"   在 {path} 中发现 API 端点:")
                    for ep in set(api_endpoints[:20]):
                        print(f"      - {ep}")
    
    print("\n" + "=" * 60)
    print("探测完成")
    print("=" * 60)


if __name__ == "__main__":
    explore()
