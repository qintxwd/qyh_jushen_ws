#!/usr/bin/env python3
"""
Standard Robots Matrix REST API 测试脚本
测试直接访问 REST API 是否需要认证
"""

import requests
import json
from pathlib import Path
import yaml

def load_config():
    """加载配置"""
    config_path = Path(__file__).parent / 'config.yaml'
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)

def test_rest_api():
    config = load_config()
    host = config['matrix']['host']
    port = config['matrix']['port']
    base_url = f"http://{host}:{port}"
    
    session = requests.Session()
    session.timeout = 5
    
    # 测试的 API 端点
    test_endpoints = [
        # 地图相关
        ("GET", "/api/v2/map", "地图列表 v2"),
        ("GET", "/api/v0/map", "地图列表 v0"),
        
        # 账户相关
        ("GET", "/api/v0/accounts", "账户列表"),
        ("GET", "/api/v0/account", "当前账户"),
        
        # 系统信息
        ("GET", "/api/v0/info", "系统信息"),
        ("GET", "/api/v0/version", "版本信息"),
        ("GET", "/api/v0/status", "状态信息"),
        
        # 配置
        ("GET", "/api/v0/config", "配置信息"),
        ("GET", "/api/v2/config", "配置信息 v2"),
    ]
    
    print("=" * 70)
    print("Standard Robots Matrix REST API 测试")
    print(f"Base URL: {base_url}")
    print("=" * 70)
    
    for method, endpoint, description in test_endpoints:
        url = f"{base_url}{endpoint}"
        try:
            if method == "GET":
                resp = session.get(url)
            elif method == "POST":
                resp = session.post(url)
            
            print(f"\n【{description}】")
            print(f"  {method} {endpoint}")
            print(f"  Status: {resp.status_code}")
            
            # 检查响应类型
            content_type = resp.headers.get('Content-Type', '')
            print(f"  Content-Type: {content_type}")
            
            if resp.status_code == 200:
                # 尝试解析 JSON
                if 'json' in content_type:
                    data = resp.json()
                    if isinstance(data, list):
                        print(f"  Response: List with {len(data)} items")
                        if data:
                            print(f"  First item: {json.dumps(data[0], ensure_ascii=False)[:200]}...")
                    elif isinstance(data, dict):
                        print(f"  Response: {json.dumps(data, ensure_ascii=False)[:300]}...")
                    else:
                        print(f"  Response: {data}")
                else:
                    print(f"  Response (text): {resp.text[:200]}...")
            elif resp.status_code in [401, 403]:
                print(f"  ⚠️ 需要认证!")
            elif resp.status_code == 404:
                print(f"  ❌ 端点不存在")
            else:
                print(f"  Response: {resp.text[:200]}")
                
        except requests.exceptions.Timeout:
            print(f"\n【{description}】")
            print(f"  {method} {endpoint}")
            print(f"  ❌ 超时")
        except requests.exceptions.ConnectionError as e:
            print(f"\n【{description}】")
            print(f"  {method} {endpoint}")
            print(f"  ❌ 连接失败: {e}")
        except Exception as e:
            print(f"\n【{description}】")
            print(f"  {method} {endpoint}")
            print(f"  ❌ 错误: {e}")
    
    # 测试 WebSocket 端口
    print("\n" + "=" * 70)
    print("【WebSocket 端口测试】")
    print("=" * 70)
    
    ws_ports = [5002, 80, 8080, 9090]
    for ws_port in ws_ports:
        try:
            # 尝试 HTTP 请求到 WebSocket 端口
            ws_url = f"http://{host}:{ws_port}"
            resp = session.get(ws_url, timeout=2)
            print(f"  Port {ws_port}: HTTP {resp.status_code}")
        except Exception as e:
            print(f"  Port {ws_port}: {type(e).__name__}")

if __name__ == '__main__':
    test_rest_api()
