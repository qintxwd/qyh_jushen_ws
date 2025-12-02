#!/usr/bin/env python3
"""测试获取当前地图和机器人状态的 API"""
import requests
import json

BASE = "http://192.168.71.50"

# 测试各种可能的端点
endpoints = [
    # 当前地图
    "/api/v0/map/current",
    "/api/v0/current/map",
    "/api/v0/system/state",
    "/api/v0/state",
    "/api/v0/robot/state",
    "/api/v0/status",
    "/api/v2/state",
    "/api/v2/status",
    "/api/v2/system",
    "/api/v2/robot",
    # 配置
    "/api/v0/config",
    "/api/v2/config",
]

print("=" * 70)
print("测试各种 API 端点")
print("=" * 70)

for ep in endpoints:
    try:
        r = requests.get(f"{BASE}{ep}", timeout=5)
        print(f"\n{ep}")
        print(f"  Status: {r.status_code}")
        if r.status_code == 200:
            ct = r.headers.get('Content-Type', '')
            if 'json' in ct:
                data = r.json()
                # 尝试找 mapName
                if isinstance(data, dict):
                    text = json.dumps(data, ensure_ascii=False)[:500]
                    print(f"  Data: {text}")
                else:
                    print(f"  Data: {str(data)[:300]}")
            else:
                print(f"  Text: {r.text[:300]}")
        else:
            print(f"  Body: {r.text[:100]}")
    except Exception as e:
        print(f"\n{ep}")
        print(f"  Error: {type(e).__name__}: {e}")
