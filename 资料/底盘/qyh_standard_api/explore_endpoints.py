#!/usr/bin/env python3
"""探索 Matrix API 端点"""
import requests

BASE = "http://192.168.71.50"

# 探索登录端点
login_endpoints = [
    "/api/user/login",
    "/api/auth/login", 
    "/api/login",
    "/api/v1/user/login",
    "/api/v2/user/login",
    "/user/login",
    "/login",
    "/api/v0/user/login",
]

print("=" * 60)
print("探索登录端点")
print("=" * 60)

for ep in login_endpoints:
    try:
        r = requests.post(
            f"{BASE}{ep}", 
            json={"username": "dev", "password": "_sr_dev_"}, 
            timeout=3
        )
        print(f"{ep}: {r.status_code}")
        if r.status_code == 200:
            print(f"  响应: {r.text[:200]}")
    except Exception as e:
        print(f"{ep}: {type(e).__name__}")

# 探索可能不需要登录的端点
print("\n" + "=" * 60)
print("探索公开端点 (GET)")
print("=" * 60)

public_endpoints = [
    "/",
    "/api",
    "/api/v0",
    "/api/v2", 
    "/api/version",
    "/api/info",
    "/api/system/state",
    "/api/v0/system/state",
    "/api/v2/system/state",
    "/api/state",
    "/api/robot/state",
    "/api/map/list",
    "/api/v0/map/list",
    "/api/v2/map/list",
]

for ep in public_endpoints:
    try:
        r = requests.get(f"{BASE}{ep}", timeout=3)
        print(f"{ep}: {r.status_code}")
        if r.status_code == 200 and len(r.text) < 500:
            ct = r.headers.get("Content-Type", "")
            if "json" in ct:
                print(f"  JSON: {r.text[:300]}")
            elif len(r.text) < 200:
                print(f"  Text: {r.text[:200]}")
    except Exception as e:
        print(f"{ep}: {type(e).__name__}")
