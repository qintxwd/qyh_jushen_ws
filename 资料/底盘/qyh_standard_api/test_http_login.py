#!/usr/bin/env python3
"""
尝试通过 HTTP 登录并获取当前状态
虽然主要功能通过 WebSocket/Protobuf，但可能有 HTTP 登录接口
"""
import requests
import json
import hashlib

BASE = "http://192.168.71.50"

session = requests.Session()

# 从 accounts API 看到密码是 MD5 哈希
# user: ee11cbb19052e40b07aac0ca060c23ee (这是 "user" 的 MD5)
# admin: 21232f297a57a5a743894a0e4a801fc3 (这是 "admin" 的 MD5)

# 尝试各种登录端点
login_endpoints = [
    "/api/v0/login",
    "/api/v0/account/login",
    "/api/v0/auth/login",
    "/api/v2/login",
    "/login",
    "/api/login",
]

# 测试用户名和密码
credentials = [
    {"username": "dev", "password": "_sr_dev_"},
    {"username": "user", "password": "user"},
    {"username": "admin", "password": "admin"},
]

print("=" * 70)
print("测试 HTTP 登录")
print("=" * 70)

for endpoint in login_endpoints:
    for cred in credentials:
        try:
            # 尝试 JSON POST
            r = session.post(
                f"{BASE}{endpoint}",
                json=cred,
                timeout=5
            )
            print(f"\n{endpoint} with {cred['username']}")
            print(f"  Status: {r.status_code}")
            if r.status_code != 404:
                print(f"  Response: {r.text[:200]}")
        except Exception as e:
            print(f"\n{endpoint}: {type(e).__name__}")

# 尝试带 session 参数的请求
print("\n" + "=" * 70)
print("测试带认证的 API")
print("=" * 70)

# 设置可能的认证头
headers = {
    "Authorization": "Basic " + "ZGV2Ol9zcl9kZXZf",  # base64(dev:_sr_dev_)
}

auth_endpoints = [
    "/api/v0/current",
    "/api/v0/robot/current",
    "/api/v0/system/current",
]

for ep in auth_endpoints:
    try:
        r = session.get(f"{BASE}{ep}", headers=headers, timeout=5)
        print(f"\n{ep}")
        print(f"  Status: {r.status_code}")
        if r.status_code == 200:
            print(f"  Response: {r.text[:200]}")
    except Exception as e:
        print(f"\n{ep}: {e}")

# 检查是否有服务器信息端点
print("\n" + "=" * 70)
print("测试其他 API")
print("=" * 70)

misc_endpoints = [
    "/api/v0/server",
    "/api/v0/robot",
    "/api/v0/robots",
    "/api/v0/mission",
    "/api/v0/missions",
    "/api/v2/mission",
    "/api/v0/task",
    "/api/v0/tasks",
]

for ep in misc_endpoints:
    try:
        r = session.get(f"{BASE}{ep}", timeout=5)
        print(f"\n{ep}")
        print(f"  Status: {r.status_code}")
        if r.status_code == 200:
            print(f"  Response: {r.text[:300]}")
    except Exception as e:
        print(f"\n{ep}: {type(e).__name__}")
