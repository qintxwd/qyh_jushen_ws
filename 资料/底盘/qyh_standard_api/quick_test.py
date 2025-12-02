#!/usr/bin/env python3
"""快速测试 API"""
import requests
import json

BASE = "http://192.168.71.50"

endpoints = [
    "/api/v2/map",
    "/api/v0/map",
    "/api/v0/accounts",
    "/api/v0/info",
    "/api/v0/version",
]

for ep in endpoints:
    try:
        r = requests.get(f"{BASE}{ep}", timeout=10)
        print(f"\n{ep}")
        print(f"  Status: {r.status_code}")
        if r.status_code == 200:
            ct = r.headers.get('Content-Type', '')
            if 'json' in ct:
                data = r.json()
                print(f"  Data: {json.dumps(data, ensure_ascii=False)[:300]}")
            else:
                print(f"  Text: {r.text[:300]}")
        else:
            print(f"  Body: {r.text[:200]}")
    except Exception as e:
        print(f"\n{ep}")
        print(f"  Error: {e}")
