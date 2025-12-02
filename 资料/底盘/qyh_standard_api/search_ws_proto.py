#!/usr/bin/env python3
"""搜索 WebSocket 和 Protobuf 的具体实现"""
import re
from pathlib import Path

def search():
    js_path = Path(__file__).parent / 'web_src' / 'js' / 'app.d3121814.js'
    with open(js_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    print("=" * 70)
    print("搜索 WebSocket 和 Protobuf")
    print("=" * 70)
    
    # 搜索 WebSocket URL 构建
    patterns = [
        (r'ws://[^"\']+', "WebSocket URL"),
        (r'5002', "5002 端口"),
        (r'new\s+WebSocket\([^)]+\)', "WebSocket 创建"),
        (r'\.proto\s*=', "Proto 对象"),
        (r'protobuf\.load', "Protobuf 加载"),
        (r'\.proto\.Message', "Proto Message"),
        (r'\.encode\([^)]*\)\.finish\(\)', "Protobuf encode"),
        (r'\.decode\([^)]*\)', "Protobuf decode"),
    ]
    
    for pattern, desc in patterns:
        matches = re.findall(pattern, content)
        if matches:
            print(f"\n【{desc}】({pattern}) 找到 {len(matches)} 处")
            for m in list(set(matches))[:5]:
                print(f"  {m[:100]}")
    
    # 搜索发送登录请求的代码
    print("\n" + "=" * 70)
    print("搜索登录和状态请求")
    print("=" * 70)
    
    # 找到 sendRequest 相关代码
    send_patterns = [
        r'__sendRequestMsg[^}]{0,200}',
        r'sendRequestMsg[^}]{0,200}',
        r'REQUEST_LOGIN[^}]{0,200}',
        r'REQUEST_ALL_STATE[^}]{0,200}',
        r'LoginRequest[^}]{0,200}',
    ]
    
    for pattern in send_patterns:
        matches = re.findall(pattern, content)
        if matches:
            print(f"\n【{pattern}】")
            for m in matches[:2]:
                print(f"  {m[:200]}")
    
    # 搜索消息类型枚举
    print("\n" + "=" * 70)
    print("搜索 RequestType 枚举")
    print("=" * 70)
    
    # 找请求类型的数字对应关系
    req_type_match = re.search(r'RequestType\s*=\s*\{[^}]+\}', content)
    if req_type_match:
        print(req_type_match.group()[:500])
    
    # 搜索更多细节
    print("\n" + "=" * 70)
    print("搜索 Protobuf schema 路径")
    print("=" * 70)
    
    proto_paths = re.findall(r'["\'][^"\']*\.proto[^"\']*["\']', content)
    for p in list(set(proto_paths))[:10]:
        print(f"  {p}")

if __name__ == '__main__':
    search()
