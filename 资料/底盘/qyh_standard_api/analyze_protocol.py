#!/usr/bin/env python3
"""
深入分析 Standard Robots Matrix 的通信协议
重点搜索 WebSocket、Protobuf 和登录相关代码
"""

import re
from pathlib import Path

def search_in_js(content, patterns, context_size=200):
    """在 JS 内容中搜索模式"""
    results = []
    for pattern in patterns:
        for match in re.finditer(pattern, content, re.IGNORECASE):
            start = max(0, match.start() - context_size)
            end = min(len(content), match.end() + context_size)
            context = content[start:end]
            results.append({
                'pattern': pattern,
                'match': match.group(),
                'context': context
            })
    return results

def main():
    web_src = Path(__file__).parent / 'web_src'
    app_js = web_src / 'js' / 'app.d3121814.js'
    
    with open(app_js, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    print("=" * 70)
    print("Standard Robots Matrix 通信协议分析")
    print("=" * 70)
    
    # 1. 搜索 WebSocket 相关
    print("\n" + "=" * 70)
    print("【WebSocket 配置】")
    print("=" * 70)
    
    ws_patterns = [
        r'ws://[^"\']+',
        r'wss://[^"\']+',
        r'WebSocket\s*\([^)]+\)',
        r'new\s+WebSocket',
        r'\.onopen',
        r'\.onmessage',
        r'protobuf',
        r'\.proto\.',
    ]
    
    # 搜索 WebSocket URL
    ws_urls = re.findall(r'["\']?(wss?://[^"\'>\s]+)["\']?', content)
    for url in set(ws_urls):
        print(f"  WebSocket URL: {url}")
    
    # 搜索端口配置
    port_patterns = re.findall(r'port["\']?\s*[:=]\s*(\d+)', content, re.IGNORECASE)
    print(f"\n  发现的端口配置: {sorted(set(port_patterns))}")
    
    # 2. 搜索 Protobuf 相关
    print("\n" + "=" * 70)
    print("【Protobuf 消息类型】")
    print("=" * 70)
    
    proto_types = re.findall(r'proto\.(\w+)', content)
    proto_types_unique = sorted(set(proto_types))[:50]
    for pt in proto_types_unique:
        print(f"  proto.{pt}")
    
    # 3. 搜索请求/响应类型
    print("\n" + "=" * 70)
    print("【请求类型 (RequestType)】")
    print("=" * 70)
    
    request_types = re.findall(r'REQUEST_(\w+)', content)
    for rt in sorted(set(request_types)):
        print(f"  REQUEST_{rt}")
    
    print("\n【响应类型 (ResponseType)】")
    response_types = re.findall(r'RESPONSE_(\w+)', content)
    for rt in sorted(set(response_types)):
        print(f"  RESPONSE_{rt}")
    
    # 4. 搜索登录相关的详细代码
    print("\n" + "=" * 70)
    print("【登录流程分析】")
    print("=" * 70)
    
    # 搜索 login 函数
    login_func = re.findall(r'login\s*[=:]\s*(?:async\s*)?\([^)]*\)\s*(?:=>)?\s*\{[^}]{0,500}', content)
    for lf in login_func[:3]:
        print(f"\n  Login 函数片段:\n    {lf[:200]}...")
    
    # 搜索认证相关
    auth_patterns = re.findall(r'["\'](?:username|password|token|session)["\']', content, re.IGNORECASE)
    print(f"\n  认证相关字段: {sorted(set(auth_patterns))}")
    
    # 5. 搜索 uiServerDomain
    print("\n" + "=" * 70)
    print("【服务器域名配置】")
    print("=" * 70)
    
    domain_patterns = re.findall(r'uiServerDomain[^;]{0,100}', content)
    for dp in set(domain_patterns)[:5]:
        print(f"  {dp}")
    
    # 6. 搜索具体的连接建立代码
    print("\n" + "=" * 70)
    print("【连接初始化代码】")
    print("=" * 70)
    
    connect_patterns = [
        r'connect\s*\([^)]*\)\s*\{[^}]{0,300}',
        r'init\s*\([^)]*\)\s*\{[^}]{0,300}',
        r'createConnection[^;]{0,200}',
    ]
    
    for pattern in connect_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        for m in matches[:2]:
            print(f"\n  {m[:150]}...")
    
    # 7. 搜索 PIN 码登录
    print("\n" + "=" * 70)
    print("【PIN 码登录】")
    print("=" * 70)
    
    pin_patterns = re.findall(r'[pP][iI][nN][^;]{0,150}', content)
    for pp in pin_patterns[:5]:
        if 'login' in pp.lower() or 'auth' in pp.lower():
            print(f"  {pp[:100]}...")
    
    # 8. 搜索 HTTP 请求头配置
    print("\n" + "=" * 70)
    print("【HTTP 请求配置】")
    print("=" * 70)
    
    headers_patterns = re.findall(r'headers\s*[:=]\s*\{[^}]{0,200}\}', content)
    for hp in set(headers_patterns)[:5]:
        print(f"  {hp}")
    
    # 9. 搜索 cookie/session 相关
    print("\n" + "=" * 70)
    print("【Session/Cookie】")
    print("=" * 70)
    
    session_patterns = re.findall(r'sessionId[^;]{0,100}', content)
    for sp in set(session_patterns)[:10]:
        print(f"  {sp[:80]}...")
    
    # 10. 重点：搜索发送登录请求的代码
    print("\n" + "=" * 70)
    print("【登录请求构造】")
    print("=" * 70)
    
    # 搜索构造 login 请求的代码
    login_req_patterns = [
        r'CMD_LOGIN[^;]{0,200}',
        r'REQUEST_LOGIN[^;]{0,200}',
        r'sendLogin[^;]{0,200}',
        r'doLogin[^;]{0,200}',
    ]
    
    for pattern in login_req_patterns:
        matches = re.findall(pattern, content, re.IGNORECASE)
        for m in matches[:3]:
            print(f"  {m[:100]}...")

if __name__ == '__main__':
    main()
