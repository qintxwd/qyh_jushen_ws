#!/usr/bin/env python3
"""
测试 WebSocket 连接获取当前地图和机器人状态
Standard Robots Matrix 使用 Protobuf 通信
"""
import websocket
import struct
import json
import time
import threading

HOST = "192.168.71.50"
# 根据 JS 分析，WebSocket 端口可能是 5002
WS_PORTS = [5002, 80, 8080, 9090]

def on_message(ws, message):
    """收到消息"""
    print(f"\n收到消息: {len(message)} bytes")
    if isinstance(message, bytes):
        # 尝试解析为 Protobuf 或其他格式
        print(f"  Raw (hex): {message[:50].hex()}...")
        # 尝试找 mapName 字符串
        try:
            text = message.decode('utf-8', errors='ignore')
            if 'map' in text.lower() or 'yuanchang' in text.lower() or 'standard' in text.lower():
                print(f"  Contains map info!")
                # 尝试找地图名
                import re
                names = re.findall(r'(yuanchang|standard|xinjianditubiaoding|calibrationoptopt)', text)
                if names:
                    print(f"  Found map names: {names}")
        except:
            pass
    else:
        print(f"  Text: {message[:200]}")

def on_error(ws, error):
    """错误"""
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    """关闭"""
    print(f"Connection closed: {close_status_code} {close_msg}")

def on_open(ws):
    """连接打开"""
    print("Connected!")
    # 尝试发送请求消息
    # 根据 JS 代码，需要发送 Protobuf 格式的请求
    # 但先尝试简单的 JSON 请求
    try:
        # 尝试 JSON 格式
        msg = {"type": "REQUEST_ALL_STATE"}
        ws.send(json.dumps(msg))
        print("Sent JSON request")
    except Exception as e:
        print(f"Failed to send: {e}")

def test_port(port):
    """测试特定端口"""
    print(f"\n{'='*50}")
    print(f"Testing ws://{HOST}:{port}")
    print('='*50)
    
    ws_url = f"ws://{HOST}:{port}"
    
    try:
        ws = websocket.WebSocketApp(
            ws_url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close
        )
        
        # 在线程中运行，设置超时
        wst = threading.Thread(target=lambda: ws.run_forever(ping_interval=5))
        wst.daemon = True
        wst.start()
        
        # 等待几秒
        time.sleep(3)
        ws.close()
        
        return True
    except Exception as e:
        print(f"Failed: {e}")
        return False

# 先测试 HTTP 升级方式
print("Testing WebSocket connections...")

for port in WS_PORTS:
    try:
        test_port(port)
    except Exception as e:
        print(f"Port {port} error: {e}")

# 也尝试 ws://host/ws 路径
print(f"\n{'='*50}")
print(f"Testing ws://{HOST}/ws")
print('='*50)

try:
    ws = websocket.create_connection(f"ws://{HOST}/ws", timeout=3)
    print("Connected to /ws!")
    ws.close()
except Exception as e:
    print(f"Failed: {e}")
