#!/usr/bin/env python3
"""
尝试通过 WebSocket 获取当前地图名称
分析原始 Protobuf 数据中的字符串
"""
import websocket
import struct
import time
import threading

HOST = "192.168.71.50"
PORT = 5002

# 已知的地图名称
KNOWN_MAPS = ['yuanchang', 'standard', 'xinjianditubiaoding', 'calibrationoptopt', 'NO_MAP']

received_data = []
found_map = None

def extract_strings(data, min_len=3, max_len=50):
    """从二进制数据中提取可能的字符串"""
    strings = []
    current = []
    
    for byte in data:
        if 32 <= byte <= 126:  # 可打印 ASCII
            current.append(chr(byte))
        else:
            if len(current) >= min_len:
                s = ''.join(current)
                strings.append(s)
            current = []
    
    if len(current) >= min_len:
        strings.append(''.join(current))
    
    return strings

def on_message(ws, message):
    global found_map
    
    if isinstance(message, bytes):
        received_data.append(message)
        
        # 提取字符串
        strings = extract_strings(message)
        
        # 查找地图名称
        for s in strings:
            for map_name in KNOWN_MAPS:
                if map_name in s:
                    print(f"\n✓ 找到地图名称: {map_name}")
                    found_map = map_name
                    return
        
        # 显示收到的数据
        print(f"\n收到 {len(message)} bytes")
        if strings:
            # 过滤掉太短或太长的字符串
            useful = [s for s in strings if 3 <= len(s) <= 50]
            if useful:
                print(f"  字符串: {useful[:10]}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, code, msg):
    print(f"Closed: {code} {msg}")

def on_open(ws):
    print("Connected to WebSocket!")
    print("等待接收系统状态数据...")
    
    # WebSocket 连接后，服务器通常会自动发送状态
    # 如果不发送，可能需要发送特定的请求消息

def main():
    global found_map
    
    ws_url = f"ws://{HOST}:{PORT}"
    print(f"连接到 {ws_url}")
    
    ws = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    
    # 在后台运行
    wst = threading.Thread(target=lambda: ws.run_forever(ping_interval=5, ping_timeout=3))
    wst.daemon = True
    wst.start()
    
    # 等待数据
    print("\n等待 10 秒收集数据...")
    for i in range(10):
        time.sleep(1)
        print(f"  {i+1}秒, 收到 {len(received_data)} 条消息")
        if found_map:
            break
    
    ws.close()
    
    print("\n" + "=" * 70)
    if found_map:
        print(f"当前地图: {found_map}")
    else:
        print("未能自动找到地图名称")
        
        # 分析收到的所有数据
        if received_data:
            print(f"\n收到 {len(received_data)} 条消息")
            print("\n所有提取的字符串:")
            all_strings = []
            for data in received_data:
                all_strings.extend(extract_strings(data))
            
            # 去重并排序
            unique_strings = sorted(set(all_strings), key=len, reverse=True)
            for s in unique_strings[:30]:
                print(f"  {s}")

if __name__ == '__main__':
    main()
