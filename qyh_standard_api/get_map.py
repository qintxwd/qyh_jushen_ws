#!/usr/bin/env python3
"""
Standard Robots Matrix 地图同步工具

功能:
1. 从机器人底盘获取当前地图名称 (WebSocket + Protobuf)
2. 下载所有地图数据 (JSON + 图片)
3. 生成 current_map.txt 记录当前地图

使用方法:
    python get_map.py

输出目录: ~/qyh_jushen_ws/maps/
"""
import os
import sys
import json
import struct
import time
import shutil
import threading
import hashlib
from pathlib import Path
from typing import Optional

import requests
import websocket
import yaml


# ==============================================================================
# 配置
# ==============================================================================

def load_config():
    """加载配置文件"""
    config_path = Path(__file__).parent / "config.yaml"
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return {}


# ==============================================================================
# Protobuf 简单编解码器 (用于 WebSocket 通信)
# ==============================================================================

class ProtobufEncoder:
    """简单的 Protobuf 编码器"""
    
    def __init__(self):
        self.buffer = bytearray()
    
    def write_varint(self, value):
        while value > 127:
            self.buffer.append((value & 0x7f) | 0x80)
            value >>= 7
        self.buffer.append(value & 0x7f)
    
    def write_field_varint(self, field_num, value):
        tag = (field_num << 3) | 0
        self.write_varint(tag)
        self.write_varint(value)
    
    def write_field_string(self, field_num, value):
        if value is None:
            return
        tag = (field_num << 3) | 2
        self.write_varint(tag)
        data = value.encode('utf-8')
        self.write_varint(len(data))
        self.buffer.extend(data)
    
    def write_field_bytes(self, field_num, value):
        if value is None:
            return
        tag = (field_num << 3) | 2
        self.write_varint(tag)
        self.write_varint(len(value))
        self.buffer.extend(value)
    
    def write_field_fixed32(self, field_num, value):
        tag = (field_num << 3) | 5
        self.write_varint(tag)
        self.buffer.extend(struct.pack('<I', value & 0xFFFFFFFF))
    
    def write_field_fixed64(self, field_num, value):
        tag = (field_num << 3) | 1
        self.write_varint(tag)
        self.buffer.extend(struct.pack('<Q', value & 0xFFFFFFFFFFFFFFFF))
    
    def get_bytes(self):
        return bytes(self.buffer)


class ProtobufDecoder:
    """简单的 Protobuf 解码器"""
    
    def __init__(self, data):
        self.data = data
        self.pos = 0
    
    def read_varint(self):
        result = 0
        shift = 0
        while self.pos < len(self.data):
            b = self.data[self.pos]
            self.pos += 1
            result |= (b & 0x7f) << shift
            if not (b & 0x80):
                break
            shift += 7
        return result
    
    def read_field(self):
        if self.pos >= len(self.data):
            return None, None
        tag = self.read_varint()
        field_num = tag >> 3
        wire_type = tag & 0x7
        return field_num, wire_type
    
    def read_bytes(self, length):
        end = self.pos + length
        result = self.data[self.pos:end]
        self.pos = end
        return result
    
    def skip_field(self, wire_type):
        if wire_type == 0:
            self.read_varint()
        elif wire_type == 1:
            self.pos += 8
        elif wire_type == 2:
            length = self.read_varint()
            self.pos += length
        elif wire_type == 5:
            self.pos += 4


# ==============================================================================
# WebSocket 获取当前地图
# ==============================================================================

def extract_map_name_from_bytes(data: bytes) -> Optional[str]:
    """从二进制数据中提取地图名称 (field 10, tag 0x52)"""
    i = 0
    while i < len(data) - 2:
        if data[i] == 0x52:
            length = data[i + 1]
            if 0 < length < 100 and i + 2 + length <= len(data):
                try:
                    map_name = data[i + 2:i + 2 + length].decode('utf-8')
                    if map_name.replace('_', '').isalnum():
                        return map_name
                except Exception:
                    pass
        i += 1
    return None


def get_current_map_via_websocket(host: str, port: int = 5002, 
                                   username: str = "dev",
                                   password: str = "",
                                   timeout: int = 10) -> Optional[str]:
    """
    通过 WebSocket 获取当前地图名称
    
    Args:
        host: 机器人 IP
        port: WebSocket 端口 (默认 5002)
        username: 用户名
        password: 密码
        timeout: 超时秒数
    
    Returns:
        当前地图名称，或 None
    """
    result = {'map_name': None, 'session_id': 0}
    buffer = bytearray()
    seq = 1970
    
    def parse_frame(data):
        if len(data) < 4:
            return None, data
        length = struct.unpack('>I', data[:4])[0]
        if len(data) < 4 + length:
            return None, data
        return data[4:4+length], data[4+length:]
    
    def decode_message(data):
        dec = ProtobufDecoder(data)
        msg = {}
        while dec.pos < len(data):
            field_num, wire_type = dec.read_field()
            if field_num is None:
                break
            if field_num == 1 and wire_type == 0:
                msg['type'] = dec.read_varint()
            elif field_num == 2 and wire_type == 5:
                msg['seq'] = struct.unpack('<I', dec.read_bytes(4))[0]
            elif field_num == 3 and wire_type == 1:
                msg['sessionId'] = struct.unpack('<Q', dec.read_bytes(8))[0]
            elif field_num == 5 and wire_type == 2:
                length = dec.read_varint()
                msg['response'] = dec.read_bytes(length)
            elif field_num == 7 and wire_type == 2:
                length = dec.read_varint()
                msg['notification'] = dec.read_bytes(length)
            else:
                dec.skip_field(wire_type)
        return msg
    
    def on_message(ws, message):
        nonlocal buffer
        if isinstance(message, bytes):
            buffer.extend(message)
            while True:
                msg_data, remaining = parse_frame(bytes(buffer))
                if msg_data is None:
                    break
                buffer = bytearray(remaining)
                msg = decode_message(msg_data)
                
                if 'sessionId' in msg and msg['sessionId'] > 0:
                    result['session_id'] = msg['sessionId']
                
                for key in ['response', 'notification']:
                    if key in msg:
                        map_name = extract_map_name_from_bytes(msg[key])
                        if map_name:
                            result['map_name'] = map_name
    
    def on_open(ws):
        nonlocal seq
        seq += 1
        
        # 创建登录请求
        password_md5 = hashlib.md5(password.encode()).hexdigest()
        
        # LoginRequest
        login_enc = ProtobufEncoder()
        login_enc.write_field_string(1, username)
        login_enc.write_field_string(2, password_md5)
        login_enc.write_field_varint(3, 0)  # AA_ROOT
        login_req = login_enc.get_bytes()
        
        # Request
        req_enc = ProtobufEncoder()
        req_enc.write_field_varint(1, 0)  # REQUEST_LOGIN
        req_enc.write_field_bytes(2, login_req)
        request = req_enc.get_bytes()
        
        # Message
        msg_enc = ProtobufEncoder()
        msg_enc.write_field_varint(1, 0)  # MSG_REQUEST
        msg_enc.write_field_fixed32(2, seq)
        msg_enc.write_field_fixed64(3, 0)
        msg_enc.write_field_bytes(4, request)
        message = msg_enc.get_bytes()
        
        # Frame
        frame = struct.pack('>I', len(message)) + message
        ws.send(frame, opcode=websocket.ABNF.OPCODE_BINARY)
    
    def on_error(ws, error):
        pass
    
    def on_close(ws, code, msg):
        pass
    
    ws_url = f"ws://{host}:{port}"
    ws = websocket.WebSocketApp(
        ws_url,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    
    wst = threading.Thread(target=lambda: ws.run_forever())
    wst.daemon = True
    wst.start()
    
    # 等待结果
    for _ in range(timeout * 10):
        time.sleep(0.1)
        if result['map_name']:
            ws.close()
            return result['map_name']
    
    ws.close()
    return None


# ==============================================================================
# HTTP API 下载地图
# ==============================================================================

def get_map_list(base_url: str, timeout: int = 10) -> list:
    """获取地图列表"""
    r = requests.get(f"{base_url}/api/v0/map", timeout=timeout)
    r.raise_for_status()
    data = r.json()
    return data.get('maps', [])


def get_map_data(base_url: str, map_name: str, timeout: int = 30) -> dict:
    """获取地图 JSON 数据"""
    r = requests.get(f"{base_url}/api/v0/map/{map_name}/data", timeout=timeout)
    r.raise_for_status()
    return r.json()


def get_map_image(base_url: str, map_name: str, scale: int = 1, timeout: int = 30) -> bytes:
    """获取地图图片"""
    r = requests.get(f"{base_url}/api/v0/map/{map_name}/image?scale={scale}", timeout=timeout)
    r.raise_for_status()
    return r.content


def download_all_maps(base_url: str, output_dir: Path, timeout: int = 30) -> list:
    """
    下载所有地图数据
    
    Returns:
        下载的地图名称列表
    """
    downloaded = []
    maps = get_map_list(base_url, timeout)
    
    for m in maps:
        map_name = m['name']
        map_dir = output_dir / map_name
        map_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"  下载地图: {map_name}")
        
        # 下载 JSON 数据
        try:
            map_data = get_map_data(base_url, map_name, timeout)
            json_file = map_dir / f"{map_name}.json"
            with open(json_file, 'w', encoding='utf-8') as f:
                json.dump(map_data, f, indent=2, ensure_ascii=False)
            
            # 显示简要信息
            if isinstance(map_data, dict):
                data = map_data.get('data', {})
                nodes = data.get('node', [])
                edges = data.get('edge', [])
                stations = data.get('station', [])
                print(f"    - JSON: {len(nodes)} 节点, {len(edges)} 边, {len(stations)} 站点")
        except Exception as e:
            print(f"    - JSON 下载失败: {e}")
        
        # 下载图片
        try:
            img_data = get_map_image(base_url, map_name, timeout=timeout)
            if img_data[:8] == b'\x89PNG\r\n\x1a\n':
                img_ext = 'png'
            elif img_data[:2] == b'\xff\xd8':
                img_ext = 'jpg'
            else:
                img_ext = 'bin'
            img_file = map_dir / f"{map_name}.{img_ext}"
            with open(img_file, 'wb') as f:
                f.write(img_data)
            print(f"    - 图片: {len(img_data)} bytes")
        except Exception as e:
            print(f"    - 图片下载失败: {e}")
        
        downloaded.append(map_name)
    
    return downloaded


# ==============================================================================
# 主函数
# ==============================================================================

def main():
    """主函数"""
    print("=" * 60)
    print("Standard Robots Matrix 地图同步工具")
    print("=" * 60)
    
    # 加载配置
    config = load_config()
    matrix_config = config.get('matrix', {})
    
    host = matrix_config.get('host', '192.168.71.50')
    http_port = matrix_config.get('port', 80)
    ws_port = matrix_config.get('ws_port', 5002)
    username = matrix_config.get('username', 'dev')
    password = matrix_config.get('password', '')
    timeout = matrix_config.get('timeout', 10)
    
    # 输出目录: ~/qyh_jushen_ws/maps/
    workspace_root = Path(__file__).parent.parent  # qyh_jushen_ws
    output_dir = workspace_root / "maps"
    
    base_url = f"http://{host}:{http_port}" if http_port != 80 else f"http://{host}"
    
    print(f"\n机器人地址: {host}")
    print(f"HTTP API: {base_url}")
    print(f"WebSocket: ws://{host}:{ws_port}")
    print(f"输出目录: {output_dir}")
    
    # Step 1: 清空输出目录
    print(f"\n[1/3] 清空输出目录...")
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"  ✓ 已清空: {output_dir}")
    
    # Step 2: 获取当前地图
    print(f"\n[2/3] 获取当前地图...")
    current_map = None
    try:
        current_map = get_current_map_via_websocket(
            host=host,
            port=ws_port,
            username=username,
            password=password,
            timeout=timeout
        )
        if current_map:
            print(f"  ✓ 当前地图: {current_map}")
        else:
            print(f"  ✗ 未能获取当前地图")
    except Exception as e:
        print(f"  ✗ WebSocket 连接失败: {e}")
    
    # Step 3: 下载所有地图
    print(f"\n[3/3] 下载地图数据...")
    try:
        downloaded = download_all_maps(base_url, output_dir, timeout)
        print(f"  ✓ 下载完成: {len(downloaded)} 个地图")
    except Exception as e:
        print(f"  ✗ 下载失败: {e}")
        downloaded = []
    
    # 写入 current_map.txt
    current_map_file = output_dir / "current_map.txt"
    if current_map:
        with open(current_map_file, 'w', encoding='utf-8') as f:
            f.write(current_map)
        print(f"\n✓ 当前地图已写入: {current_map_file}")
    else:
        # 如果无法获取当前地图，尝试使用最新修改的地图
        if downloaded:
            current_map = downloaded[0]  # 使用第一个作为默认
            with open(current_map_file, 'w', encoding='utf-8') as f:
                f.write(current_map)
            print(f"\n⚠ 使用默认地图: {current_map}")
    
    # 打印结果摘要
    print("\n" + "=" * 60)
    print("同步完成!")
    print("=" * 60)
    print(f"地图目录: {output_dir}")
    print(f"当前地图: {current_map or '未知'}")
    print(f"地图列表: {', '.join(downloaded) if downloaded else '无'}")
    print("=" * 60)
    
    return 0 if current_map else 1


if __name__ == '__main__':
    sys.exit(main())
