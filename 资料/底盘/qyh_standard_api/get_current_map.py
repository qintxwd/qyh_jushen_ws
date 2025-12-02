#!/usr/bin/env python3
"""
获取当前地图名称
依次尝试以下方法:
1. REST API - 无需登录
2. WebSocket - 从 Protobuf 状态推送中解析 mapName
3. 网页爬取 - 从 Matrix 网页界面获取
"""
import requests
import websocket
import json
import time
import threading
import struct
from typing import Optional, List


# 配置
HOST = "192.168.71.50"
HTTP_PORT = 80
WS_PORT = 5002

# 已知地图名 (用于快速匹配)
KNOWN_MAPS = [
    'yuanchang', 'standard', 'xinjianditubiaoding', 
    'calibrationoptopt', 'NO_MAP'
]


class CurrentMapFinder:
    """获取当前地图名称的工具类"""
    
    def __init__(self, host: str = HOST, http_port: int = HTTP_PORT,
                 ws_port: int = WS_PORT):
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        
        if http_port != 80:
            self.base_url = f"http://{host}:{http_port}"
        else:
            self.base_url = f"http://{host}"
        self.ws_url = f"ws://{host}:{ws_port}"
        
        self.session = requests.Session()
        self.session.headers.update({
            "Accept": "application/json, text/plain, */*",
        })
    
    # ==================== 方法1: REST API ====================
    
    def try_rest_api(self) -> Optional[str]:
        """尝试通过 REST API 获取当前地图 (无需登录)"""
        print("\n" + "=" * 60)
        print("方法1: REST API")
        print("=" * 60)
        
        # 尝试的端点列表 (基于 download_maps.py 的成功经验)
        endpoints = [
            # 地图列表 (已知可用)
            "/api/v0/map",
            # 可能的状态/当前地图端点
            "/api/v0/state",
            "/api/v0/robot/state",
            "/api/v0/system/state",
            "/api/v0/map/current",
            "/api/v0/navigation/state",
            "/api/v0/robot/status",
        ]
        
        for endpoint in endpoints:
            result = self._check_endpoint(endpoint)
            if result:
                return result
        
        print("  REST API 方法未能找到当前地图")
        return None
    
    def _check_endpoint(self, endpoint: str) -> Optional[str]:
        """检查单个端点"""
        try:
            url = f"{self.base_url}{endpoint}"
            response = self.session.get(url, timeout=5)
            
            if response.status_code == 200:
                ct = response.headers.get('Content-Type', '')
                if 'json' in ct:
                    data = response.json()
                    map_name = self._extract_map_name(data)
                    if map_name:
                        print(f"  ✅ {endpoint} -> {map_name}")
                        return map_name
                    # 简短显示响应
                    text = json.dumps(data, ensure_ascii=False)[:200]
                    print(f"  {endpoint}: {text}...")
            elif response.status_code != 404:
                print(f"  {endpoint}: HTTP {response.status_code}")
        except requests.exceptions.Timeout:
            print(f"  {endpoint}: 超时")
        except Exception as e:
            pass
        return None
    
    def _extract_map_name(self, data, depth=0) -> Optional[str]:
        """从 JSON 数据中提取地图名称"""
        if depth > 5:
            return None
        
        if isinstance(data, str):
            for known in KNOWN_MAPS:
                if known == data:
                    return known
            return None
        
        if isinstance(data, dict):
            # 跳过地图列表响应 (只包含 'maps' 字段)
            if 'maps' in data and len(data) == 1:
                return None
            
            # 检查常见的地图名字段
            map_keys = [
                'mapName', 'map_name', 'MapName', 'currentMap',
                'current_map', 'loadedMap', 'activeMap'
            ]
            for key in map_keys:
                if key in data:
                    value = data[key]
                    if isinstance(value, str) and len(value) > 0:
                        if value != 'NO_MAP':
                            return value
            
            # 递归检查 (但跳过 'maps' 数组)
            for key, value in data.items():
                if key != 'maps':
                    result = self._extract_map_name(value, depth + 1)
                    if result:
                        return result
        
        if isinstance(data, list):
            for item in data:
                result = self._extract_map_name(item, depth + 1)
                if result:
                    return result
        
        return None
    
    # ==================== 方法2: WebSocket ====================
    
    def try_websocket(self, timeout: int = 10) -> Optional[str]:
        """尝试通过 WebSocket 获取当前地图"""
        print("\n" + "=" * 60)
        print("方法2: WebSocket (Protobuf)")
        print("=" * 60)
        
        result = {"map_name": None, "messages": [], "connected": False}
        
        def extract_strings(data: bytes, min_len: int = 3) -> List[str]:
            """从二进制数据中提取 ASCII 字符串"""
            strings = []
            current = []
            for byte in data:
                if 32 <= byte <= 126:
                    current.append(chr(byte))
                else:
                    if len(current) >= min_len:
                        strings.append(''.join(current))
                    current = []
            if len(current) >= min_len:
                strings.append(''.join(current))
            return strings
        
        def on_message(ws, message):
            if isinstance(message, bytes):
                result["messages"].append(message)
                
                # 从二进制消息中提取字符串
                strings = extract_strings(message)
                for s in strings:
                    # 检查是否是已知地图名
                    for known in KNOWN_MAPS:
                        if known == s and known != 'NO_MAP':
                            result["map_name"] = known
                            print(f"  ✅ 找到地图名: {known}")
                            ws.close()
                            return
                    # 检查是否可能是地图名（合理长度的标识符）
                    if (3 < len(s) < 50 and 
                        s.replace('_', '').replace('-', '').isalnum() and
                        not s.startswith('/')):
                        # 可能是地图名，记录下来
                        pass
        
        def on_error(ws, error):
            print(f"  WebSocket 错误: {error}")
        
        def on_close(ws, code, msg):
            pass
        
        def on_open(ws):
            result["connected"] = True
            print(f"  已连接到 {self.ws_url}")
            print(f"  等待系统状态推送...")
        
        try:
            ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            
            # 后台线程运行
            wst = threading.Thread(
                target=lambda: ws.run_forever(ping_interval=5)
            )
            wst.daemon = True
            wst.start()
            
            # 等待结果
            start = time.time()
            while time.time() - start < timeout:
                if result["map_name"]:
                    return result["map_name"]
                time.sleep(0.5)
            
            ws.close()
            
            # 分析收到的消息
            if result["messages"]:
                print(f"  收到 {len(result['messages'])} 条消息")
                all_strings = set()
                for msg in result["messages"]:
                    all_strings.update(extract_strings(msg))
                # 过滤可能的地图名
                candidates = [
                    s for s in all_strings 
                    if 3 < len(s) < 50 and 
                    s.replace('_', '').replace('-', '').isalnum()
                ]
                if candidates:
                    print(f"  可能的地图名: {sorted(candidates)[:10]}")
            elif not result["connected"]:
                print(f"  无法连接到 WebSocket")
            else:
                print(f"  未收到任何消息")
                
        except Exception as e:
            print(f"  WebSocket 异常: {e}")
        
        return None
    
    # ==================== 方法3: 网页爬取 ====================
    
    def try_web_scraping(self) -> Optional[str]:
        """尝试从网页界面获取当前地图名"""
        print("\n" + "=" * 60)
        print("方法3: 网页爬取")
        print("=" * 60)
        
        # 尝试获取主页面
        try:
            url = f"{self.base_url}/"
            response = self.session.get(url, timeout=10)
            
            if response.status_code == 200:
                # 检查页面内容中是否有地图名
                for known in KNOWN_MAPS:
                    if known != 'NO_MAP' and known in response.text:
                        print(f"  ✅ 在页面中找到: {known}")
                        return known
                print("  页面中未找到已知地图名")
        except Exception as e:
            print(f"  获取页面失败: {e}")
        
        return None
    
    # ==================== 综合方法 ====================
    
    def get_current_map(self) -> Optional[str]:
        """
        获取当前地图名称
        依次尝试所有方法直到成功
        """
        # 方法1: REST API
        map_name = self.try_rest_api()
        if map_name:
            return map_name
        
        # 方法2: WebSocket
        map_name = self.try_websocket(timeout=8)
        if map_name:
            return map_name
        
        # 方法3: 网页爬取
        map_name = self.try_web_scraping()
        if map_name:
            return map_name
        
        return None
    
    def close(self):
        """清理资源"""
        self.session.close()


def main():
    print("=" * 60)
    print("获取 Matrix 系统当前地图")
    print(f"目标: {HOST}")
    print("=" * 60)
    
    finder = CurrentMapFinder()
    
    try:
        map_name = finder.get_current_map()
        
        print("\n" + "=" * 60)
        if map_name:
            print(f"✅ 当前地图: {map_name}")
        else:
            print("❌ 无法获取当前地图")
            print("\n提示: 可能需要通过 WebSocket Protobuf 解析")
            print("或者查看网页界面右上角的地图名称")
        print("=" * 60)
        
        return map_name
        
    finally:
        finder.close()


if __name__ == '__main__':
    main()
