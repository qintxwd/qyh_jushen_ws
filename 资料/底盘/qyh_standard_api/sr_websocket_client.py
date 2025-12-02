#!/usr/bin/env python3
"""
Standard Robots WebSocket 客户端
使用 Protobuf 协议获取机器人状态和当前地图名称

协议分析自 Wireshark 抓包和 Matrix Web UI JS 代码
"""
import websocket
import struct
import time
import threading
import hashlib
from typing import Optional, Callable
from enum import IntEnum


class MessageType(IntEnum):
    MSG_REQUEST = 0
    MSG_RESPONSE = 1
    MSG_COMMAND = 2
    MSG_NOTIFICATION = 3


class RequestType(IntEnum):
    REQUEST_LOGIN = 0
    REQUEST_INFO = 1
    REQUEST_SYSTEM_STATE = 2
    REQUEST_HARDWARE_STATE = 3
    REQUEST_MAP_LIST = 4
    REQUEST_BAG_LIST = 5
    REQUEST_LASER_POINTS = 6
    REQUEST_TASK_STATE = 7
    REQUEST_FILE_OPERATE = 8
    REQUEST_LOGOUT = 9
    REQUEST_LOAD_CONFIG = 10
    REQUEST_SAVE_CONFIG = 11
    REQUEST_FILE_LIST = 12
    REQUEST_MONITOR_DATA = 13
    REQUEST_CHANGE_PW = 14
    REQUEST_TIMESTAMP = 15
    REQUEST_GET_CUT_MAP = 16
    REQUEST_MISSION_LIST = 17
    REQUEST_HEARTBEAT = 18
    REQUEST_CONNECT_INFO = 19
    REQUEST_ALL_STATE = 33


class AccessAuthority(IntEnum):
    AA_ROOT = 0
    AA_ADMIN = 10
    AA_USER = 20
    AA_GUEST = 30


class ProtobufEncoder:
    """简单的 Protobuf 编码器"""
    
    def __init__(self):
        self.buffer = bytearray()
    
    def write_varint(self, value):
        """写入 varint"""
        while value > 127:
            self.buffer.append((value & 0x7f) | 0x80)
            value >>= 7
        self.buffer.append(value & 0x7f)
    
    def write_field_varint(self, field_num, value):
        """写入 varint 字段"""
        tag = (field_num << 3) | 0  # wire type 0
        self.write_varint(tag)
        self.write_varint(value)
    
    def write_field_string(self, field_num, value):
        """写入字符串字段"""
        if value is None:
            return
        tag = (field_num << 3) | 2  # wire type 2 (length-delimited)
        self.write_varint(tag)
        data = value.encode('utf-8')
        self.write_varint(len(data))
        self.buffer.extend(data)
    
    def write_field_bytes(self, field_num, value):
        """写入嵌套消息字段"""
        if value is None:
            return
        tag = (field_num << 3) | 2  # wire type 2 (length-delimited)
        self.write_varint(tag)
        self.write_varint(len(value))
        self.buffer.extend(value)
    
    def write_field_fixed32(self, field_num, value):
        """写入 fixed32 字段 (4字节小端序)"""
        tag = (field_num << 3) | 5  # wire type 5
        self.write_varint(tag)
        self.buffer.extend(struct.pack('<I', value & 0xFFFFFFFF))
    
    def write_field_fixed64(self, field_num, value):
        """写入 fixed64 字段 (8字节小端序)"""
        tag = (field_num << 3) | 1  # wire type 1
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
        """读取 varint"""
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
        """读取字段标签和类型"""
        if self.pos >= len(self.data):
            return None, None
        tag = self.read_varint()
        field_num = tag >> 3
        wire_type = tag & 0x7
        return field_num, wire_type
    
    def read_string(self, length):
        """读取字符串"""
        end = self.pos + length
        result = self.data[self.pos:end].decode('utf-8', errors='ignore')
        self.pos = end
        return result
    
    def read_bytes(self, length):
        """读取字节"""
        end = self.pos + length
        result = self.data[self.pos:end]
        self.pos = end
        return result
    
    def skip_field(self, wire_type):
        """跳过字段"""
        if wire_type == 0:  # varint
            self.read_varint()
        elif wire_type == 1:  # 64-bit
            self.pos += 8
        elif wire_type == 2:  # length-delimited
            length = self.read_varint()
            self.pos += length
        elif wire_type == 5:  # 32-bit
            self.pos += 4


def create_login_request(username: str, password: str, access: int = 30, 
                         password_is_md5: bool = False) -> bytes:
    """
    创建 LoginRequest 消息
    message LoginRequest {
        string username = 1;
        string password = 2;  // MD5 hash
        AccessAuthority access = 3;
    }
    """
    enc = ProtobufEncoder()
    enc.write_field_string(1, username)
    # 密码需要 MD5 哈希 (即使空密码也要计算 MD5)
    if password_is_md5:
        password_md5 = password
    else:
        password_md5 = hashlib.md5(password.encode()).hexdigest()
    enc.write_field_string(2, password_md5)
    enc.write_field_varint(3, access)
    return enc.get_bytes()


def create_request_message(request_type: int, login_request: bytes = None) -> bytes:
    """
    创建 Request 消息
    message Request {
        RequestType requestType = 1;
        LoginRequest loginRequest = 2;
        ...
    }
    """
    enc = ProtobufEncoder()
    enc.write_field_varint(1, request_type)
    if login_request:
        enc.write_field_bytes(2, login_request)
    return enc.get_bytes()


def create_message(msg_type: int, seq: int, session_id: int = 0, 
                   request: bytes = None) -> bytes:
    """
    创建完整的 Message
    message Message {
        MessageType type = 1;    // int32 (varint)
        uint32 seq = 2;          // fixed32 (wire type 5)
        int64 sessionId = 3;     // fixed64 (wire type 1)
        Request request = 4;     // length-delimited
        Response response = 5;
        Command command = 6;
        Notification notification = 7;
    }
    """
    enc = ProtobufEncoder()
    enc.write_field_varint(1, msg_type)       # type: varint
    enc.write_field_fixed32(2, seq)           # seq: fixed32
    enc.write_field_fixed64(3, session_id)    # sessionId: fixed64
    if request:
        enc.write_field_bytes(4, request)      # request: length-delimited
    return enc.get_bytes()


def build_frame(message: bytes) -> bytes:
    """
    构建帧：4字节长度(big-endian) + 消息内容
    """
    length = len(message)
    header = struct.pack('>I', length)
    return header + message


def parse_frame(data: bytes) -> tuple:
    """
    解析帧，返回 (消息, 剩余数据)
    """
    if len(data) < 4:
        return None, data
    length = struct.unpack('>I', data[:4])[0]
    if len(data) < 4 + length:
        return None, data
    message = data[4:4+length]
    remaining = data[4+length:]
    return message, remaining


def decode_message(data: bytes) -> dict:
    """解码 Message"""
    dec = ProtobufDecoder(data)
    result = {}
    
    while dec.pos < len(data):
        field_num, wire_type = dec.read_field()
        if field_num is None:
            break
        
        if field_num == 1 and wire_type == 0:  # type (varint)
            result['type'] = dec.read_varint()
        elif field_num == 2 and wire_type == 5:  # seq (fixed32)
            result['seq'] = struct.unpack('<I', dec.read_bytes(4))[0]
        elif field_num == 3 and wire_type == 1:  # sessionId (fixed64)
            result['sessionId'] = struct.unpack('<Q', dec.read_bytes(8))[0]
        elif field_num == 4 and wire_type == 2:  # request
            length = dec.read_varint()
            result['request'] = dec.read_bytes(length)
        elif field_num == 5 and wire_type == 2:  # response
            length = dec.read_varint()
            result['response'] = dec.read_bytes(length)
        elif field_num == 6 and wire_type == 2:  # command
            length = dec.read_varint()
            result['command'] = dec.read_bytes(length)
        elif field_num == 7 and wire_type == 2:  # notification
            length = dec.read_varint()
            result['notification'] = dec.read_bytes(length)
        else:
            dec.skip_field(wire_type)
    
    return result


def extract_map_name_from_bytes(data: bytes) -> Optional[str]:
    """
    从二进制数据中提取地图名称
    地图名在 SystemState 中，field 10 (tag 0x52), 格式: 52 09 "yuanchang"
    """
    # 搜索 field 10 string (tag = 0x52 = 82 = (10 << 3) | 2)
    i = 0
    while i < len(data) - 2:
        if data[i] == 0x52:  # field 10, wire type 2 (string)
            length = data[i + 1]
            if length > 0 and length < 100 and i + 2 + length <= len(data):
                try:
                    map_name = data[i + 2:i + 2 + length].decode('utf-8')
                    # 验证是否是有效的地图名（不含特殊字符）
                    if map_name.isalnum() or '_' in map_name:
                        return map_name
                except Exception:
                    pass
        i += 1
    
    # 备用方法：搜索已知地图名
    known_maps = ['yuanchang', 'standard', 'xinjianditubiaoding', 
                  'calibrationoptopt', 'NO_MAP']
    for map_name in known_maps:
        if map_name.encode('utf-8') in data:
            return map_name
    
    return None


class SRWebSocketClient:
    """Standard Robots WebSocket 客户端"""
    
    def __init__(self, host: str, port: int = 5002):
        self.host = host
        self.port = port
        self.ws = None
        self.seq = 1970  # 起始序号
        self.session_id = 0
        self.logged_in = False
        self.buffer = bytearray()
        self.lock = threading.Lock()
        
        # 回调
        self.on_system_state: Optional[Callable] = None
        self.on_login_response: Optional[Callable] = None
        
        # 存储状态
        self.current_map_name = None
        self.system_state_raw = None
        
    def connect(self):
        """连接到 WebSocket 服务器"""
        ws_url = f"ws://{self.host}:{self.port}"
        
        self.ws = websocket.WebSocketApp(
            ws_url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close
        )
        
        # 在后台线程运行
        self.wst = threading.Thread(target=lambda: self.ws.run_forever())
        self.wst.daemon = True
        self.wst.start()
    
    def _on_open(self, ws):
        pass  # 连接成功
    
    def _on_message(self, ws, message):
        """处理收到的消息"""
        if isinstance(message, bytes):
            with self.lock:
                self.buffer.extend(message)
                self._process_buffer()
    
    def _on_error(self, ws, error):
        pass  # print(f"WebSocket 错误: {error}")
    
    def _on_close(self, ws, close_status_code, close_msg):
        pass  # 连接关闭
    
    def _process_buffer(self):
        """处理缓冲区中的消息"""
        while True:
            msg, remaining = parse_frame(bytes(self.buffer))
            if msg is None:
                break
            self.buffer = bytearray(remaining)
            self._handle_message(msg)
    
    def _handle_message(self, data: bytes):
        """处理解析后的消息"""
        try:
            msg = decode_message(data)
            msg_type = msg.get('type', -1)
            
            # 提取 sessionId
            if 'sessionId' in msg and msg['sessionId'] > 0:
                if self.session_id == 0:
                    self.session_id = msg['sessionId']
            
            if msg_type == MessageType.MSG_RESPONSE:
                self._handle_response(msg, data)
            elif msg_type == MessageType.MSG_NOTIFICATION:
                self._handle_notification(msg, data)
                
        except Exception as e:
            print(f"处理消息错误: {e}")
            import traceback
            traceback.print_exc()
    
    def _handle_response(self, msg: dict, raw_data: bytes):
        """处理响应消息"""
        response_data = msg.get('response')
        if response_data:
            # 尝试提取地图名
            map_name = extract_map_name_from_bytes(response_data)
            if map_name:
                self.current_map_name = map_name
                self.system_state_raw = response_data
                
                if self.on_system_state:
                    self.on_system_state(response_data, map_name)
            
            # 检查是否登录成功
            if not self.logged_in and self.session_id > 0:
                self.logged_in = True
                if self.on_login_response:
                    self.on_login_response({'sessionId': self.session_id})
    
    def _handle_notification(self, msg: dict, raw_data: bytes):
        """处理通知消息"""
        notification_data = msg.get('notification')
        if notification_data:
            # 尝试提取地图名
            map_name = extract_map_name_from_bytes(notification_data)
            if map_name:
                self.current_map_name = map_name
                self.system_state_raw = notification_data
                
                if self.on_system_state:
                    self.on_system_state(notification_data, map_name)
    
    def login(self, username: str = "dev", password: str = "", 
               access: int = AccessAuthority.AA_ROOT):
        """发送登录请求"""
        if not self.ws:
            print("未连接")
            return
        
        self.seq += 1
        
        # 创建登录请求
        login_req = create_login_request(username, password, access)
        request = create_request_message(RequestType.REQUEST_LOGIN, login_req)
        message = create_message(
            MessageType.MSG_REQUEST, 
            self.seq, 
            self.session_id, 
            request
        )
        frame = build_frame(message)
        
        print(f"发送登录请求 (user={username}, access={access}, seq={self.seq})...")
        print(f"发送数据 ({len(frame)} bytes): {frame.hex(':')}")
        
        self.ws.send(frame, opcode=websocket.ABNF.OPCODE_BINARY)
    
    def send_heartbeat(self):
        """发送心跳请求"""
        if not self.ws or not self.logged_in:
            return
        
        self.seq += 1
        request = create_request_message(RequestType.REQUEST_HEARTBEAT)
        message = create_message(
            MessageType.MSG_REQUEST, 
            self.seq, 
            self.session_id, 
            request
        )
        frame = build_frame(message)
        
        self.ws.send(frame, opcode=websocket.ABNF.OPCODE_BINARY)
    
    def request_system_state(self):
        """请求系统状态"""
        if not self.ws or not self.logged_in:
            return
        
        self.seq += 1
        request = create_request_message(RequestType.REQUEST_SYSTEM_STATE)
        message = create_message(
            MessageType.MSG_REQUEST, 
            self.seq, 
            self.session_id, 
            request
        )
        frame = build_frame(message)
        
        print(f"请求系统状态 (seq={self.seq})...")
        self.ws.send(frame, opcode=websocket.ABNF.OPCODE_BINARY)
    
    def close(self):
        """关闭连接"""
        if self.ws:
            self.ws.close()


def get_current_map(host: str = "192.168.71.50",
                    port: int = 5002,
                    username: str = "dev",
                    password: str = "",
                    password_md5: str = None,
                    timeout: int = 10) -> Optional[str]:
    """
    获取当前地图名称
    
    Args:
        host: 机器人 IP
        port: WebSocket 端口 (默认 5002)
        username: 登录用户名 (默认 "dev")
        password: 登录密码 (默认空)
        password_md5: 密码的 MD5 哈希 (如果提供，直接使用而不计算)
        timeout: 超时时间秒
    
    Returns:
        当前地图名称，或 None
    """
    client = SRWebSocketClient(host, port)
    client.connect()
    time.sleep(0.5)
    
    # 创建登录请求
    if password_md5:
        login_req = create_login_request(username, password_md5, 
                                          AccessAuthority.AA_ROOT, 
                                          password_is_md5=True)
    else:
        login_req = create_login_request(username, password, 
                                          AccessAuthority.AA_ROOT)
    
    request = create_request_message(RequestType.REQUEST_LOGIN, login_req)
    client.seq += 1
    message = create_message(
        MessageType.MSG_REQUEST,
        client.seq,
        client.session_id,
        request
    )
    frame = build_frame(message)
    client.ws.send(frame, opcode=websocket.ABNF.OPCODE_BINARY)
    
    # 等待结果
    for i in range(timeout * 2):
        time.sleep(0.5)
        if client.current_map_name:
            result = client.current_map_name
            client.close()
            return result
    
    client.close()
    return None


def main():
    """主函数"""
    HOST = "192.168.71.50"
    PORT = 5002
    # 已知的密码 MD5 (从 Wireshark 抓包获取)
    # 如果你知道密码，可以用 hashlib.md5(password.encode()).hexdigest() 计算
    PASSWORD_MD5 = "902f36338ff60cbee8901c6f1ca45beb"
    
    print("=" * 60)
    print("Standard Robots WebSocket 客户端")
    print("=" * 60)
    
    # 方法1: 使用便捷函数
    print("\n使用便捷函数获取当前地图...")
    map_name = get_current_map(
        host=HOST,
        port=PORT,
        username="dev",
        password_md5=PASSWORD_MD5
    )
    
    if map_name:
        print(f"✓ 当前地图: {map_name}")
    else:
        print("✗ 未能获取地图名称")
    
    print("=" * 60)


if __name__ == '__main__':
    main()
