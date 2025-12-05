#!/usr/bin/env python3
"""
UDP Controller Data Receiver
接收来自 PICO VR 控制器的 UDP 广播数据

Usage:
    python udp_receiver.py
"""

import socket
import struct
import sys
from datetime import datetime

# 数据包格式定义 (与 C++ #pragma pack(1) 结构体匹配)
# q = int64 (8 bytes) - timestamp
# 3f = 3 floats (12 bytes) - head_position
# 4f = 4 floats (16 bytes) - head_orientation
# B = uint8 (1 byte)  - left_active
# 3f = 3 floats (12 bytes) - left_position
# 4f = 4 floats (16 bytes) - left_orientation
# 2f = 2 floats (8 bytes) - left_joystick
# f = float (4 bytes) - left_trigger
# f = float (4 bytes) - left_grip
# B = uint8 (1 byte)  - right_active
# 3f = 3 floats (12 bytes) - right_position
# 4f = 4 floats (16 bytes) - right_orientation
# 2f = 2 floats (8 bytes) - right_joystick
# f = float (4 bytes) - right_trigger
# f = float (4 bytes) - right_grip
# I = uint32 (4 bytes) - buttons_bitmask
# I = uint32 (4 bytes) - touches_bitmask
# Total: 8 + 12 + 16 + 1 + 12 + 16 + 8 + 4 + 4 + 1 + 12 + 16 + 8 + 4 + 4 + 4 + 4 = 134 bytes

PACKET_FORMAT = '<q3f4fB3f4f2fffB3f4f2fffII'  # < = little-endian
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
UDP_PORT = 9999

# 按钮位掩码定义 (来自 XrFrameIn)
BUTTON_A = 1 << 0
BUTTON_B = 1 << 1
BUTTON_X = 1 << 2
BUTTON_Y = 1 << 3
BUTTON_MENU = 1 << 4
BUTTON_HOME = 1 << 5
BUTTON_GRIP_TRIGGER_LEFT = 1 << 6
BUTTON_GRIP_TRIGGER_RIGHT = 1 << 7
BUTTON_TRIGGER_LEFT = 1 << 8
BUTTON_TRIGGER_RIGHT = 1 << 9
BUTTON_JOYSTICK_LEFT = 1 << 10
BUTTON_JOYSTICK_RIGHT = 1 << 11

# 触摸位掩码定义
TOUCH_JOYSTICK_LEFT = 1 << 0
TOUCH_JOYSTICK_RIGHT = 1 << 1
TOUCH_TRIGGER_LEFT = 1 << 2
TOUCH_TRIGGER_RIGHT = 1 << 3


def decode_buttons(bitmask):
    """解码按钮位掩码"""
    buttons = []
    if bitmask & BUTTON_A: buttons.append("A")
    if bitmask & BUTTON_B: buttons.append("B")
    if bitmask & BUTTON_X: buttons.append("X")
    if bitmask & BUTTON_Y: buttons.append("Y")
    if bitmask & BUTTON_MENU: buttons.append("Menu")
    if bitmask & BUTTON_HOME: buttons.append("Home")
    if bitmask & BUTTON_GRIP_TRIGGER_LEFT: buttons.append("GripTrig-L")
    if bitmask & BUTTON_GRIP_TRIGGER_RIGHT: buttons.append("GripTrig-R")
    if bitmask & BUTTON_TRIGGER_LEFT: buttons.append("Trig-L")
    if bitmask & BUTTON_TRIGGER_RIGHT: buttons.append("Trig-R")
    if bitmask & BUTTON_JOYSTICK_LEFT: buttons.append("Joy-L")
    if bitmask & BUTTON_JOYSTICK_RIGHT: buttons.append("Joy-R")
    return buttons if buttons else ["None"]


def decode_touches(bitmask):
    """解码触摸位掩码"""
    touches = []
    if bitmask & TOUCH_JOYSTICK_LEFT: touches.append("Joy-L")
    if bitmask & TOUCH_JOYSTICK_RIGHT: touches.append("Joy-R")
    if bitmask & TOUCH_TRIGGER_LEFT: touches.append("Trigger-L")
    if bitmask & TOUCH_TRIGGER_RIGHT: touches.append("Trigger-R")
    return touches if touches else ["None"]


def main():
    print("=" * 70)
    print("PICO VR Controller Data UDP Receiver")
    print("=" * 70)
    print(f"Listening on UDP port {UDP_PORT}")
    print(f"Expected packet size: {PACKET_SIZE} bytes")
    print(f"Packet format: {PACKET_FORMAT}")
    print("=" * 70)
    print()
    
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('', UDP_PORT))
    except OSError as e:
        print(f"Error: Failed to bind to port {UDP_PORT}")
        print(f"       {e}")
        print(f"       Port may already be in use.")
        sys.exit(1)
    
    print("Waiting for data...\n")
    
    packet_count = 0
    
    try:
        while True:
            data, addr = sock.recvfrom(2048)
            
            if len(data) != PACKET_SIZE:
                print(f"Warning: Received {len(data)} bytes, expected {PACKET_SIZE} bytes")
                print(f"         Raw data: {data.hex()[:100]}...")
                continue
            
            # 解包数据
            values = struct.unpack(PACKET_FORMAT, data)
            
            # 解析数据
            idx = 0
            timestamp = values[idx]; idx += 1
            
            # 头部位姿
            head_pos = values[idx:idx+3]; idx += 3
            head_ori = values[idx:idx+4]; idx += 4

            # 左手柄
            left_active = values[idx]; idx += 1
            left_pos = values[idx:idx+3]; idx += 3
            left_ori = values[idx:idx+4]; idx += 4
            left_joy = values[idx:idx+2]; idx += 2
            left_trigger = values[idx]; idx += 1
            left_grip = values[idx]; idx += 1
            
            # 右手柄
            right_active = values[idx]; idx += 1
            right_pos = values[idx:idx+3]; idx += 3
            right_ori = values[idx:idx+4]; idx += 4
            right_joy = values[idx:idx+2]; idx += 2
            right_trigger = values[idx]; idx += 1
            right_grip = values[idx]; idx += 1
            
            # 按钮和触摸
            buttons_bitmask = values[idx]; idx += 1
            touches_bitmask = values[idx]; idx += 1
            
            packet_count += 1
            
            # 清屏并显示数据 (每10个包刷新一次以避免太快)
            if packet_count % 3 == 0:
                print("\033[2J\033[H", end="")  # 清屏
                print(f"{'=' * 70}")
                print(f"Packet #{packet_count} | From: {addr[0]}:{addr[1]}")
                print(f"Timestamp: {timestamp} ms | {datetime.fromtimestamp(timestamp/1000.0).strftime('%H:%M:%S.%f')[:-3]}")
                print(f"{'-' * 70}")
                
                # 头部位姿
                print(f"HEAD Pose:")
                print(f"  Position:    ({head_pos[0]:8.4f}, {head_pos[1]:8.4f}, {head_pos[2]:8.4f}) m")
                print(f"  Orientation: ({head_ori[0]:7.4f}, {head_ori[1]:7.4f}, {head_ori[2]:7.4f}, {head_ori[3]:7.4f})")
                print(f"{'-' * 70}")

                # 左手柄信息
                print(f"LEFT Controller:  {'ACTIVE' if left_active else 'INACTIVE'}")
                if left_active:
                    print(f"  Position:    ({left_pos[0]:8.4f}, {left_pos[1]:8.4f}, {left_pos[2]:8.4f}) m")
                    print(f"  Orientation: ({left_ori[0]:7.4f}, {left_ori[1]:7.4f}, {left_ori[2]:7.4f}, {left_ori[3]:7.4f})")
                    print(f"  Joystick:    ({left_joy[0]:6.3f}, {left_joy[1]:6.3f})")
                    print(f"  Trigger:     {left_trigger:5.3f}  |  Grip: {left_grip:5.3f}")
                
                print(f"{'-' * 70}")
                
                # 右手柄信息
                print(f"RIGHT Controller: {'ACTIVE' if right_active else 'INACTIVE'}")
                if right_active:
                    print(f"  Position:    ({right_pos[0]:8.4f}, {right_pos[1]:8.4f}, {right_pos[2]:8.4f}) m")
                    print(f"  Orientation: ({right_ori[0]:7.4f}, {right_ori[1]:7.4f}, {right_ori[2]:7.4f}, {right_ori[3]:7.4f})")
                    print(f"  Joystick:    ({right_joy[0]:6.3f}, {right_joy[1]:6.3f})")
                    print(f"  Trigger:     {right_trigger:5.3f}  |  Grip: {right_grip:5.3f}")
                
                print(f"{'-' * 70}")
                
                # 按钮和触摸状态
                buttons = decode_buttons(buttons_bitmask)
                touches = decode_touches(touches_bitmask)
                print(f"Buttons Pressed: {', '.join(buttons)}")
                print(f"Touches Active:  {', '.join(touches)}")
                print(f"{'=' * 70}")
            
    except KeyboardInterrupt:
        print("\n\nReceived keyboard interrupt. Shutting down...")
    finally:
        sock.close()
        print(f"Total packets received: {packet_count}")
        print("UDP receiver closed.")


if __name__ == "__main__":
    main()
