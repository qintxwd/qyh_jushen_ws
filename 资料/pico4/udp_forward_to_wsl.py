#!/usr/bin/env python3
"""
UDP 转发器：Windows -> WSL
在 Windows 上运行，接收 PICO VR 的 UDP 广播，转发到 WSL

Usage:
    1. 先获取 WSL IP: wsl hostname -I
    2. python udp_forward_to_wsl.py <WSL_IP>
    
Example:
    python udp_forward_to_wsl.py 172.25.123.45
"""

import socket
import sys

LISTEN_PORT = 9999      # 监听端口 (接收 PICO VR 数据)
FORWARD_PORT = 9999     # 转发端口 (发送到 WSL)

def main():
    if len(sys.argv) < 2:
        # 尝试自动获取 WSL IP
        import subprocess
        try:
            result = subprocess.run(['wsl', 'hostname', '-I'], capture_output=True, text=True)
            wsl_ip = result.stdout.strip().split()[0]
            print(f"自动检测到 WSL IP: {wsl_ip}")
        except:
            print("用法: python udp_forward_to_wsl.py <WSL_IP>")
            print("获取 WSL IP: wsl hostname -I")
            sys.exit(1)
    else:
        wsl_ip = sys.argv[1]
    
    print("=" * 60)
    print("UDP 转发器: Windows -> WSL")
    print("=" * 60)
    print(f"监听端口: {LISTEN_PORT}")
    print(f"转发目标: {wsl_ip}:{FORWARD_PORT}")
    print("=" * 60)
    
    # 创建接收 socket
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind(('0.0.0.0', LISTEN_PORT))
    
    # 创建发送 socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"\n正在监听... (Ctrl+C 退出)")
    
    packet_count = 0
    try:
        while True:
            data, addr = recv_sock.recvfrom(2048)
            
            # 转发到 WSL
            send_sock.sendto(data, (wsl_ip, FORWARD_PORT))
            
            packet_count += 1
            if packet_count % 100 == 0:
                print(f"\r已转发 {packet_count} 个数据包 (来自 {addr[0]})", end="", flush=True)
                
    except KeyboardInterrupt:
        print(f"\n\n停止转发，共转发 {packet_count} 个数据包")
    finally:
        recv_sock.close()
        send_sock.close()

if __name__ == "__main__":
    main()
