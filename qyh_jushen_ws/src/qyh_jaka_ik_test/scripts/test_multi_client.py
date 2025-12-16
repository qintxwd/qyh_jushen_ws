#!/usr/bin/env python3
"""
简单的多客户端连接测试脚本（Python版本）
不需要编译，直接运行

测试步骤：
1. 终端1: ros2 launch qyh_jaka_control jaka_control.launch.py
2. 终端2: python3 test_multi_client.py
"""

import socket
import time
import sys

def test_connection(robot_ip, test_id):
    """尝试建立TCP连接到JAKA控制器"""
    print(f"\n{'='*60}")
    print(f"测试 #{test_id}: 尝试连接到 {robot_ip}")
    print(f"{'='*60}")
    
    # JAKA控制器通常使用端口10000
    port = 10000
    timeout = 5
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        
        print(f"正在连接 {robot_ip}:{port}...")
        sock.connect((robot_ip, port))
        
        print(f"✅ 连接成功！")
        print(f"   - 本地地址: {sock.getsockname()}")
        print(f"   - 远程地址: {sock.getpeername()}")
        
        # 保持连接一段时间
        print(f"\n保持连接10秒...")
        for i in range(10):
            time.sleep(1)
            print(f"  {i+1}/10秒", end="\r")
        
        print("\n\n断开连接...")
        sock.close()
        print("✅ 测试完成")
        return True
        
    except socket.timeout:
        print(f"❌ 连接超时（{timeout}秒）")
        return False
    except ConnectionRefusedError:
        print(f"❌ 连接被拒绝")
        print(f"   可能原因：")
        print(f"   1. 控制器未启动")
        print(f"   2. 端口错误（JAKA默认10000）")
        return False
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return False
    finally:
        try:
            sock.close()
        except:
            pass

def main():
    robot_ip = "192.168.2.200"
    
    if len(sys.argv) > 1:
        robot_ip = sys.argv[1]
    
    print("\n" + "="*60)
    print("  JAKA多客户端连接测试")
    print("="*60)
    print(f"目标IP: {robot_ip}")
    print(f"目标端口: 10000 (JAKA默认)")
    print("\n测试说明：")
    print("1. 这个脚本会尝试建立第二个TCP连接")
    print("2. 如果qyh_jaka_control已经连接，可以验证多客户端支持")
    print("3. 不会发送实际控制命令，仅测试连接")
    print()
    
    input("按Enter开始测试...")
    
    # 第一次测试
    success1 = test_connection(robot_ip, 1)
    
    if not success1:
        print("\n⚠️  第一次连接失败，请检查：")
        print("   1. 网络连接: ping", robot_ip)
        print("   2. 控制器状态")
        print("   3. IP地址是否正确")
        return
    
    print("\n" + "="*60)
    print("第一次测试成功！现在测试同时连接...")
    print("="*60)
    input("按Enter继续...")
    
    # 第二次测试（应该在第一个还连接时）
    success2 = test_connection(robot_ip, 2)
    
    print("\n" + "="*60)
    print("测试结果总结")
    print("="*60)
    if success1 and success2:
        print("✅ 测试1: 成功")
        print("✅ 测试2: 成功")
        print()
        print("🎉 SDK支持多客户端连接！")
        print()
        print("📌 下一步：可以创建独立的IK求解节点")
    else:
        print("⚠️  多客户端连接可能不被支持")
        print()
        print("📌 建议：在qyh_jaka_control内集成IK功能")

if __name__ == "__main__":
    main()
