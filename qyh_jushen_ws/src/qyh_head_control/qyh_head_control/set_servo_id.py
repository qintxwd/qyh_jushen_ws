#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
舵机ID设置工具
用于设置HTD-85H舵机的ID
"""

import sys
import os
import time

# 添加 SDK 路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sdk_path = os.path.join(current_dir, 'sdk')
if sdk_path not in sys.path:
    sys.path.insert(0, sdk_path)

try:
    from sdk.hiwonder_servo_controller import HiwonderServoController
except ImportError as e:
    print(f"错误：无法导入SDK库: {e}")
    print(f"SDK路径: {sdk_path}")
    sys.exit(1)


def detect_servo(port='/dev/ttyTHS1', baudrate=115200):
    """检测当前连接的舵机"""
    try:
        servo_control = HiwonderServoController(port, baudrate)
        time.sleep(0.5)
        
        print("\n正在扫描舵机...")
        found_servos = []
        
        # 尝试读取ID 1-10
        for servo_id in range(1, 11):
            try:
                pos = servo_control.get_servo_position(servo_id)
                if pos is not None:
                    voltage = servo_control.get_servo_vin(servo_id)
                    temp = servo_control.get_servo_temp(servo_id)
                    angle_range = servo_control.get_servo_range(servo_id)
                    
                    found_servos.append({
                        'id': servo_id,
                        'position': pos,
                        'voltage': voltage,
                        'temp': temp,
                        'range': angle_range
                    })
                    print(f"  ✓ 发现舵机 ID={servo_id}: 位置={pos}, 电压={voltage}mV, 温度={temp}℃")
            except:
                pass
        
        if not found_servos:
            print("  ✗ 未检测到任何舵机")
        
        return servo_control, found_servos
    
    except Exception as e:
        print(f"错误：无法连接串口 - {e}")
        return None, []


def set_servo_id(servo_control, old_id, new_id):
    """设置舵机ID"""
    try:
        print(f"\n正在将舵机ID从 {old_id} 修改为 {new_id}...")
        
        # 读取当前位置确认舵机存在
        pos = servo_control.get_servo_position(old_id)
        if pos is None:
            print(f"  ✗ 错误：未找到ID={old_id}的舵机")
            return False
        
        print(f"  当前舵机位置: {pos}")
        
        # 设置新ID
        result = servo_control.set_servo_id(old_id, new_id)
        time.sleep(1)
        
        # 验证新ID
        new_pos = servo_control.get_servo_position(new_id)
        if new_pos is not None:
            print(f"  ✓ 成功！舵机ID已修改为 {new_id}")
            print(f"  验证位置: {new_pos}")
            return True
        else:
            print(f"  ✗ 失败：无法验证新ID={new_id}")
            return False
    
    except Exception as e:
        print(f"  ✗ 设置ID失败: {e}")
        return False


def interactive_mode(port='/dev/ttyTHS1', baudrate=115200):
    """交互式设置模式"""
    print("=" * 60)
    print("        HTD-85H 舵机ID设置工具")
    print("=" * 60)
    
    # 检测舵机
    servo_control, found_servos = detect_servo(port, baudrate)
    
    if servo_control is None:
        return
    
    if not found_servos:
        print("\n提示：")
        print("  1. 请确认只连接了一个舵机")
        print("  2. 检查舵机供电是否正常 (6-12V)")
        print("  3. 检查串口连接是否正确")
        print("  4. 舵机可能使用了非标准ID，请尝试手动指定")
        print("\n是否手动指定旧ID? (y/n): ", end='')
        choice = input().strip().lower()
        if choice != 'y':
            return
    
    print("\n" + "=" * 60)
    print("请选择操作:")
    print("  1 - 设置舵机ID为 1 (Pan左右转动)")
    print("  2 - 设置舵机ID为 2 (Tilt上下转动)")
    print("  3 - 自定义ID设置")
    print("  q - 退出")
    print("=" * 60)
    
    choice = input("\n请输入选项 (1/2/3/q): ").strip()
    
    if choice == 'q':
        print("退出")
        return
    
    elif choice == '1':
        if found_servos:
            old_id = found_servos[0]['id']
        else:
            old_id = int(input("请输入当前舵机ID (通常是1): ").strip())
        
        print("\n⚠️  重要提示:")
        print("  - 确保这是要设置为Pan(左右转动)的舵机")
        print("  - 确保只连接了这一个舵机")
        confirm = input("  确认设置为ID=1? (y/n): ").strip().lower()
        
        if confirm == 'y':
            success = set_servo_id(servo_control, old_id, 1)
            if success:
                print("\n✓ Pan舵机ID设置完成！")
                print("  请断开这个舵机，连接下一个舵机")
    
    elif choice == '2':
        if found_servos:
            old_id = found_servos[0]['id']
        else:
            old_id = int(input("请输入当前舵机ID (通常是1): ").strip())
        
        print("\n⚠️  重要提示:")
        print("  - 确保这是要设置为Tilt(上下转动)的舵机")
        print("  - 确保只连接了这一个舵机")
        print("  - 如果两个舵机都是ID=1，必须先断开第一个舵机")
        confirm = input("  确认设置为ID=2? (y/n): ").strip().lower()
        
        if confirm == 'y':
            success = set_servo_id(servo_control, old_id, 2)
            if success:
                print("\n✓ Tilt舵机ID设置完成！")
                print("  现在可以连接两个舵机一起使用了")
    
    elif choice == '3':
        if found_servos:
            print(f"\n当前检测到的舵机ID: {[s['id'] for s in found_servos]}")
            old_id = int(input("请输入要修改的舵机ID: ").strip())
        else:
            old_id = int(input("请输入当前舵机ID: ").strip())
        
        new_id = int(input("请输入新的舵机ID (1-253): ").strip())
        
        if new_id < 1 or new_id > 253:
            print("错误：ID必须在1-253之间")
            return
        
        confirm = input(f"确认将ID {old_id} 改为 {new_id}? (y/n): ").strip().lower()
        if confirm == 'y':
            set_servo_id(servo_control, old_id, new_id)
    
    else:
        print("无效选项")


def batch_mode(old_id, new_id, port='/dev/ttyTHS1', baudrate=115200):
    """批处理模式（用于launch文件）"""
    print(f"批处理模式：将舵机ID从 {old_id} 修改为 {new_id}")
    
    servo_control, found_servos = detect_servo(port, baudrate)
    
    if servo_control is None:
        sys.exit(1)
    
    success = set_servo_id(servo_control, old_id, new_id)
    
    if success:
        print(f"\n✓ 舵机ID设置成功: {old_id} -> {new_id}")
        sys.exit(0)
    else:
        print(f"\n✗ 舵机ID设置失败")
        sys.exit(1)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='HTD-85H舵机ID设置工具')
    parser.add_argument('--port', default='/dev/ttyTHS1', help='串口设备')
    parser.add_argument('--baudrate', type=int, default=115200, help='波特率')
    parser.add_argument('--old-id', type=int, help='当前舵机ID')
    parser.add_argument('--new-id', type=int, help='新的舵机ID')
    parser.add_argument('--batch', action='store_true', help='批处理模式（用于自动化）')
    
    args = parser.parse_args()
    
    if args.batch:
        if args.old_id is None or args.new_id is None:
            print("错误：批处理模式需要指定 --old-id 和 --new-id")
            sys.exit(1)
        batch_mode(args.old_id, args.new_id, args.port, args.baudrate)
    else:
        interactive_mode(args.port, args.baudrate)


if __name__ == '__main__':
    main()
