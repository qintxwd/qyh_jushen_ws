#!/usr/bin/env python3
"""
Standard Robots Matrix API 测试脚本
用于连接 Matrix 系统、登录并获取地图
"""

import os
import sys
import yaml
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from sr_matrix import MatrixClient, MapManager


def load_config(config_path: str = None) -> dict:
    """加载配置文件"""
    if config_path is None:
        config_path = project_root / "config.yaml"
    
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    """主函数"""
    print("=" * 60)
    print("Standard Robots Matrix API 测试")
    print("=" * 60)
    
    # 加载配置
    config = load_config()
    matrix_config = config["matrix"]
    
    host = matrix_config["host"]
    port = matrix_config.get("port", 80)
    username = matrix_config["username"]
    password = matrix_config["password"]
    timeout = matrix_config.get("timeout", 10)
    map_save_path = matrix_config.get("map_save_path", "./maps")
    
    print(f"\n目标主机: {host}:{port}")
    print(f"用户名: {username}")
    print(f"地图保存路径: {map_save_path}")
    
    # 创建客户端
    client = MatrixClient(host=host, port=port, timeout=timeout)
    
    # 1. 检查连接
    print("\n" + "-" * 40)
    print("步骤 1: 检查连接")
    if not client.check_connection():
        print("❌ 无法连接到 Matrix 系统")
        print("请检查:")
        print(f"  1. 机器人是否开机")
        print(f"  2. IP 地址是否正确: {host}")
        print(f"  3. 网络是否可达")
        return 1
    print("✅ 连接正常")
    
    # 2. 探索 API
    print("\n" + "-" * 40)
    print("步骤 2: 探索 API")
    discovered = client.explore_api()
    if discovered["working_endpoints"]:
        print("发现的端点:")
        for ep in discovered["working_endpoints"]:
            print(f"  - {ep['endpoint']}: {ep['content_type']}")
    
    # 3. 登录
    print("\n" + "-" * 40)
    print("步骤 3: 登录")
    if not client.login(username, password):
        print("❌ 登录失败")
        return 1
    
    # 4. 获取地图
    print("\n" + "-" * 40)
    print("步骤 4: 获取地图")
    
    map_manager = MapManager(client, save_path=map_save_path)
    
    # 获取地图列表
    print("\n获取地图列表...")
    maps = map_manager.list_maps()
    if maps:
        print(f"找到 {len(maps)} 个地图:")
        for m in maps:
            print(f"  - {m.get('name', m.get('mapName', 'unknown'))}")
    
    # 获取当前地图
    print("\n获取当前地图...")
    current_map = map_manager.get_current_map_info()
    if current_map:
        print(f"当前地图: {current_map}")
    
    # 下载当前地图
    print("\n下载当前地图...")
    saved_path = map_manager.download_current_map()
    if saved_path:
        print(f"✅ 地图已保存到: {saved_path}")
    
    # 获取站点
    print("\n获取站点列表...")
    sites = map_manager.get_sites_for_map()
    if sites:
        print(f"找到 {len(sites)} 个站点:")
        for site in sites[:10]:  # 只显示前 10 个
            print(f"  - {site.get('name', site.get('siteName', 'unknown'))}: "
                  f"({site.get('x', '?')}, {site.get('y', '?')})")
        map_manager.save_sites()
    
    # 5. 获取机器人状态
    print("\n" + "-" * 40)
    print("步骤 5: 获取机器人状态")
    
    status = client.get_robot_status()
    if status:
        print(f"机器人状态: {status}")
    
    pose = client.get_robot_pose()
    if pose:
        print(f"机器人位姿: {pose}")
    
    # 登出
    client.logout()
    
    print("\n" + "=" * 60)
    print("测试完成!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
