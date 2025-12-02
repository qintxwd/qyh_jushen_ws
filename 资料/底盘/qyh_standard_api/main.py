#!/usr/bin/env python3
"""
Standard Robots Matrix API 工具

命令行工具，用于与 Standard Robots Matrix 系统交互。

用法:
    python main.py list                    # 列出所有地图
    python main.py download [map_name]     # 下载地图（不指定则下载全部）
    python main.py info <map_name>         # 显示地图详情
    python main.py stations <map_name>     # 显示工位列表
"""

import sys
import argparse
import logging
from pathlib import Path
import yaml

from matrix import MatrixClient, download_maps, list_maps


def load_config():
    """加载配置文件"""
    config_path = Path(__file__).parent / 'config.yaml'
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return {
        'matrix': {
            'host': '192.168.71.50',
            'port': 80
        }
    }


def cmd_list(args, client: MatrixClient):
    """列出所有地图"""
    maps = client.get_map_list()
    print(f"找到 {len(maps)} 个地图:\n")
    for m in maps:
        print(f"  📍 {m.name}")
        print(f"     修改时间: {m.modify_time}")
        print(f"     MD5: {m.md5[:16]}...")
        print()


def cmd_download(args, client: MatrixClient):
    """下载地图"""
    output_dir = Path(args.output)
    
    if args.map_name:
        # 下载指定地图
        print(f"正在下载地图: {args.map_name}")
        result = client.download_map(args.map_name, output_dir)
        print(f"✓ JSON: {result['json']}")
        print(f"✓ 图片: {result['image']}")
    else:
        # 下载所有地图
        print("正在下载所有地图...")
        result = client.download_all_maps(output_dir)
        print(f"\n下载完成! 共 {len(result)} 个地图")
        for name, files in result.items():
            print(f"  ✓ {name}")


def cmd_info(args, client: MatrixClient):
    """显示地图详情"""
    map_name = args.map_name
    
    print(f"地图: {map_name}")
    print("=" * 50)
    
    # 元数据
    meta = client.get_map_meta(map_name)
    print(f"\n【元数据】")
    print(f"  版本: {meta.version}")
    print(f"  分辨率: {meta.resolution} mm/pixel")
    print(f"  尺寸: {meta.width} x {meta.height} pixels")
    print(f"  实际尺寸: {meta.width * meta.resolution / 1000:.1f} x {meta.height * meta.resolution / 1000:.1f} m")
    print(f"  原点: ({meta.origin_x:.1f}, {meta.origin_y:.1f}) mm")
    
    # 节点
    nodes = client.get_map_nodes(map_name)
    print(f"\n【导航节点】共 {len(nodes)} 个")
    
    # 边
    edges = client.get_map_edges(map_name)
    print(f"\n【路径边】共 {len(edges)} 条")
    
    # 站点
    stations = client.get_map_stations(map_name)
    print(f"\n【工位/站点】共 {len(stations)} 个")
    if stations and not args.brief:
        for s in stations[:5]:
            print(f"  - {s.name} (id={s.id}, x={s.pos_x:.0f}, y={s.pos_y:.0f})")
        if len(stations) > 5:
            print(f"  ... 还有 {len(stations) - 5} 个站点")


def cmd_stations(args, client: MatrixClient):
    """显示工位列表"""
    map_name = args.map_name
    stations = client.get_map_stations(map_name)
    
    print(f"地图 '{map_name}' 的工位列表 (共 {len(stations)} 个):\n")
    
    print(f"{'ID':>6} {'名称':<20} {'类型':<10} {'X(mm)':>10} {'Y(mm)':>10}")
    print("-" * 60)
    
    for s in stations:
        name = s.name[:18] if len(s.name) > 18 else s.name
        print(f"{s.id:>6} {name:<20} {s.station_type:<10} {s.pos_x:>10.0f} {s.pos_y:>10.0f}")


def cmd_nodes(args, client: MatrixClient):
    """显示节点列表"""
    map_name = args.map_name
    nodes = client.get_map_nodes(map_name)
    
    print(f"地图 '{map_name}' 的节点列表 (共 {len(nodes)} 个):\n")
    
    if args.limit:
        nodes = nodes[:args.limit]
    
    print(f"{'ID':>6} {'X(mm)':>12} {'Y(mm)':>12} {'Yaw(rad)':>10} {'描述'}")
    print("-" * 60)
    
    for n in nodes:
        print(f"{n.id:>6} {n.x:>12.1f} {n.y:>12.1f} {n.yaw:>10.3f} {n.desc}")


def main():
    parser = argparse.ArgumentParser(
        description='Standard Robots Matrix API 工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument('--host', help='Matrix 服务器地址')
    parser.add_argument('--port', type=int, help='HTTP 端口')
    parser.add_argument('-v', '--verbose', action='store_true', help='详细输出')
    
    subparsers = parser.add_subparsers(dest='command', help='子命令')
    
    # list 命令
    list_parser = subparsers.add_parser('list', help='列出所有地图')
    
    # download 命令
    dl_parser = subparsers.add_parser('download', help='下载地图')
    dl_parser.add_argument('map_name', nargs='?', help='地图名称（不指定则下载全部）')
    dl_parser.add_argument('-o', '--output', default='./maps', help='输出目录')
    
    # info 命令
    info_parser = subparsers.add_parser('info', help='显示地图详情')
    info_parser.add_argument('map_name', help='地图名称')
    info_parser.add_argument('-b', '--brief', action='store_true', help='简要模式')
    
    # stations 命令
    st_parser = subparsers.add_parser('stations', help='显示工位列表')
    st_parser.add_argument('map_name', help='地图名称')
    
    # nodes 命令
    nd_parser = subparsers.add_parser('nodes', help='显示节点列表')
    nd_parser.add_argument('map_name', help='地图名称')
    nd_parser.add_argument('-n', '--limit', type=int, help='限制显示数量')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # 配置日志
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)
    
    # 加载配置
    config = load_config()
    host = args.host or config['matrix']['host']
    port = args.port or config['matrix'].get('port', 80)
    
    # 创建客户端
    client = MatrixClient(host, port)
    
    # 执行命令
    commands = {
        'list': cmd_list,
        'download': cmd_download,
        'info': cmd_info,
        'stations': cmd_stations,
        'nodes': cmd_nodes,
    }
    
    try:
        commands[args.command](args, client)
    except Exception as e:
        print(f"错误: {e}")
        if args.verbose:
            raise
        sys.exit(1)


if __name__ == '__main__':
    main()
