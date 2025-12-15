#!/usr/bin/env python3
"""
启动VR到base_vr的数据桥接节点

此launch文件启动VR数据接收节点，该节点：
1. 监听UDP端口接收PICO 4 VR数据
2. 在base_vr坐标系下发布vr_left和vr_right两个TF frame
3. VR手柄坐标直接对应base_vr的坐标系（X右 Y上 Z后）

使用方法:
    ros2 launch dual_arms vr_bridge.launch.py
    
参数:
    udp_port: UDP监听端口，默认9999
    publish_rate: TF发布频率，默认100 Hz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成launch描述"""
    
    # 声明参数
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='9999',
        description='UDP port for receiving VR data'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='TF publish rate in Hz'
    )
    
    # VR到base_vr的桥接节点
    vr_bridge_node = Node(
        package='dual_arms',
        executable='vr_to_base_vr.py',
        name='vr_to_base_vr',
        output='screen',
        parameters=[{
            'udp_port': LaunchConfiguration('udp_port'),
            'publish_rate': LaunchConfiguration('publish_rate')
        }]
    )
    
    return LaunchDescription([
        udp_port_arg,
        publish_rate_arg,
        vr_bridge_node
    ])
