#!/usr/bin/env python3
"""
头部舵机控制启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('qyh_head_control')
    config_file = os.path.join(pkg_dir, 'config', 'head_params.yaml')
    
    # 声明启动参数
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for servo communication'
    )
    
    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )
    
    declare_use_config = DeclareLaunchArgument(
        'use_config',
        default_value='true',
        description='Use parameters from config file'
    )
    
    # 头部舵机控制节点
    head_servo_node = Node(
        package='qyh_head_control',
        executable='head_servo_node',
        name='head_servo_node',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baudrate,
        declare_use_config,
        head_servo_node,
    ])
