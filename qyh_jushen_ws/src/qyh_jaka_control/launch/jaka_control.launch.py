#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('qyh_jaka_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='JAKA robot IP address'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to robot configuration YAML file'
        ),
        DeclareLaunchArgument(
            'auto_initialize',
            default_value='false',
            description='Auto: power on, enable, init pose, start VR'
        ),
        DeclareLaunchArgument(
            'cycle_time_ms',
            default_value='8.0',
            description='Control cycle time in ms (8.0ms = 125Hz)'
        ),
        DeclareLaunchArgument(
            'buffer_size',
            default_value='10',
            description='Smooth servo bridge buffer size'
        ),
        DeclareLaunchArgument(
            'interpolation_weight',
            default_value='0.5',
            description='Smooth servo bridge interpolation weight'
        ),

        # JAKA完整控制节点（基础+伺服+VR）
        Node(
            package='qyh_jaka_control',
            executable='jaka_control_node',
            name='jaka_control_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'auto_initialize': LaunchConfiguration('auto_initialize'),
                    'cycle_time_ms': LaunchConfiguration('cycle_time_ms'),
                    'buffer_size': LaunchConfiguration('buffer_size'),
                    'interpolation_weight': LaunchConfiguration('interpolation_weight')
                }
            ]
        )
    ])
