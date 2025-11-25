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
            'config_file',
            default_value=config_file,
            description='Path to robot configuration YAML file'
        ),
        DeclareLaunchArgument(
            'auto_initialize',
            default_value='false',
            description='Auto: power on, enable, init pose, start VR'
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
                    'auto_initialize': LaunchConfiguration('auto_initialize'),
                }
            ],
        ),
    ])
