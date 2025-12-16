#!/usr/bin/env python3
"""
IK多客户端连接测试

启动方式：
  终端1: ros2 launch qyh_jaka_control jaka_control.launch.py
  终端2: ros2 launch qyh_jaka_ik_test ik_test.launch.py

观察两个节点是否都能正常连接和工作
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('qyh_jaka_ik_test')
    config_file = os.path.join(pkg_share, 'config', 'ik_test_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='JAKA robot IP (same as qyh_jaka_control)'
        ),
        DeclareLaunchArgument(
            'ik_test_rate',
            default_value='100.0',
            description='IK test frequency in Hz'
        ),
        
        Node(
            package='qyh_jaka_ik_test',
            executable='ik_test_node',
            name='ik_test_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'ik_test_rate': LaunchConfiguration('ik_test_rate'),
                }
            ]
        ),
    ])
