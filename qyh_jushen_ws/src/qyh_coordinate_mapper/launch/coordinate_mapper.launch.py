#!/usr/bin/env python3
"""
坐标映射与滤波节点Launch文件

使用方式：
  终端1: ros2 launch qyh_vr_bridge vr_bridge.launch.py
  终端2: ros2 launch qyh_coordinate_mapper coordinate_mapper.launch.py
  终端3: ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('qyh_coordinate_mapper')
    config_file = os.path.join(pkg_share, 'config', 'mapper_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'update_rate',
            default_value='100.0',
            description='Coordinate mapping update rate in Hz'
        ),
        DeclareLaunchArgument(
            'position_scale',
            default_value='1.0',
            description='Position scaling factor'
        ),
        DeclareLaunchArgument(
            'filter_alpha',
            default_value='0.3',
            description='Low-pass filter coefficient (0-1)'
        ),
        
        Node(
            package='qyh_coordinate_mapper',
            executable='coordinate_mapper_node',
            name='coordinate_mapper',
            output='screen',
            parameters=[
                config_file,
                {
                    'update_rate': LaunchConfiguration('update_rate'),
                    'position_scale': LaunchConfiguration('position_scale'),
                    'filter_alpha': LaunchConfiguration('filter_alpha'),
                }
            ]
        ),
    ])
