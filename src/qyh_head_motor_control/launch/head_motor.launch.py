#!/usr/bin/env python3
"""
Head Motor Control Launch File
头部电机控制启动文件
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('qyh_head_motor_control')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM_steering_gear',
        description='Serial port for motor communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1000000',
        description='Serial baudrate'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'head_motor_params.yaml'),
        description='Path to configuration file'
    )
    
    # Head motor node
    head_motor_node = Node(
        package='qyh_head_motor_control',
        executable='head_motor_node',
        name='head_motor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
            }
        ],
        remappings=[
            # Remap topics if needed
            # ('~/joint_states', '/head/joint_states'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        config_file_arg,
        head_motor_node,
    ])
