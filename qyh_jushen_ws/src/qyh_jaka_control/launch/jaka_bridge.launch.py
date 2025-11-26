#!/usr/bin/env python3
"""
Launch JAKA bridge node for teleoperation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.2.200',
        description='JAKA robot IP address'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('qyh_jaka_control'),
            'config',
            'jaka_bridge_params.yaml'
        ]),
        description='Path to bridge configuration file'
    )
    
    # JAKA bridge node
    jaka_bridge_node = Node(
        package='qyh_jaka_control',
        executable='jaka_bridge_node',
        name='jaka_bridge_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'robot_ip': LaunchConfiguration('robot_ip')}
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        config_file_arg,
        jaka_bridge_node,
    ])
