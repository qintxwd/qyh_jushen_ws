#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='IP address of the JAKA robot'
        ),
        DeclareLaunchArgument(
            'cycle_time_ms',
            default_value='8.0',
            description='Servo cycle time in milliseconds (125Hz = 8ms)'
        ),
        DeclareLaunchArgument(
            'auto_connect',
            default_value='true',
            description='Automatically connect to robot on startup'
        ),
        DeclareLaunchArgument(
            'auto_power_on',
            default_value='false',
            description='Automatically power on robot after connection'
        ),
        DeclareLaunchArgument(
            'auto_enable',
            default_value='false',
            description='Automatically enable robot after power on'
        ),

        # JAKA完整控制节点（基础+伺服+VR）
        Node(
            package='qyh_jaka_control',
            executable='jaka_control_node',
            name='jaka_control_node',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'cycle_time_ms': LaunchConfiguration('cycle_time_ms'),
                'auto_connect': LaunchConfiguration('auto_connect'),
                'auto_power_on': LaunchConfiguration('auto_power_on'),
                'auto_enable': LaunchConfiguration('auto_enable'),
                'use_cartesian': False,
                'recording_output_dir': '/tmp/jaka_recordings',
            }],
        ),
    ])
