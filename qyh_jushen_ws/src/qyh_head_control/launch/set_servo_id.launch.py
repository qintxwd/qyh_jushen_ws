#!/usr/bin/env python3
"""
舵机ID设置启动文件
用于批量设置舵机ID
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    
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
    
    declare_old_id = DeclareLaunchArgument(
        'old_id',
        default_value='1',
        description='Current servo ID (factory default is 1)'
    )
    
    declare_new_id = DeclareLaunchArgument(
        'new_id',
        default_value='2',
        description='New servo ID to set (1 for Pan, 2 for Tilt)'
    )
    
    # 设置舵机ID的节点
    set_id_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'qyh_head_control', 'set_servo_id',
            '--port', LaunchConfiguration('serial_port'),
            '--baudrate', LaunchConfiguration('baudrate'),
            '--old-id', LaunchConfiguration('old_id'),
            '--new-id', LaunchConfiguration('new_id'),
            '--batch'
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baudrate,
        declare_old_id,
        declare_new_id,
        set_id_process,
    ])
