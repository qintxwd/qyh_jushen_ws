#!/usr/bin/env python3
"""
双臂IK求解器Launch文件

使用方式：
  终端1: ros2 launch qyh_jaka_control jaka_control.launch.py
  终端2: ros2 launch qyh_dual_arm_teleop teleop.launch.py
  终端3: ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('qyh_dual_arm_ik_solver')
    config_file = os.path.join(pkg_share, 'config', 'ik_solver_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='JAKA controller IP (same as qyh_jaka_control)'
        ),
        DeclareLaunchArgument(
            'ik_rate',
            default_value='125.0',
            description='IK solving frequency in Hz'
        ),
        
        Node(
            package='qyh_dual_arm_ik_solver',
            executable='dual_arm_ik_solver_node',
            name='dual_arm_ik_solver',
            output='screen',
            parameters=[
                config_file,
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'ik_rate': LaunchConfiguration('ik_rate'),
                }
            ]
        ),
    ])
