#!/usr/bin/env python3
"""
简化版 Robot State Publisher 启动文件
只启动 robot_state_publisher，不启动 RViz 和 GUI
用于真机 VR 遥操作场景
"""
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('qyh_dual_arms_description')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_arms.urdf.xacro')
    
    # 使用xacro处理URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher
    ])
