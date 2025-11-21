#!/usr/bin/env python3
"""
演示录制Launch文件
同时启动：仿真、键盘控制、数据录制
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('sim_training_env')
    
    # 参数
    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value=os.path.expanduser('~/demo_recordings'),
        description='Directory to save recorded demonstrations'
    )
    
    task_name_arg = DeclareLaunchArgument(
        'task_name',
        default_value='pick_place_upright',
        description='Name of the task being demonstrated'
    )
    
    # 包含仿真环境
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(sim_pkg, 'launch', 'training_sim.launch.py')
        ])
    )
    
    # 数据录制节点
    data_recorder = Node(
        package='sim_training_env',
        executable='data_recorder_enhanced.py',
        name='data_recorder',
        output='screen',
        parameters=[{
            'save_dir': LaunchConfiguration('save_dir'),
            'task_name': LaunchConfiguration('task_name')
        }]
    )
    
    return LaunchDescription([
        save_dir_arg,
        task_name_arg,
        simulation,
        data_recorder,
    ])
