#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    
    # 获取环境变量
    robot_name = os.environ.get('GLOBAL_ROBOT_NAME', 'general')
    robot_version = os.environ.get('GLOBAL_ROBOT_VERSION', '1.0')
    
    # 构建launch文件名
    daemon_launch_name = f"{robot_name}_{robot_version}_daemon.launch.py"
    # nav_launch_name = f"{robot_name}_{robot_version}_nav.launch.py"
    
    # 查找bringup包的路径
    bringup_pkg_share = FindPackageShare('bringup')
    
    # daemon launch文件路径
    daemon_launch_path = PathJoinSubstitution([
        bringup_pkg_share,
        'launch',
        f"{robot_name}_{robot_version}",
        daemon_launch_name
    ])
        
    # 包含daemon launch文件
    daemon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(daemon_launch_path),
        launch_arguments={}.items()
    )
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加子launch文件
    ld.add_action(daemon_launch)
    
    return ld