#!/usr/bin/env python3
"""
Enhanced VR Interface Launch - User-calibrated posture-based mapping
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('qyh_vr_calibration')
    
    # Declare launch arguments
    username_arg = DeclareLaunchArgument(
        'username',
        default_value='',
        description='Username for calibration (leave empty for no auto-load)'
    )
    
    mapping_mode_arg = DeclareLaunchArgument(
        'mapping_mode',
        default_value='hybrid',
        description='Mapping mode: direct, incremental, or hybrid'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'vr_interface_params.yaml'),
        description='Path to VR interface parameters file'
    )
    
    # Enhanced VR calibration node (service provider)
    vr_calibration_node = Node(
        package='qyh_vr_calibration',
        executable='vr_calibration_node',
        name='vr_calibration_node',
        output='screen',
        emulate_tty=True,
    )
    
    # Enhanced VR interface node (with hybrid mapping)
    vr_interface_node = Node(
        package='qyh_vr_calibration',
        executable='vr_interface_node',
        name='vr_interface_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'default_user': LaunchConfiguration('username'),
                'mapping_mode': LaunchConfiguration('mapping_mode'),
            }
        ],
    )
    
    # VR simulator (for testing)
    vr_simulator_node = Node(
        package='qyh_vr_calibration',
        executable='vr_simulator_node',
        name='vr_simulator_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        username_arg,
        mapping_mode_arg,
        config_file_arg,
        vr_calibration_node,
        vr_interface_node,
        vr_simulator_node,
    ])
