#!/usr/bin/env python3
"""
Launch file for VR interface node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('qyh_vr_calibration'),
            'config',
            'vr_interface_params.yaml'
        ]),
        description='Path to VR interface configuration file'
    )
    
    # VR interface node
    vr_interface_node = Node(
        package='qyh_vr_calibration',
        executable='vr_interface_node',
        name='vr_interface_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Remap if needed based on actual VR hardware topics
            # ('/vr/left_hand/pose', '/pico4/left_controller/pose'),
            # ('/vr/right_hand/pose', '/pico4/right_controller/pose'),
            # ('/vr/buttons', '/pico4/buttons'),
        ]
    )
    
    return LaunchDescription([
        config_arg,
        vr_interface_node,
    ])
