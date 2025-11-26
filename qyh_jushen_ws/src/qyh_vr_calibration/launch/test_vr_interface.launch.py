#!/usr/bin/env python3
"""
Test launch file using VR simulator instead of real VR hardware
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    motion_type_arg = DeclareLaunchArgument(
        'motion_type',
        default_value='circle',
        description='Motion type: circle, figure8, vertical, static'
    )
    
    amplitude_arg = DeclareLaunchArgument(
        'amplitude',
        default_value='0.05',
        description='Motion amplitude in meters'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='0.1',
        description='Motion frequency in Hz'
    )
    
    # VR simulator node
    vr_simulator = Node(
        package='qyh_vr_calibration',
        executable='vr_simulator_node',
        name='vr_simulator_node',
        output='screen',
        parameters=[{
            'motion_type': LaunchConfiguration('motion_type'),
            'motion_amplitude': LaunchConfiguration('amplitude'),
            'motion_frequency': LaunchConfiguration('frequency'),
            'publish_rate': 90.0,
        }]
    )
    
    # VR interface node
    vr_interface = Node(
        package='qyh_vr_calibration',
        executable='vr_interface_node',
        name='vr_interface_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('qyh_vr_calibration'),
                'config',
                'vr_interface_params.yaml'
            ])
        ]
    )
    
    return LaunchDescription([
        motion_type_arg,
        amplitude_arg,
        frequency_arg,
        vr_simulator,
        vr_interface,
    ])
