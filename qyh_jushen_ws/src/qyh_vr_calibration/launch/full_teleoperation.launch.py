#!/usr/bin/env python3
"""
Complete VR teleoperation system launch file
Launches: VR interface → Virtual arms (MoveIt) → Teleoperation controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # 1. VR interface node
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
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 2. MoveIt move_group (virtual arms)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_dual_arms_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # 3. Teleoperation controller
    teleoperation_controller = Node(
        package='qyh_teleoperation_controller',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('qyh_teleoperation_controller'),
                'config',
                'teleoperation_params.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        vr_interface,
        moveit_launch,
        teleoperation_controller,
    ])
