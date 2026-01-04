#!/usr/bin/env python3
"""Launch file for displaying qyh_dual_arms_description robot in RViz2."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for qyh_dual_arms_description display."""
    # Get the package directory
    pkg_share = get_package_share_directory('qyh_dual_arms_description')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_arms.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # # Force j4 to zero node (corrects floating point errors from GUI)
    # force_j4_zero_node = Node(
    #     package='qyh_dual_arms_description',
    #     executable='force_j4_zero.py',
    #     name='force_j4_zero',
    #     output='screen'
    # )

    # Robot State Publisher node (uses corrected joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }],
        # remappings=[
        #     ('/joint_states', '/joint_states_corrected')
        # ]
    )

    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2
        ',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'urdf.rviz')] if os.path.exists(
            os.path.join(pkg_share, 'urdf.rviz')) else []
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # force_j4_zero_node,
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
