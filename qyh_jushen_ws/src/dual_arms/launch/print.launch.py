#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file to print joint states and tool poses at 5Hz with j4 forced to zero
    """
    # Get the package directory
    pkg_share = get_package_share_directory('dual_arms')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_arms.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Force j4 to zero node (corrects floating point errors)
    force_j4_zero_node = Node(
        package='dual_arms',
        executable='force_j4_zero.py',
        name='force_j4_zero',
        output='screen'
    )

    # Robot State Publisher node (uses corrected joint states for TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }],
        remappings=[
            ('/joint_states', '/joint_states_corrected')
        ]
    )

    # Print robot info node (uses corrected joint states)
    print_info_node = Node(
        package='dual_arms',
        executable='print_robot_info.py',
        name='print_robot_info',
        output='screen',
        parameters=[{
            'frequency': 5.0,  # 5Hz
            'left_arm_joints': [
                'l-j1', 'l-j2', 'l-j3',
                'l-j4', 'l-j5', 'l-j6'
            ],
            'right_arm_joints': [
                'r-j1', 'r-j2', 'r-j3',
                'r-j4', 'r-j5', 'r-j6'
            ],
            'left_tool_frame': 'lt',
            'right_tool_frame': 'rt',
            'base_frame': 'base_link'
        }],
        remappings=[
            ('/joint_states', '/joint_states_corrected')
        ]
    )

    return LaunchDescription([
        force_j4_zero_node,
        robot_state_publisher_node,
        print_info_node
    ])
