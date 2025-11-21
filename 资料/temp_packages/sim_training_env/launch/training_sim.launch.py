#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('sim_training_env')
    dual_arms_pkg = get_package_share_directory('dual_arms')
    
    world_file = os.path.join(sim_pkg, 'worlds', 'pick_place_training.world')
    urdf_file = os.path.join(dual_arms_pkg, 'urdf', 'Dual-Arms-gazebo.urdf')
    controllers_file = os.path.join(dual_arms_pkg, 'config', 'dual_arms_controllers.yaml')
    
    # Load URDF
    with open(urdf_file, 'rb') as f:
        robot_desc = f.read().decode('utf-8')
    if robot_desc.startswith('<?xml'):
        robot_desc = robot_desc.split('?>', 1)[1].strip()

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'dual_arms',
                   '-x', '0', '-y', '0', '-z', '0.5'],
        output='screen'
    )

    # START ros2_control_node immediately (BYPASSES gazebo_ros2_control slow init)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen'
    )

    # Spawner with SHORT delays (only waiting for robot to spawn, not for plugin)
    load_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', 
                          '--controller-manager', '/controller_manager',
                          '--controller-manager-timeout', '10'],
                output='screen'
            )
        ]
    )

    load_right_arm_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_arm_controller',
                          '--controller-manager', '/controller_manager',
                          '--controller-manager-timeout', '10'],
                output='screen'
            )
        ]
    )

    load_left_arm_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller',
                          '--controller-manager', '/controller_manager',
                          '--controller-manager-timeout', '10'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        ros2_control_node,
        spawn_entity,
        load_joint_state_broadcaster,
        load_right_arm_controller,
        load_left_arm_controller,
    ])
