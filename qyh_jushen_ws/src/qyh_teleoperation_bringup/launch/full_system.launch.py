#!/usr/bin/env python3
"""
Complete VR teleoperation system launch file
Launches the full pipeline: VR → Virtual Arms → Teleoperation Controller → JAKA Bridge → Real Robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.2.200',
        description='JAKA robot IP address'
    )
    
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='false',
        description='Use VR simulator instead of real VR'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # 1. VR simulator (optional)
    vr_simulator = Node(
        package='qyh_vr_calibration',
        executable='vr_simulator_node',
        name='vr_simulator_node',
        output='screen',
        parameters=[{
            'motion_type': 'circle',
            'motion_amplitude': 0.05,
            'motion_frequency': 0.1,
            'publish_rate': 90.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_simulator'))
    )
    
    # 2. VR interface node
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
    
    # 3. MoveIt move_group (virtual arms for visualization)
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
    
    # 4. Teleoperation controller (differential IK + smoothing + safety)
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
    
    # 5. JAKA bridge node (smooth servo to real robot)
    jaka_bridge = Node(
        package='qyh_jaka_control',
        executable='jaka_bridge_node',
        name='jaka_bridge_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('qyh_jaka_control'),
                'config',
                'jaka_bridge_params.yaml'
            ]),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_ip_arg,
        use_simulator_arg,
        rviz_arg,
        vr_simulator,
        vr_interface,
        moveit_launch,
        teleoperation_controller,
        jaka_bridge,
    ])
