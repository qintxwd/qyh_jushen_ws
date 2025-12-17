#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('qyh_jaka_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    # 获取URDF文件（用于robot_state_publisher）
    urdf_pkg_share = get_package_share_directory('qyh_dual_arms_description')
    urdf_file = os.path.join(urdf_pkg_share, 'urdf', 'dual_arms.urdf')
    
    # 读取URDF内容
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='JAKA robot IP address'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to robot configuration YAML file'
        ),
        DeclareLaunchArgument(
            'auto_initialize',
            default_value='false',
            description='Auto: power on, enable, init pose, start VR'
        ),
        DeclareLaunchArgument(
            'cycle_time_ms',
            default_value='8.0',
            description='Control cycle time in ms (8.0ms = 125Hz)'
        ),
        DeclareLaunchArgument(
            'buffer_size',
            default_value='10',
            description='Smooth servo bridge buffer size'
        ),
        DeclareLaunchArgument(
            'interpolation_weight',
            default_value='0.5',
            description='Smooth servo bridge interpolation weight'
        ),

        # JAKA完整控制节点（基础+伺服+VR）
        # remapping： /joint_states  ==> /joint_states_raw
        Node(
            package='qyh_jaka_control',
            executable='jaka_control_node',
            name='jaka_control_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'auto_initialize': LaunchConfiguration('auto_initialize'),
                    'cycle_time_ms': LaunchConfiguration('cycle_time_ms'),
                    'buffer_size': LaunchConfiguration('buffer_size'),
                    'interpolation_weight': LaunchConfiguration('interpolation_weight')
                }
            ],
            remappings=[
                ('/joint_states', '/joint_states_raw')
            ]
        ),
        
        # 关节名称适配器节点
        # 功能: 将 JAKA 驱动的关节名称格式转换为 URDF 格式
        # 输入: /joint_states_raw (left_jointN / right_jointN 或其他格式)
        # 输出: /joint_states (l-jN / r-jN, 匹配URDF)
        Node(
            package='qyh_jaka_control',
            executable='qyh_jaka_joint_adapter_node.py',
            name='qyh_jaka_joint_adapter',
            output='screen'
        ),
        
        # Robot State Publisher节点
        # 功能: 根据 joint_states 和 URDF 发布 TF 变换
        # 订阅: /joint_states (来自适配器，关节名称已匹配URDF)
        # 发布: /tf, /tf_static (完整的机械臂 TF 树)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        )
    ])
