#!/usr/bin/env python3
"""
双臂VR遥操作完整启动文件

启动节点：
  1. static_transform_publisher: base_link → teleop_base (静态TF)
  2. teleop_manager_node: 状态管理和零位校准
  3. coordinate_mapper_node: 坐标变换和滤波

依赖节点（需要单独启动）：
  - vr_bridge_node: VR数据接收 (来自qyh_vr_bridge包)
  - dual_arm_ik_solver_node: IK求解 (未实现)
  - arm_controller_node: 机械臂控制 (未实现)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('qyh_dual_arm_teleop')
    
    # 参数文件路径
    config_file = PathJoinSubstitution([
        pkg_share, 'config', 'teleop_params.yaml'
    ])
    
    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # === 节点1: 静态TF - base_link → teleop_base ===
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='teleop_base_publisher',
        arguments=[
            '0.3', '0.0', '0.0',     # xyz: 人在机器人前方0.5米
            '0.0', '0.0', '0.0',     # rpy: 无旋转
            'base_link',             # parent_frame
            'teleop_base'            # child_frame
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # === 节点2: Teleop Manager ===
    teleop_manager_node = Node(
        package='qyh_dual_arm_teleop',
        executable='teleop_manager_node',
        name='teleop_manager_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            # 订阅VR手柄joy数据
            ('/vr/left_controller/joy', '/vr/left_controller/joy'),
            ('/vr/right_controller/joy', '/vr/right_controller/joy'),
        ]
    )
    
    # === 节点3: Coordinate Mapper ===
    coordinate_mapper_node = Node(
        package='qyh_dual_arm_teleop',
        executable='coordinate_mapper_node',
        name='coordinate_mapper_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            # 发布目标位姿
            ('/teleop/left_hand/target', '/teleop/left_hand/target'),
            ('/teleop/right_hand/target', '/teleop/right_hand/target'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        static_tf_node,
        teleop_manager_node,
        coordinate_mapper_node,
    ])
