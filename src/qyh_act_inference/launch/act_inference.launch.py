"""
ACT 推理节点启动文件

使用方法:
    ros2 launch qyh_act_inference act_inference.launch.py
    ros2 launch qyh_act_inference act_inference.launch.py model_path:=/path/to/policy.pt
    ros2 launch qyh_act_inference act_inference.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('qyh_act_inference')
    default_config = os.path.join(pkg_share, 'config', 'act_inference.yaml')
    
    # 声明参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to the ACT model (policy.pt)'
    )
    
    declare_device = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Inference device (cuda/cpu)'
    )
    
    declare_action_scale = DeclareLaunchArgument(
        'action_scale',
        default_value='0.4',
        description='Action scale factor'
    )
    
    declare_use_right_arm = DeclareLaunchArgument(
        'use_right_arm',
        default_value='true',
        description='Use right arm'
    )
    
    declare_use_right_gripper = DeclareLaunchArgument(
        'use_right_gripper',
        default_value='true',
        description='Use right gripper'
    )
    
    # 推理节点
    act_inference_node = Node(
        package='qyh_act_inference',
        executable='act_inference_node.py',
        name='act_inference_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'action_scale': LaunchConfiguration('action_scale'),
                'use_right_arm': LaunchConfiguration('use_right_arm'),
                'use_right_gripper': LaunchConfiguration('use_right_gripper'),
            }
        ],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_model_path,
        declare_device,
        declare_action_scale,
        declare_use_right_arm,
        declare_use_right_gripper,
        act_inference_node,
    ])
