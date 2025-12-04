#!/usr/bin/env python3
"""
VR Clutch 仿真测试 Launch文件

启动内容:
1. MoveIt + RViz 机械臂可视化
2. VR模拟器节点 (键盘控制)
3. 仿真Clutch节点
4. 仿真机械臂控制器

使用方法:
ros2 launch qyh_vr_calibration sim_test.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 包路径
    pkg_vr_calibration = get_package_share_directory('qyh_vr_calibration')
    pkg_moveit_config = get_package_share_directory('qyh_dual_arms_moveit_config')
    
    # 参数文件
    sim_params_file = os.path.join(pkg_vr_calibration, 'config', 'sim_test_params.yaml')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. 包含MoveIt demo launch (启动MoveGroup + RViz)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'demo.launch.py')
        )
    )
    
    # 2. VR模拟器节点
    vr_simulator_node = Node(
        package='qyh_vr_calibration',
        executable='vr_simulator_node',
        name='vr_simulator_node',
        output='screen',
        parameters=[sim_params_file],
    )
    
    # 3. 仿真Clutch节点
    sim_clutch_node = Node(
        package='qyh_vr_calibration',
        executable='sim_clutch_node',
        name='sim_clutch_node',
        output='screen',
        parameters=[sim_params_file],
    )
    
    # 4. 仿真机械臂控制器 (可选，如果想让机械臂实际动)
    # 注：这个节点需要MoveIt正常工作后才能使用
    # sim_arm_controller = Node(
    #     package='qyh_vr_calibration',
    #     executable='sim_arm_controller',
    #     name='sim_arm_controller',
    #     output='screen',
    #     parameters=[sim_params_file],
    # )
    
    return LaunchDescription([
        # MoveIt demo (包含RViz)
        moveit_demo,
        
        # VR模拟器
        vr_simulator_node,
        
        # Clutch控制器
        sim_clutch_node,
        
        # 机械臂控制器 (暂时注释，先测试Clutch逻辑)
        # sim_arm_controller,
    ])
