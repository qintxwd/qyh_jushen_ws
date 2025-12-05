#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node  

import os

#qyh_nav_ws的仿真启动
def generate_launch_description():
    # 声明launch参数
    # 原本此部分的tf、imu、mcu、lidar替换为gazebo发布
    # 启动仿真
    use_sim_time = 'true'
    simu_ros2_pkg_share = FindPackageShare('simu_ros2')
    
    # simple_world.launch.py文件路径
    simple_world_launch_path = PathJoinSubstitution([
        simu_ros2_pkg_share,
        'launch',
        'simple_world.launch.py'
    ])
    
    # 包含simple_world launch文件
    simple_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simple_world_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    # 启动robot_pose_publisher节点
    robot_pose_publisher_cmd = Node(
        package='robot_pose_publisher',
        executable='robot_pose_publisher',
        name='robot_pose_publisher',
        output='screen'
    )

    #region 启动key_operator节点，暂时先不启动
    key_operator_pkg_share = FindPackageShare('key_operator')

    key_operator_launch_path = PathJoinSubstitution([
        key_operator_pkg_share,
        'launch',
        'key_operator.launch.py'
    ])

    key_operator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(key_operator_launch_path)
    )
    #endregion
    
    #region 启动地图管理器
    map_manager_pkg_share = FindPackageShare('map_manager')

    map_manager_launch_path = PathJoinSubstitution([
        map_manager_pkg_share,
        'launch',
        'map_manager.launch.py'
    ])

    map_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_manager_launch_path)
    )
    # 延迟1秒启动地图管理器
    delayed_map_manager_launch = TimerAction(
        period=5.0,  # 延迟1秒
        actions=[map_manager_launch]
    )
    #endregion

    #region 启动3d地图加载器
    map3d_server_pkg_share = FindPackageShare('map3d_server')

    map3d_server_launch_path = PathJoinSubstitution([
        map3d_server_pkg_share,
        'launch',
        'map3d_server.launch.py'
    ])

    map3d_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map3d_server_launch_path)
    )
    #endregion

    #region 启动lidar_localization节点
    lidar_localization_pkg_share = FindPackageShare('lidar_localization_ros2')

    lidar_localization_launch_path = PathJoinSubstitution([
        lidar_localization_pkg_share,
        'launch',
        'lidar_localization.launch.py'
    ])

    lidar_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_localization_launch_path)
    )
    # 延迟5秒启动lidar_localization节点
    delayed_lidar_localization_launch = TimerAction(
        period=2.0,  # 延迟 3秒
        actions=[lidar_localization_launch]
    )
    #endregion

    #region 启动静态 TF 发布节点（固定变换 map -> odom）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1.0', '-1.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )
    #endregion
    
    #region 启动convert2png
    convert2png_cmd = Node(
        package='convert2png',
        executable='convert2png_node',
        name='convert2png_node',
        output='screen'
    )
    #endregion

    #region 启动nav2
    robot_name = os.environ.get('GLOBAL_ROBOT_NAME', 'general')
    robot_version = os.environ.get('GLOBAL_ROBOT_VERSION', '1.0')
    
    param_name = 'params.yaml'
    bringup_pkg_share = FindPackageShare('bringup')
    
    nav2_param__path = PathJoinSubstitution([
        bringup_pkg_share,
        'launch',
        f"{robot_name}_{robot_version}",
        'param',
        param_name
    ])
    nav2_pkg_share = FindPackageShare('nav2_bringup')
    nav2_launch_path = PathJoinSubstitution([
        nav2_pkg_share,
        'launch',
    ])

    map_dir = ''
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param__path}.items(),
    )
    
    #endregion

    #message_center和task_manager暂时不启动


    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    # ld.add_action(sonar_amount_arg)
    
    # 添加静态TF发布节点（使用OpaqueFunction以便访问context）
    ld.add_action(simple_world_launch)

    ld.add_action(map3d_server_launch)
    ld.add_action(delayed_lidar_localization_launch)
    # ld.add_action(static_tf_node)

    # ld.add_action(key_operator_launch)
    # ld.add_action(convert2png_cmd)
    # ld.add_action(nav2_launch)
    #延时1秒，确保nav2启动完成

    # ld.add_action(delayed_map_manager_launch)
    # ld.add_action(robot_pose_publisher_cmd)
    
    return ld