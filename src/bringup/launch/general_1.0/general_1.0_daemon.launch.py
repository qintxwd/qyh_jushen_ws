#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明launch参数
    declare_enable_jaka = DeclareLaunchArgument(
        'enable_jaka', default_value='true',
        description='启用JAKA机械臂控制'
    )
    
    declare_enable_chassis = DeclareLaunchArgument(
        'enable_chassis', default_value='true',
        description='启用底盘控制'
    )
    
    declare_enable_task_engine = DeclareLaunchArgument(
        'enable_task_engine', default_value='true',
        description='启用任务引擎'
    )
    
    declare_enable_lift = DeclareLaunchArgument(
        'enable_lift', default_value='true',
        description='启用升降台控制'
    )
    
    declare_enable_gripper = DeclareLaunchArgument(
        'enable_gripper', default_value='true',
        description='启用夹爪控制'
    )
    
    declare_enable_waist = DeclareLaunchArgument(
        'enable_waist', default_value='true',
        description='启用腰部控制'
    )
    
    declare_enable_head = DeclareLaunchArgument(
        'enable_head', default_value='true',
        description='启用头部控制'
    )
    
    declare_enable_shutdown = DeclareLaunchArgument(
        'enable_shutdown', default_value='true',
        description='启用关机控制'
    )
    
    
    declare_enable_vr = DeclareLaunchArgument(
        'enable_vr', default_value='true',
        description='启用VR遥操作'
    )
    
    declare_enable_bag_recorder = DeclareLaunchArgument(
        'enable_bag_recorder', default_value='true',
        description='启用Bag录制'
    )
    
    declare_enable_camera = DeclareLaunchArgument(
        'enable_camera', default_value='true',
        description='启用摄像头'
    )
    
    declare_enable_web_video = DeclareLaunchArgument(
        'enable_web_video', default_value='true',
        description='启用Web视频服务'
    )
    
    # 获取环境变量
    qyh_data_path = EnvironmentVariable('QYH_DATA_PATH', default_value=os.path.expanduser('~/qyh-robot-system/model_actions'))
    
    # 1. JAKA臂控制节点
    jaka_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_jaka_control'),
                'launch',
                'jaka_control.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_jaka'))
    )
    
    # 2. 底盘控制节点
    chassis_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_standard_robot'),
                'launch',
                'standard_robot.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_chassis'))
    )
    
    # 3. 任务引擎节点
    task_engine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_task_engine'),
                'launch',
                'task_engine.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_task_engine'))
    )
    
    # 4. 升降台控制节点
    lift_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_lift_control'),
                'launch',
                'lift_control.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_lift'))
    )
    
    # 5. 夹爪控制节点
    gripper_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_gripper_control'),
                'launch',
                'gripper.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_gripper'))
    )
    
    # 6. 腰部控制节点
    waist_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_waist_control'),
                'launch',
                'waist_control.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_waist'))
    )
    
    # 7. 头部控制节点
    head_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_head_motor_control'),
                'launch',
                'head_motor.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_head'))
    )
    
    # 8. 关机控制节点
    shutdown_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_shutdown'),
                'launch',
                'qyh_shutdown.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_shutdown'))
    )
    
    # 9. VR Bridge节点
    vr_bridge_node = Node(
        package='qyh_vr_bridge',
        executable='vr_bridge_node',
        name='vr_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_vr'))
    )
    
    # 10. VR Teleop启动
    vr_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_dual_arm_teleop_python'),
                'launch',
                'qyh_dual_teleop_launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_vr'))
    )
    
    # 11. VR Button Event启动
    vr_button_event_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_vr_button_event'),
                'launch',
                'qyh_vr_button_event.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_vr'))
    )
    
    # 12. Bag录制节点
    bag_recorder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_bag_recorder'),
                'launch',
                'bag_recorder.launch.py'
            ])
        ]),
        launch_arguments={
            'base_path': qyh_data_path
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_bag_recorder'))
    )
    
    # 13. 摄像头节点
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'qyh.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    # 14. Web视频服务节点
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'server_threads': 8,
            'ros_threads': 8
        }],
        condition=IfCondition(LaunchConfiguration('enable_web_video'))
    )
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加所有声明的参数
    ld.add_action(declare_enable_jaka)
    ld.add_action(declare_enable_chassis)
    ld.add_action(declare_enable_task_engine)
    ld.add_action(declare_enable_lift)
    ld.add_action(declare_enable_gripper)
    ld.add_action(declare_enable_waist)
    ld.add_action(declare_enable_head)
    ld.add_action(declare_enable_shutdown)
    ld.add_action(declare_enable_vr)
    ld.add_action(declare_enable_bag_recorder)
    ld.add_action(declare_enable_camera)
    ld.add_action(declare_enable_web_video)
    
    # 添加所有节点和launch文件
    # 硬件控制节点
    ld.add_action(jaka_control_launch)        # JAKA机械臂
    ld.add_action(chassis_control_launch)     # 底盘
    ld.add_action(lift_control_launch)        # 升降台
    ld.add_action(gripper_control_launch)     # 夹爪
    ld.add_action(waist_control_launch)       # 腰部
    ld.add_action(head_control_launch)        # 头部
    ld.add_action(shutdown_control_launch)    # 关机控制
    
    # 任务引擎
    ld.add_action(task_engine_launch)
    
    # VR遥操作
    ld.add_action(vr_bridge_node)
    ld.add_action(vr_teleop_launch)
    ld.add_action(vr_button_event_launch)
    
    # 数据录制
    ld.add_action(bag_recorder_launch)
    
    # 视觉系统
    ld.add_action(camera_launch)
    ld.add_action(web_video_server_node)
    
    return ld
