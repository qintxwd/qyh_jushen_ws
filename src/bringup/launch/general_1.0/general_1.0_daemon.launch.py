#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 声明launch参数
    
    # 获取环境变量配置
    def get_static_tf_nodes(context):
        nodes = []
        
        # ==================== 静态TF变换: base_footprint -> base_link ====================
        base_footprint_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_broadcaster',
            arguments=[
                '0', '0', '0.065',  # x, y, z
                '0', '0', '0',       # roll, pitch, yaw
                'base_footprint',
                'base_link'
            ],
            output='screen'
        )
        nodes.append(base_footprint_tf)
        
        # ==================== 静态TF变换: base_link -> front_laser ====================
        # 从环境变量读取前激光雷达位姿
        front_laser_x = os.environ.get('GLOBAL_LASER_1_POSE_X', '0')
        front_laser_y = os.environ.get('GLOBAL_LASER_1_POSE_Y', '0')
        front_laser_z = os.environ.get('GLOBAL_LASER_1_POSE_Z', '0')
        front_laser_roll = os.environ.get('GLOBAL_LASER_1_POSE_ROLL', '0')
        front_laser_pitch = os.environ.get('GLOBAL_LASER_1_POSE_PITCH', '0')
        front_laser_yaw = os.environ.get('GLOBAL_LASER_1_POSE_YAW', '0')
        
        front_laser_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_laser_broadcaster',
            arguments=[
                front_laser_x, front_laser_y, front_laser_z,
                front_laser_roll, front_laser_pitch, front_laser_yaw,
                'base_link',
                'front_laser'
            ],
            output='screen'
        )
        nodes.append(front_laser_tf)
        
        # ==================== 静态TF变换: base_link -> base_imu ====================
        # 从环境变量读取IMU位姿
        imu_x = os.environ.get('GLOBAL_IMU_X', '0')
        imu_y = os.environ.get('GLOBAL_IMU_Y', '0')
        imu_z = os.environ.get('GLOBAL_IMU_Z', '0')
        imu_roll = os.environ.get('GLOBAL_IMU_ROLL', '0')
        imu_pitch = os.environ.get('GLOBAL_IMU_PITCH', '0')
        imu_yaw = os.environ.get('GLOBAL_IMU_YAW', '0')
        
        imu_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=[
                imu_x, imu_y, imu_z,
                imu_roll, imu_pitch, imu_yaw,
                'base_link',
                'base_imu'
            ],
            output='screen'
        )
        nodes.append(imu_tf)
        
        return nodes
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加参数声明
    # ld.add_action(sonar_amount_arg)
    
    # 添加静态TF发布节点（使用OpaqueFunction以便访问context）
    ld.add_action(OpaqueFunction(function=get_static_tf_nodes))
    
    return ld