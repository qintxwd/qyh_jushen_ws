"""
VR Clutch模式启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('qyh_vr_calibration')
    config_file = os.path.join(pkg_dir, 'config', 'vr_clutch_params.yaml')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the clutch configuration file'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',  # 默认为仿真模式
            description='True for simulation, False for real robot'
        ),
        
        # VR Clutch Node
        Node(
            package='qyh_vr_calibration',
            executable='vr_clutch_node',
            name='vr_clutch_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'simulation_mode': LaunchConfiguration('simulation_mode')}
            ],
            remappings=[
                # 可以在这里添加话题重映射
            ]
        ),
    ])
