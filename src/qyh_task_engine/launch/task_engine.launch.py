"""
任务引擎启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('qyh_task_engine')
    config_file = os.path.join(pkg_dir, 'config', 'task_engine.yaml')
    
    # 参数
    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate',
        default_value='10.0',
        description='Tick rate in Hz'
    )
    
    status_rate_arg = DeclareLaunchArgument(
        'status_publish_rate',
        default_value='5.0',
        description='Status publish rate in Hz'
    )
    
    # 任务引擎节点
    task_engine_node = Node(
        package='qyh_task_engine',
        executable='task_engine_node.py',
        name='task_engine_node',
        output='screen',
        parameters=[
            config_file,
            {
                'tick_rate': LaunchConfiguration('tick_rate'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
            }
        ]
    )
    
    return LaunchDescription([
        tick_rate_arg,
        status_rate_arg,
        task_engine_node,
    ])
