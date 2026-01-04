"""
QYH Bag Recorder 启动文件

启动 rosbag 录制节点
话题通过服务请求动态指定，不在 launch 中写死
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value='',
        description='录制文件保存的基础路径，默认为 ~/qyh_jushen_ws/DATA'
    )
    
    # Bag Recorder 节点
    bag_recorder_node = Node(
        package='qyh_bag_recorder',
        executable='bag_recorder_node',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'base_path': LaunchConfiguration('base_path'),
            # 话题不再写死，通过 StartRecording 服务动态传入
        }],
        remappings=[
            # 可以在这里添加话题重映射
        ]
    )
    
    return LaunchDescription([
        base_path_arg,
        bag_recorder_node
    ])
