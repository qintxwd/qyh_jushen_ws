from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('qyh_vr_button_event')
    config = os.path.join(pkg_share, 'config', 'config.yaml')

    node = Node(
        package='qyh_vr_button_event',
        executable='qyh_vr_button_event',
        name='qyh_vr_button_event',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([node])
