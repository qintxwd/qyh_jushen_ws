from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('qyh_gripper_control')
    config_file = os.path.join(pkg_share, 'config', 'gripper_right.yaml')
    
    gripper_node = Node(
        package='qyh_gripper_control',
        executable='gripper_control_node',
        name='right_gripper_control',
        namespace='right',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        gripper_node
    ])
