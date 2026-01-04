from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('qyh_dual_arm_teleop_python')
    left_yaml = os.path.join(pkg_share, 'config', 'left.yaml')
    right_yaml = os.path.join(pkg_share, 'config', 'right.yaml')

    # Launch two instances of the same executable with different params
    left_node = Node(
        package='qyh_dual_arm_teleop_python',
        executable='qyh_teleop',
        name='qyh_teleop_left',
        parameters=[left_yaml],
        output='screen'
    )

    right_node = Node(
        package='qyh_dual_arm_teleop_python',
        executable='qyh_teleop',
        name='qyh_teleop_right',
        parameters=[right_yaml],
        output='screen'
    )

    ld.add_action(left_node)
    ld.add_action(right_node)

    return ld
