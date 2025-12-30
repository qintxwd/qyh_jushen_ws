from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    left_node = Node(
        package='qyh_dual_arm_teleop_python',
        executable='qyh_teleop_left',
        name='qyh_teleop_left',
        output='screen'
    )

    right_node = Node(
        package='qyh_dual_arm_teleop_python',
        executable='qyh_teleop_right',
        name='qyh_teleop_right',
        output='screen'
    )

    ld.add_action(left_node)
    ld.add_action(right_node)

    return ld
