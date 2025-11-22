from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch both left and right gripper control nodes"""
    
    # Left gripper
    left_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_gripper_control'),
                'launch',
                'gripper_control_left.launch.py'
            ])
        ])
    )
    
    # Right gripper
    right_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('qyh_gripper_control'),
                'launch',
                'gripper_control_right.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        left_gripper,
        right_gripper
    ])
