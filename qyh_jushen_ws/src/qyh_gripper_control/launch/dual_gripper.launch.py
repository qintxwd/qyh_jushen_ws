from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch both left and right gripper control nodes"""
    
    gripper_launch = PathJoinSubstitution([
        FindPackageShare('qyh_gripper_control'),
        'launch',
        'gripper_control.launch.py'
    ])
    
    # Left gripper
    left_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch),
        launch_arguments={'side': 'left'}.items()
    )
    
    # Right gripper
    right_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_launch),
        launch_arguments={'side': 'right'}.items()
    )
    
    return LaunchDescription([
        left_gripper,
        right_gripper
    ])
