import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    teleoperation_pkg = get_package_share_directory('qyh_teleoperation_controller')
    moveit_config_pkg = get_package_share_directory('qyh_dual_arms_moveit_config')
    description_pkg = get_package_share_directory('qyh_dual_arms_description')
    
    # Configuration file
    config_file = os.path.join(
        teleoperation_pkg,
        'config',
        'teleoperation_params.yaml'
    )
    
    # URDF file path
    urdf_file = os.path.join(
        description_pkg,
        'urdf',
        'dual_arms.urdf.xacro'
    )
    
    # Teleoperation Node
    teleoperation_node = Node(
        package='qyh_teleoperation_controller',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen',
        parameters=[
            config_file,
            {
                'robot_description': urdf_file,
                'use_sim_time': False
            }
        ]
    )
    
    return LaunchDescription([
        teleoperation_node
    ])
