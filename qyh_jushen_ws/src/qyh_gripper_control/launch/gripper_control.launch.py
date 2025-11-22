from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    side_arg = DeclareLaunchArgument(
        'side',
        default_value='left',
        description='Which gripper: left or right'
    )
    
    # Config file path - 正确拼接文件名
    config_file = PathJoinSubstitution([
        FindPackageShare('qyh_gripper_control'),
        'config',
        PythonExpression(["'gripper_", LaunchConfiguration('side'), ".yaml'"])
    ])
    
    # Gripper control node
    gripper_node = Node(
        package='qyh_gripper_control',
        executable='gripper_control_node',
        name=PythonExpression([
            "'", LaunchConfiguration('side'), "_gripper_control'"
        ]),
        namespace=LaunchConfiguration('side'),
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        side_arg,
        gripper_node
    ])
