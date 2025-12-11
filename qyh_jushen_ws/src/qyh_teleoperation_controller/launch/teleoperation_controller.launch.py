import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    teleoperation_pkg = get_package_share_directory('qyh_teleoperation_controller')
    description_pkg = get_package_share_directory('qyh_dual_arms_description')
    moveit_config_pkg = get_package_share_directory('qyh_dual_arms_moveit_config')
    
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
    
    # SRDF file path
    srdf_file = os.path.join(
        moveit_config_pkg,
        'config',
        'qyh_dual_arms.srdf'
    )
    
    # 使用 xacro 处理 URDF 文件，获取实际的 XML 内容
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # 读取 SRDF 文件内容
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    
    # Teleoperation Node
    teleoperation_node = Node(
        package='qyh_teleoperation_controller',
        executable='teleoperation_node',
        name='teleoperation_node',
        output='screen',
        parameters=[
            config_file,
            robot_description,
            robot_description_semantic,
            {
                'use_sim_time': False
            }
        ]
    )
    
    return LaunchDescription([
        teleoperation_node
    ])
