import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('qyh_dual_arms_description')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_arms.urdf.xacro')
    
    # RViz配置文件路径
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 使用xacro处理URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher GUI节点
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2节点
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
