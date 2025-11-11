from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    启动JAKA双臂机器人控制节点
    
    参数:
        robot_ip: 机器人控制器IP地址 (默认: 192.168.2.200)
        config_file: 配置文件路径 (默认: 使用package中的robot_config.yaml)
        auto_connect: 是否自动连接 (默认: true)
        auto_power_on: 是否自动上电 (默认: false)
        auto_enable: 是否自动使能 (默认: false)
    """
    
    # 获取包路径
    pkg_share = get_package_share_directory('jaka_control')
    default_config = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    # 声明启动参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.2.200',
        description='Robot controller IP address'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to robot configuration file'
    )
    
    auto_connect_arg = DeclareLaunchArgument(
        'auto_connect',
        default_value='true',
        description='Auto connect to robot on startup'
    )
    
    auto_power_on_arg = DeclareLaunchArgument(
        'auto_power_on',
        default_value='false',
        description='Auto power on robot on startup'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Auto enable robot on startup'
    )
    
    # 创建机器人控制节点
    jaka_robot_node = Node(
        package='jaka_control',
        executable='jaka_robot_node',
        name='jaka_robot_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'auto_connect': LaunchConfiguration('auto_connect'),
                'auto_power_on': LaunchConfiguration('auto_power_on'),
                'auto_enable': LaunchConfiguration('auto_enable'),
            }
        ],
        remappings=[
            ('/joint_states', '/jaka_robot/joint_states'),
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        config_file_arg,
        auto_connect_arg,
        auto_power_on_arg,
        auto_enable_arg,
        jaka_robot_node,
    ])
