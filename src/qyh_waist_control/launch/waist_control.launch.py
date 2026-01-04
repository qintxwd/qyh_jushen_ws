"""
腰部电机控制节点启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('qyh_waist_control')
    config_file = os.path.join(pkg_dir, 'config', 'waist_params.yaml')

    # 声明启动参数
    plc_ip_arg = DeclareLaunchArgument(
        'plc_ip',
        default_value='192.168.1.88',
        description='PLC IP address'
    )

    plc_port_arg = DeclareLaunchArgument(
        'plc_port',
        default_value='502',
        description='Modbus TCP port'
    )

    # 腰部电机控制节点
    waist_control_node = Node(
        package='qyh_waist_control',
        executable='waist_control_node',
        name='waist_control_node',
        output='screen',
        parameters=[
            config_file,
            {
                'plc_ip': LaunchConfiguration('plc_ip'),
                'plc_port': LaunchConfiguration('plc_port'),
            }
        ],
        remappings=[
            ('waist_state', '/waist/state'),
            ('waist_control', '/waist/control'),
        ]
    )

    return LaunchDescription([
        plc_ip_arg,
        plc_port_arg,
        waist_control_node,
    ])
