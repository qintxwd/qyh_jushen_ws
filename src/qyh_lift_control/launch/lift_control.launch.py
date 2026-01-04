"""
升降电机控制节点启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    pkg_dir = get_package_share_directory('qyh_lift_control')
    config_file = os.path.join(pkg_dir, 'config', 'lift_params.yaml')

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

    # 升降电机控制节点
    lift_control_node = Node(
        package='qyh_lift_control',
        executable='lift_control_node',
        name='lift_control_node',
        output='screen',
        parameters=[
            config_file,
            {
                'plc_ip': LaunchConfiguration('plc_ip'),
                'plc_port': LaunchConfiguration('plc_port'),
            }
        ],
        remappings=[
            ('lift_state', '/lift/state'),
            ('lift_control', '/lift/control'),
        ]
    )

    return LaunchDescription([
        plc_ip_arg,
        plc_port_arg,
        lift_control_node,
    ])
