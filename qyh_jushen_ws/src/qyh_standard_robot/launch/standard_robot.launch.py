from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the config file path
    config_file = os.path.join(
        get_package_share_directory('qyh_standard_robot'),
        'config',
        'standard_robot.yaml'
    )

    # Declare launch arguments (empty default means use config file value)
    modbus_ip_arg = DeclareLaunchArgument(
        'modbus_ip',
        default_value='',
        description='IP address of the Standard robot chassis (leave empty to use config file)'
    )
    
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port',
        default_value='',
        description='Modbus TCP port (leave empty to use config file)'
    )
    
    slave_id_arg = DeclareLaunchArgument(
        'slave_id',
        default_value='',
        description='Modbus slave ID (leave empty to use config file)'
    )

    # Standard robot node - only use config file, launch args can override if provided
    standard_robot_node = Node(
        package='qyh_standard_robot',
        executable='qyh_standard_robot_node_exe',
        name='standard_robot_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        modbus_ip_arg,
        modbus_port_arg,
        slave_id_arg,
        standard_robot_node
    ])
