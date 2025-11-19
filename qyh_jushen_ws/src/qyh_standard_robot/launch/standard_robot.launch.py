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

    # Declare launch arguments
    modbus_ip_arg = DeclareLaunchArgument(
        'modbus_ip',
        default_value='192.168.1.100',
        description='IP address of the Standard robot chassis'
    )
    
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port',
        default_value='502',
        description='Modbus TCP port'
    )
    
    slave_id_arg = DeclareLaunchArgument(
        'slave_id',
        default_value='1',
        description='Modbus slave ID'
    )

    # Standard robot node
    standard_robot_node = Node(
        package='qyh_standard_robot',
        executable='standard_robot_node',
        name='standard_robot_node',
        output='screen',
        parameters=[
            config_file,
            {
                'modbus_ip': LaunchConfiguration('modbus_ip'),
                'modbus_port': LaunchConfiguration('modbus_port'),
                'slave_id': LaunchConfiguration('slave_id'),
            }
        ]
    )

    return LaunchDescription([
        modbus_ip_arg,
        modbus_port_arg,
        slave_id_arg,
        standard_robot_node
    ])
