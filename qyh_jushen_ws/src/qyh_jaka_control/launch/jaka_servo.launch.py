from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('qyh_jaka_control')
    config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.2.200',
            description='IP address of the robot controller'
        ),
        
        DeclareLaunchArgument(
            'cycle_time_ms',
            default_value='8.0',
            description='Servo control cycle time in milliseconds'
        ),

        Node(
            package='qyh_jaka_control',
            executable='jaka_servo_node_vr',
            name='jaka_servo_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'cycle_time_ms': LaunchConfiguration('cycle_time_ms'),
                }
            ]
        ),
    ])
