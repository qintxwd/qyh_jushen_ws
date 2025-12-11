from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('orbbec_camera'),
                    'launch',
                    'gemini_330_series.launch.py'
                ])
            ),
            launch_arguments={
                'camera_name': 'head_camera',
                'enumerate_net_device': 'true',
                # 'net_device_ip': '192.168.1.10', # Uncomment for static IP
                'device_num': '1',
                'color_width': '640',
                'color_height': '400',
                'color_fps': '15',
                'depth_width': '640',
                'depth_height': '400',
                'depth_fps': '15',
                'enable_colored_point_cloud': 'false',
                'depth_registration': 'true',
                'enable_point_cloud': 'false',
            }.items()
        )
    ])
