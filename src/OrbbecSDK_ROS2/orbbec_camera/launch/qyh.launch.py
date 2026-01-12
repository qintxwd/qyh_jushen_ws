from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    config_file_dir = os.path.join(package_dir, 'config')
    config_file_path = os.path.join(config_file_dir, 'camera_params.yaml')

    head_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={                     
            'enumerate_net_device': 'true',
            'camera_name': 'head_camera',
            'device_num': '1',
            'sync_mode': 'standalone',  # 单相机使用 standalone 模式，无需硬件触发
            'config_file_path': config_file_path,
            'camera_serial': 'CPE895300029',
        }.items(),
    )

    right_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'right_camera',
            'usb_port': '2-3.4.2',
            'device_num': '2',
            'sync_mode': 'software_triggering',
            'config_file_path': config_file_path,
            'camera_serial': 'CP0BB530003J'

        }.items()
    )

    left_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        ),
        launch_arguments={
            'camera_name': 'left_camera',
            'usb_port': '2-3.4.1',
            'device_num': '2',
            'sync_mode': 'standalone',  # 单相机使用 standalone 模式，无需硬件触发
            'config_file_path': config_file_path,
            'camera_serial': 'CP0BB53000AT'
        }.items()
    )
    
    ld = LaunchDescription([
        TimerAction(period=1.0, actions=[GroupAction([head_camera])]), # The primary camera should be launched at last
        # TimerAction(period=5.0, actions=[GroupAction([right_camera])]), # The primary camera should be launched at last
        # TimerAction(period=10.0, actions=[GroupAction([left_camera])]), # The primary camera should be launched at last
    ])

    return ld
