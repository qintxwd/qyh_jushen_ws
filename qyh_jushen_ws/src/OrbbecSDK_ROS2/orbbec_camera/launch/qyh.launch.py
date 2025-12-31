from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    
    launch_head_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        # ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true color_width:=640 color_height:=480 depth_width:=640 depth_height:=480 depth_fps:=15 color_fps:=15 depth_registration:=true enumerate_net_device:=true camera_name:=head_camera camera_serial:=CPE895300029 enumerate_usb_device:=false
        # ros2 launch orbbec_camera gemini_330_series.launch.py
        # enable_point_cloud:=true 
        # color_width:=640 
        # color_height:=480 
        # depth_width:=640 
        # depth_height:=480 
        # depth_fps:=15 
        # color_fps:=15 
        # depth_registration:=true 
        # enumerate_net_device:=true 
        # camera_name:=head_camera 
        # camera_serial:=CPE895300029
        launch_arguments={
            'enable_point_cloud': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '15',
            'color_fps': '15',            
            'depth_registration': 'true',            
            'enumerate_net_device': 'true',
            'enumerate_usb_device': 'false',
            'camera_name': 'head_camera',
            'camera_serial': 'CPE895300029',
        }.items(),
    )
    
    launch_left_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        # ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true color_width:=640 color_height:=480 depth_width:=640 depth_height:=480 depth_fps:=15 color_fps:=15 depth_registration:=true enumerate_net_device:=false camera_name:=left_camera camera_serial:=CP0BB53000AT
        # ros2 launch orbbec_camera gemini_330_series.launch.py 
        # enable_point_cloud:=true 
        # color_width:=640 
        # color_height:=480 
        # depth_width:=640 
        # depth_height:=480 
        # depth_fps:=15 
        # color_fps:=15 
        # depth_registration:=true 
        # enumerate_net_device:=false 
        # camera_name:=left_camera 
        # camera_serial:=CP0BB53000AT
        launch_arguments={
            'enable_point_cloud': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '15',
            'color_fps': '15',            
            'depth_registration': 'true',            
            'enumerate_net_device': 'false',
            'enumerate_usb_device': 'true',
            'camera_name': 'left_hand_camera',
            'camera_serial': 'CP0BB53000AT',
            'usb_port': '2-3.3.4',
        }.items(),
    )
     
    launch_right_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        # ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true color_width:=640 color_height:=480 depth_width:=640 depth_height:=480 depth_fps:=15 color_fps:=15 depth_registration:=true enumerate_net_device:=false camera_name:=right_camera camera_serial:=CP0BB530003J
        # ros2 launch orbbec_camera gemini_330_series.launch.py 
        # enable_point_cloud:=true 
        # color_width:=640 
        # color_height:=480 
        # depth_width:=640 
        # depth_height:=480 
        # depth_fps:=15 
        # color_fps:=15 
        # depth_registration:=true 
        # enumerate_net_device:=false 
        # camera_name:=right_camera 
        # camera_serial:=CP0BB530003J
        launch_arguments={
            'enable_point_cloud': 'true',
            'color_width': '640',
            'color_height': '480',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '15',
            'color_fps': '15',            
            'depth_registration': 'true',            
            'enumerate_net_device': 'false',
            'enumerate_usb_device': 'true',
            'camera_name': 'right_hand_camera',
            'camera_serial': 'CP0BB530003J',
            'usb_port': '2-3.1.3',
        }.items(),
    )
    
    ld = LaunchDescription(
        [
            TimerAction(period=0.0, actions=[GroupAction([launch_head_include])]),
            TimerAction(period=5.0, actions=[GroupAction([launch_left_include])]),
            TimerAction(period=10.0, actions=[GroupAction([launch_right_include])]),
        ]
    )

    return ld




    # # 相机启动函数，保证顺序启动
    # def launch_cameras(context, *args, **kwargs):
        # actions = []

        # # 头部网络相机
        # head_camera = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
        #     ),
        #     launch_arguments={
        #         'camera_name': 'head_camera',
        #         'camera_serial': 'CPE895300029',
        #         'enumerate_net_device': 'true',
        #         'sync_mode': 'PRIMARY',
        #         'enable_left_ir': 'true',
        #         'enable_right_ir': 'true',
        #     }.items()
        # )
        # actions.append(head_camera)

    # # 左手 USB 相机
    # left_hand_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
    #     ),
    #     launch_arguments={
    #         'camera_name': 'left_hand_camera',
    #         'camera_serial': 'CP0BB530003J',
    #         'usb_port': '2-3.1.3-6',
    #         'enumerate_usb_device': 'true',
    #         'sync_mode': 'SECONDARY',
    #         'enable_left_ir': 'true',
    #         'enable_right_ir': 'true',
    #     }.items()
    # )
    # # actions.append(left_hand_camera)

    # # 右手 USB 相机
    # right_hand_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_file_dir, 'gemini_330_series.launch.py')
    #     ),
    #     launch_arguments={
    #         'camera_name': 'right_hand_camera',
    #         'camera_serial':  'CP0BB53000AT',
    #         'usb_port': '2-3.1.1-5',
    #         'enumerate_usb_device': 'true',
    #         'sync_mode': 'SECONDARY',
    #         'enable_left_ir': 'true',
    #         'enable_right_ir': 'true',
    #     }.items()
    # )
    # # actions.append(right_hand_camera)

    #     # return actions

    # return LaunchDescription([
    #     OpaqueFunction(function=launch_cameras)
    # ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource


# def generate_launch_description():
#     # Include launch files
#     package_dir = get_package_share_directory('orbbec_camera')
#     launch_file_dir = os.path.join(package_dir, 'launch')

#     HEAD_CAMERA = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
#         ),
#         launch_arguments={
#             'camera_model': 'gemini330_series',
#             'attach_component_container_enable': 'true',
#             'camera_name': 'head_camera',
#             'usb_port': '2-3.1.1',
#             'camera_serial': 'CPE895300029',
#             'enumerate_net_device': 'true',
#             'enable_left_ir': 'true',
#             'enable_right_ir': 'true',
#             # 'sync_mode': 'primary',
#         }.items()
#     )

#     LEFT_CAMERA = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
#         ),
#         launch_arguments={
#             'camera_model': 'gemini330_series',
#             'attach_component_container_enable': 'false',
#             'camera_name': 'left_hand_camera',
#             'usb_port': '2-3.1.1',        # ← 左手相机端口
#             'device_num': '4',
#             'sync_mode': 'secondary_synced',
#             'enable_left_ir': 'true',
#             'enable_right_ir': 'true',
#         }.items()
#     )

#     RIGHT_CAMERA = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
#         ),
#         launch_arguments={
#             'camera_model': 'gemini330_series',
#             'attach_component_container_enable': 'true',
#             'camera_name': 'right_hand_camera',
#             'usb_port': '2-3.1.3',        # ← 右手相机端口
#             'device_num': '4',
#             'sync_mode': 'secondary_synced',
#             'enable_left_ir': 'true',
#             'enable_right_ir': 'true',
#         }.items()
#     )

#     # Launch description
#     ld = LaunchDescription(
#         [
#             TimerAction(period=0.0, actions=[GroupAction([HEAD_CAMERA])]),
#             TimerAction(period=4.0, actions=[GroupAction([LEFT_CAMERA])]),
#             TimerAction(period=8.0, actions=[GroupAction([RIGHT_CAMERA])])
#             # The primary camera should be launched at last
#         ]
#     )

#     return ld