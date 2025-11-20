#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qyh_jaka_control_gui',
            executable='jaka_gui',
            name='jaka_complete_gui',
            output='screen',
            emulate_tty=True,
        ),
    ])
