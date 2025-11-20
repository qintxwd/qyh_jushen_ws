#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qyh_jaka_control_gui',
            executable='vr_control_gui',
            name='vr_control_gui',
            output='screen',
            emulate_tty=True,
        ),
    ])
