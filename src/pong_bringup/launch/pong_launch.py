#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='pong_controller',
            executable='pong_controller',
            name='pong_controller',
            parameters=[],
            output='screen'
            ),

        Node(
            package='pong_ball',
            executable='pong_ball',
            name='pong_ball',
            parameters=[],
            output='screen'
            ),

    ])
