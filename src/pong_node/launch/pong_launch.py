#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    return LaunchDescription([

        # DeclareLaunchArgument(
        #     'scan_mode',
        #     default_value=scan_mode,
        #     description='Specifying scan mode'),

        Node(
            package='pong_node',
            executable='pong_controller',
            name='pong_controller',
            parameters=[],
            output='screen'
            ),

        Node(
            package='pong_node',
            executable='pong_ball',
            name='pong_ball',
            parameters=[],
            output='screen'
            ),
            
        # Node(
        #      package='tf2_ros',
        #      executable='static_transform_publisher',
        #      arguments = ['--frame-id', 'map', '--child-frame-id', 'ball']
        #     ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     # arguments=['-d', rviz_config_dir],
        #     output='screen'
        #     )
    ])