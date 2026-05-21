#!/usr/bin/env python3
"""
Launch file to publish body state name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('body_state_name', default_value='body_state',
                             description='Body state name'),

        Node(
            package='uuv_assistants',
            executable='uuv_publish_body_sname',
            name='publish_body_sname',
            output='screen',
            namespace='$(var namespace)',
            parameters=[{
                'body_state_name': LaunchConfiguration('body_state_name'),
            }]
        ),
    ])
