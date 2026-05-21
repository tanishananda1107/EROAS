#!/usr/bin/env python3
"""
Launch file to publish world NED frame.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('world_frame', default_value='world',
                             description='World frame ID'),

        Node(
            package='uuv_assistants',
            executable='uuv_publish_world_ned_frame',
            name='publish_world_ned_frame',
            output='screen',
            namespace='$(var namespace)',
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
            }]
        ),
    ])
