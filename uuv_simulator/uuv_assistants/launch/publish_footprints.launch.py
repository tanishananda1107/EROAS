#!/usr/bin/env python3
"""
Launch file to publish footprints.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('footprint_topic', default_value='footprint',
                             description='Footprint topic'),
        DeclareLaunchArgument('child_frame_id', default_value='$(var namespace)/base_footprint',
                             description='Child frame ID'),

        Node(
            package='uuv_assistants',
            executable='uuv_publish_footprints',
            name='publish_footprints',
            output='screen',
            namespace='$(var namespace)',
            parameters=[{
                'footprint_topic': LaunchConfiguration('footprint_topic'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
            }]
        ),
    ])
