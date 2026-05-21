#!/usr/bin/env python3
"""
Launch file for unpause simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'timeout',
            default_value='0',
            description='Timeout value'
        ),
        Node(
            package='uuv_assistants',
            executable='unpause_simulation.py',
            name='unpause_simulation',
            output='screen',
            parameters=[{
                'timeout': LaunchConfiguration('timeout')
            }]
        ),
    ])
