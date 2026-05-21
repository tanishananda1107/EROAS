#!/usr/bin/env python3
"""
Launch file for setting simulation timeout.
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
            description='Simulation timeout value'
        ),
        Node(
            package='uuv_assistants',
            executable='set_simulation_timer.py',
            name='simulation_timeout',
            output='screen',
            parameters=[{
                'timeout': LaunchConfiguration('timeout')
            }]
        ),
    ])
