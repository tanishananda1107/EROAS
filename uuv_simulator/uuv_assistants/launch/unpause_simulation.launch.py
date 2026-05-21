#!/usr/bin/env python3
"""
Launch file to unpause simulation.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uuv_assistants',
            executable='uuv_unpause_simulation',
            name='uuv_unpause_simulation',
            output='screen',
        ),
    ])
