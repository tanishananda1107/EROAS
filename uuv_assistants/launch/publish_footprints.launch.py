#!/usr/bin/env python3
"""
Launch file for publishing footprints.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        Node(
            package='uuv_assistants',
            executable='publish_footprints.py',
            name='publish_footprints',
            output='screen'
        ),
    ])
