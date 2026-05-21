#!/usr/bin/env python3
"""
Launch file for publishing world NED frame transform.
Creates a static transform from world to world_ned frame.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_ned_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'world_ned']
        ),
    ])
