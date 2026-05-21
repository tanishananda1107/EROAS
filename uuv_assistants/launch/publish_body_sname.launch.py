#!/usr/bin/env python3
"""
Launch file for publishing body sname frame transform.
Creates a static transform from base_link to base_link_ned frame.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for body sname frame."""
    uuv_name = LaunchConfiguration('uuv_name').perform(context)

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='sname_frame_publisher',
            namespace=uuv_name,
            arguments=[
                '0', '0', '0', '0', '0', '3.141592653589793',
                '/base_link',
                '/base_link_ned'
            ]
        ),
    ]


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'uuv_name',
            description='UUV name for namespace'
        ),
    ])
