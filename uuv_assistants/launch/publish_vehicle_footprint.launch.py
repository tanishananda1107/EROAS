#!/usr/bin/env python3
"""
Launch file for publishing vehicle footprint.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'uuv_name',
            description='UUV name for namespace'
        ),
        DeclareLaunchArgument(
            'scale_footprint',
            default_value='10',
            description='Scale footprint'
        ),
        DeclareLaunchArgument(
            'scale_label',
            default_value='10',
            description='Scale label'
        ),
        DeclareLaunchArgument(
            'label_x_offset',
            default_value='60',
            description='Label X offset'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='pose_gt',
            description='Odometry topic'
        ),
        Node(
            package='uuv_assistants',
            executable='publish_vehicle_footprint.py',
            name='publish_footprints',
            namespace=LaunchConfiguration('uuv_name'),
            output='screen',
            parameters=[{
                'scale_footprint': LaunchConfiguration('scale_footprint'),
                'scale_label': LaunchConfiguration('scale_label'),
                'label_x_offset': LaunchConfiguration('label_x_offset'),
            }],
            remappings=[
                ('odom', LaunchConfiguration('odom_topic')),
            ]
        ),
    ])
