#!/usr/bin/env python3
"""
Launch file for UUV keyboard teleoperation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='Name of the UUV'),
        DeclareLaunchArgument('output_topic', default_value='cmd_vel', description='Output topic'),
        DeclareLaunchArgument('message_type', default_value='twist', description='Message type'),

        Node(
            package='uuv_teleop',
            executable='vehicle_keyboard_teleop.py',
            name='keyboard_uuv_velocity_teleop',
            output='screen',
            remappings=[
                ('output', f'/{LaunchConfiguration("uuv_name")}/{LaunchConfiguration("output_topic")}'),
            ],
            parameters=[{
                'type': LaunchConfiguration('message_type')
            }]
        ),
    ])
