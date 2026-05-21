#!/usr/bin/env python3
"""
Launch file for rexrov default demo with Logitech joystick.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov', description='Namespace'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('x', default_value='0', description='X position'),
        DeclareLaunchArgument('y', default_value='0', description='Y position'),
        DeclareLaunchArgument('z', default_value='-70', description='Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_gazebo'),
                    'launch',
                    'rexrov_demos',
                    'rexrov_default.launch.py'
                ])
            ),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'joy_id': LaunchConfiguration('joy_id'),
                'axis_yaw': '2',
                'axis_x': '1',
                'axis_y': '0',
                'axis_z': '5',
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items(),
        ),
    ])
