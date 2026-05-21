#!/usr/bin/env python3
"""
Launch file for rexrov BOP panel demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),

        # Include uuv_gazebo_worlds BOP panel world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_gazebo_worlds'),
                    'launch',
                    'rov_bop_panel.launch.py'
                ])
            ),
            launch_arguments={
                'x': '2.5',
                'y': '0',
                'z': '-4',
                'angle': '3.14',
            }.items(),
        ),

        # Include rexrov oberon arms demo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_gazebo'),
                    'launch',
                    'rexrov_demos',
                    'rexrov_oberon_arms_demo.launch.py'
                ])
            ),
            launch_arguments={
                'x': '0',
                'y': '0',
                'z': '-4',
                'joy_id': LaunchConfiguration('joy_id'),
            }.items(),
        ),
    ])
