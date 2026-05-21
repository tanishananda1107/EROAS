#!/usr/bin/env python3
"""
Launch file for disturbance manager demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='UUV name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_control_utils'),
                    'launch',
                    'start_disturbance_manager.launch.py'
                ])
            ),
            launch_arguments={
                'uuv_name': LaunchConfiguration('uuv_name'),
                'use_file': '1',
                'disturbance_file': PathJoinSubstitution([
                    FindPackageShare('uuv_gazebo'),
                    'config',
                    'disturbances',
                    'test_disturbances.yaml'
                ]),
            }.items(),
        ),
    ])
