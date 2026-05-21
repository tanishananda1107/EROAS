#!/usr/bin/env python3
"""
Launch file to upload rexrov model.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='0',
                             description='Debug mode'),
        DeclareLaunchArgument('x', default_value='0',
                             description='X position'),
        DeclareLaunchArgument('y', default_value='0',
                             description='Y position'),
        DeclareLaunchArgument('z', default_value='-20',
                             description='Z position'),
        DeclareLaunchArgument('roll', default_value='0.0',
                             description='Roll angle'),
        DeclareLaunchArgument('pitch', default_value='0.0',
                             description='Pitch angle'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                             description='Yaw angle'),
        DeclareLaunchArgument('mode', default_value='default',
                             description='Mode'),
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('use_ned_frame', default_value='false',
                             description='Use NED frame'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('uuv_descriptions'),
                    'launch',
                    f'upload_rexrov_{LaunchConfiguration("mode")}.launch.py'
                ])
            ]),
            launch_arguments={
                'debug': LaunchConfiguration('debug'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'roll': LaunchConfiguration('roll'),
                'pitch': LaunchConfiguration('pitch'),
                'yaw': LaunchConfiguration('yaw'),
                'use_ned_frame': LaunchConfiguration('use_ned_frame'),
                'namespace': LaunchConfiguration('namespace'),
            }.items()
        ),
    ])
