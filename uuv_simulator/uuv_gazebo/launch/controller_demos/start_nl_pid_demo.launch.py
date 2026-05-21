#!/usr/bin/env python3
"""
Launch file for NL PID controller demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration."""
    record = LaunchConfiguration('record').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)

    actions = []

    # Include ocean_waves world
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_gazebo_worlds'),
                'launch',
                'ocean_waves.launch.py'
            ])
        )
    ))

    # Include upload_rexrov
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_descriptions'),
                'launch',
                'upload_rexrov.launch.py'
            ])
        ),
        launch_arguments={
            'use_ned_frame': use_ned_frame,
        }.items(),
    ))

    # Include NL PID controller
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'launch',
                'rov_nl_pid_controller.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'model_name': 'rexrov',
            'use_ned_frame': use_ned_frame,
        }.items(),
    ))

    # Include record_demo if requested
    if record == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_gazebo'),
                    'launch',
                    'controller_demos',
                    'record_demo.launch.py'
                ])
            ),
            launch_arguments={
                'record': record,
                'use_ned_frame': use_ned_frame,
            }.items(),
        ))

    # RViz
    actions.append(Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('uuv_gazebo'),
            'rviz',
            'controller_demo.rviz'
        ])]
    ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='false', description='Record rosbag'),
        DeclareLaunchArgument('use_ned_frame', default_value='false', description='Use NED frame'),

        OpaqueFunction(function=launch_setup),
    ])
