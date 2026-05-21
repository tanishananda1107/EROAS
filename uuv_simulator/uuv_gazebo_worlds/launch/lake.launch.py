#!/usr/bin/env python3
"""
Launch file for lake world simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for lake world."""
    gui = LaunchConfiguration('gui').perform(context)
    paused = LaunchConfiguration('paused').perform(context)
    set_timeout = LaunchConfiguration('set_timeout').perform(context)
    timeout = LaunchConfiguration('timeout').perform(context)

    actions = []

    # Include empty world launch from gazebo_ros
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'empty_world.launch.py'
            ])
        ),
        launch_arguments={
            'world_name': PathJoinSubstitution([
                FindPackageShare('uuv_gazebo_worlds'),
                'worlds',
                'lake.world'
            ]),
            'paused': paused,
            'use_sim_time': 'true',
            'gui': gui,
            'headless': 'false',
            'debug': 'false',
            'verbose': 'true',
        }.items(),
    ))

    # Include publish_world_ned_frame
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_assistants'),
                'launch',
                'publish_world_ned_frame.launch.py'
            ])
        ),
    ))

    # Node for publishing world models
    actions.append(Node(
        package='uuv_assistants',
        executable='publish_world_models.py',
        name='publish_world_models',
        output='screen',
        parameters=[{
            'meshes': {
                'lake': {
                    'mesh': 'package://uuv_gazebo_worlds/models/lake/meshes/LakeBottom.dae',
                    'model': 'lake'
                }
            }
        }],
    ))

    # Conditional include for set_simulation_timer
    if set_timeout == 'true':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uuv_assistants'),
                    'launch',
                    'set_simulation_timer.launch.py'
                ])
            ),
            launch_arguments={
                'timeout': timeout,
            }.items(),
            condition=IfCondition(condition='true'),
        ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Enable GUI'
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start simulation paused'
        ),
        DeclareLaunchArgument(
            'set_timeout',
            default_value='false',
            description='Set simulation timeout'
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='0.0',
            description='Timeout value'
        ),
        OpaqueFunction(function=launch_setup),
    ])
