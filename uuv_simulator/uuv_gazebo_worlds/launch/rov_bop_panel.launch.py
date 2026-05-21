#!/usr/bin/env python3
"""
Launch file for spawning rov_bop_panel model.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for rov_bop_panel."""
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    angle = LaunchConfiguration('angle').perform(context)

    actions = []

    # Spawn the rov_bop_panel model
    actions.append(Node(
        package='gazebo_ros',
        executable='spawn_model',
        name='sdf_spawner',
        output='screen',
        arguments=[
            '-sdf',
            '-x', x,
            '-y', y,
            '-z', z,
            '-P', '1.57',
            '-Y', angle,
            '-model', 'rov_bop_panel',
            '-database', 'rov_bop_panel'
        ],
    ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'x',
            default_value='-1',
            description='X coordinate'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='-1',
            description='Y coordinate'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='-4',
            description='Z coordinate'
        ),
        DeclareLaunchArgument(
            'angle',
            default_value='0',
            description='Rotation angle'
        ),
        OpaqueFunction(function=launch_setup),
    ])
