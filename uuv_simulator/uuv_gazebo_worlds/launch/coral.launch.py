#!/usr/bin/env python3
"""
Launch file for coral world simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for coral world."""
    gui = LaunchConfiguration('gui').perform(context)
    paused = LaunchConfiguration('paused').perform(context)

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
                'coral.world'
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
                'heightmap': {
                    'mesh': 'package://uuv_gazebo_worlds/models/sand_heightmap/meshes/heightmap.dae',
                    'model': 'sand_heightmap'
                },
                'seafloor': {
                    'plane': [2000, 2000, 0.1],
                    'pose': {
                        'position': [0, 0, -100]
                    }
                },
                'north': {
                    'plane': [0.1, 2000, 100],
                    'pose': {
                        'position': [1000, 0, -50]
                    }
                },
                'south': {
                    'plane': [0.1, 2000, 100],
                    'pose': {
                        'position': [-1000, 0, -50]
                    }
                },
                'west': {
                    'plane': [2000, 0.1, 100],
                    'pose': {
                        'position': [0, -1000, -50]
                    }
                },
                'east': {
                    'plane': [2000, 0.1, 100],
                    'pose': {
                        'position': [0, 1000, -50]
                    }
                }
            }
        }],
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
        OpaqueFunction(function=launch_setup),
    ])
