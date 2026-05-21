#!/usr/bin/env python3
"""
Launch file for uploading rexrov default noisy pose model.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration for uploading rexrov default noisy pose."""
    namespace = LaunchConfiguration('namespace').perform(context)
    world_frame = LaunchConfiguration('world_frame').perform(context)
    pose_noise = LaunchConfiguration('pose_noise').perform(context)

    actions = []

    # Spawn model
    actions.append(Node(
        package='gazebo_ros',
        executable='spawn_model.py',
        name='urdf_spawner_' + namespace,
        output='screen',
        arguments=[
            '-urdf',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw'),
            '-model', namespace,
            '-param', '/' + namespace + '/robot_description'
        ]
    ))

    # Robot state publisher
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_' + namespace,
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': '/robot_description',
            'publish_frequency': 5.0
        }]
    ))

    # Include publish_body_sname launch
    publish_body_sname_launch = Path(__file__).parent / 'publish_body_sname.launch.py'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(publish_body_sname_launch)),
        launch_arguments={
            'uuv_name': namespace
        }.items()
    ))

    # Include message_to_tf launch
    message_to_tf_launch = Path(__file__).parent / 'message_to_tf.launch.py'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(message_to_tf_launch)),
        launch_arguments={
            'namespace': namespace,
            'world_frame': world_frame
        }.items()
    ))

    return actions


def launch_describe():
    """Describe the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='0',
            description='Debug mode'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0',
            description='X position'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0',
            description='Y position'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='-20',
            description='Z position'
        ),
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Roll angle'
        ),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Pitch angle'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Yaw angle'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='default',
            description='Mode'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='rexrov',
            description='Namespace'
        ),
        DeclareLaunchArgument(
            'pose_noise',
            default_value='0.02',
            description='Pose noise amplitude'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='World frame'
        ),
    ])
