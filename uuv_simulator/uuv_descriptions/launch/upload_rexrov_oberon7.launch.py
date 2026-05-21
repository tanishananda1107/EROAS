#!/usr/bin/env python3
"""
Launch file to upload rexrov with oberon7 arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    roll = LaunchConfiguration('roll').perform(context)
    pitch = LaunchConfiguration('pitch').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    debug = LaunchConfiguration('debug').perform(context)

    inertial_reference_frame = 'world_ned' if use_ned_frame == 'true' else 'world'

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{
            'robot_description': f'/{namespace}/robot_description',
        }]
    )

    # Include message_to_tf launch
    message_to_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('uuv_assistants'),
                'launch',
                'message_to_tf.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'world_frame': 'world',
            'child_frame_id': f'/{namespace}/base_link',
        }.items()
    )

    return [robot_state_publisher, message_to_tf]


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
        DeclareLaunchArgument('namespace', default_value='rexrov',
                             description='Namespace for the robot'),
        DeclareLaunchArgument('use_ned_frame', default_value='false',
                             description='Use NED frame'),

        OpaqueFunction(function=launch_setup)
    ])
