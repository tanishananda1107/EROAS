#!/usr/bin/env python3
"""
Launch file for PID controller demo with teleop.
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
    joy_id = LaunchConfiguration('joy_id').perform(context)
    axis_yaw = LaunchConfiguration('axis_yaw').perform(context)
    axis_x = LaunchConfiguration('axis_x').perform(context)
    axis_y = LaunchConfiguration('axis_y').perform(context)
    axis_z = LaunchConfiguration('axis_z').perform(context)

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
            'x': '20',
            'y': '0',
            'z': '-20',
            'yaw': '0',
            'use_ned_frame': use_ned_frame,
        }.items(),
    ))

    # Include PID controller
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'launch',
                'rov_pid_controller.launch.py'
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

    # Include uuv_teleop
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_teleop'),
                'launch',
                'uuv_teleop.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': 'rexrov',
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': axis_yaw,
            'axis_x': axis_x,
            'axis_y': axis_y,
            'axis_z': axis_z,
            'gain_yaw': '0.2',
            'gain_x': '0.5',
            'gain_y': '0.5',
            'gain_z': '0.5',
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
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis for yaw'),
        DeclareLaunchArgument('axis_x', default_value='4', description='Axis for x'),
        DeclareLaunchArgument('axis_y', default_value='3', description='Axis for y'),
        DeclareLaunchArgument('axis_z', default_value='1', description='Axis for z'),

        OpaqueFunction(function=launch_setup),
    ])
