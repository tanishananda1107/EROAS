#!/usr/bin/env python3
"""
Launch file for UUV teleoperation using joystick.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='Name of the UUV'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('deadman_button', default_value='-1', description='Deadman button'),
        DeclareLaunchArgument('exclusion_buttons', default_value='4,5', description='Exclusion buttons'),
        DeclareLaunchArgument('axis_roll', default_value='-1', description='Axis for roll'),
        DeclareLaunchArgument('axis_pitch', default_value='-1', description='Axis for pitch'),
        DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis for yaw'),
        DeclareLaunchArgument('axis_x', default_value='4', description='Axis for x'),
        DeclareLaunchArgument('axis_y', default_value='3', description='Axis for y'),
        DeclareLaunchArgument('axis_z', default_value='1', description='Axis for z'),
        DeclareLaunchArgument('gain_roll', default_value='0.0', description='Gain for roll'),
        DeclareLaunchArgument('gain_pitch', default_value='0.0', description='Gain for pitch'),
        DeclareLaunchArgument('gain_yaw', default_value='0.2', description='Gain for yaw'),
        DeclareLaunchArgument('gain_x', default_value='2', description='Gain for x'),
        DeclareLaunchArgument('gain_y', default_value='0.3', description='Gain for y'),
        DeclareLaunchArgument('gain_z', default_value='0.3', description='Gain for z'),
        DeclareLaunchArgument('output_topic', default_value='cmd_vel', description='Output topic'),
        DeclareLaunchArgument('message_type', default_value='twist', description='Message type'),

        Node(
            package='joy',
            executable='joy_node',
            name='joystick',
            parameters=[{
                'autorepeat_rate': 10,
                'dev': f'/dev/input/js{LaunchConfiguration("joy_id")}'
            }]
        ),

        Node(
            package='uuv_teleop',
            executable='vehicle_teleop.py',
            name='joy_uuv_velocity_teleop',
            remappings=[
                ('output', f'/{LaunchConfiguration("uuv_name")}/{LaunchConfiguration("output_topic")}'),
                ('joy', f'/{LaunchConfiguration("uuv_name")}/joy'),
            ],
            parameters=[{
                'type': LaunchConfiguration('message_type'),
                'deadman_button': int(LaunchConfiguration('deadman_button')),
                'exclusion_buttons': [int(x) for x in LaunchConfiguration('exclusion_buttons').get().split(',')],
                'mapping': {
                    'x': {'axis': int(LaunchConfiguration('axis_x')), 'gain': float(LaunchConfiguration('gain_x'))},
                    'y': {'axis': int(LaunchConfiguration('axis_y')), 'gain': float(LaunchConfiguration('gain_y'))},
                    'z': {'axis': int(LaunchConfiguration('axis_z')), 'gain': float(LaunchConfiguration('gain_z'))},
                    'roll': {'axis': int(LaunchConfiguration('axis_roll')), 'gain': float(LaunchConfiguration('gain_roll'))},
                    'pitch': {'axis': int(LaunchConfiguration('axis_pitch')), 'gain': float(LaunchConfiguration('gain_pitch'))},
                    'yaw': {'axis': int(LaunchConfiguration('axis_yaw')), 'gain': float(LaunchConfiguration('gain_yaw'))}
                }
            }]
        ),
    ])
