#!/usr/bin/env python3
"""
Launch file for finned UUV teleoperation using joystick.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='Name of the UUV'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('use_param_file', default_value='0', description='Use parameter file'),
        DeclareLaunchArgument('filename', default_value='.', description='Parameter file path'),

        DeclareLaunchArgument('axis_thruster', default_value='1', description='Axis for thruster'),
        DeclareLaunchArgument('axis_roll', default_value='0', description='Axis for roll'),
        DeclareLaunchArgument('axis_pitch', default_value='4', description='Axis for pitch'),
        DeclareLaunchArgument('axis_yaw', default_value='3', description='Axis for yaw'),

        DeclareLaunchArgument('n_fins', default_value='4', description='Number of fins'),
        DeclareLaunchArgument('thruster_joy_gain', default_value='1.0', description='Thruster joy gain'),
        DeclareLaunchArgument('thruster_rotor_gain', default_value='0.0009', description='Thruster rotor gain'),
        DeclareLaunchArgument('max_thrust', default_value='200', description='Max thrust'),
        DeclareLaunchArgument('thruster_topic', default_value='thrusters/0/input', description='Thruster topic'),
        DeclareLaunchArgument('fin_topic_prefix', default_value='fins/', description='Fin topic prefix'),
        DeclareLaunchArgument('fin_topic_suffix', default_value='/input', description='Fin topic suffix'),

        DeclareLaunchArgument('gain_roll', default_value='1,1,1,1', description='Gain for roll'),
        DeclareLaunchArgument('gain_pitch', default_value='1,1,-1,-1', description='Gain for pitch'),
        DeclareLaunchArgument('gain_yaw', default_value='-1,1,1,-1', description='Gain for yaw'),

        # Conditional include for parameter file
        GroupAction(
            actions=[
                Node(
                    package='uuv_teleop',
                    executable='finned_uuv_teleop.py',
                    name='finned_uuv_teleop',
                    output='screen',
                    parameters=[{'param_file': LaunchConfiguration('filename')}]
                ),
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joystick',
                    parameters=[{
                        'autorepeat_rate': 10,
                        'dev': f'/dev/input/js{LaunchConfiguration("joy_id")}'
                    }]
                ),
            ],
            condition=IfCondition(LaunchConfiguration('use_param_file'))
        ),

        # Conditional include for default parameters
        GroupAction(
            actions=[
                Node(
                    package='uuv_teleop',
                    executable='finned_uuv_teleop.py',
                    name='finned_uuv_teleop',
                    output='screen',
                    parameters=[{
                        'axis_thruster': int(LaunchConfiguration('axis_thruster')),
                        'axis_roll': int(LaunchConfiguration('axis_roll')),
                        'axis_pitch': int(LaunchConfiguration('axis_pitch')),
                        'axis_yaw': int(LaunchConfiguration('axis_yaw')),
                        'n_fins': int(LaunchConfiguration('n_fins')),
                        'thruster_joy_gain': float(LaunchConfiguration('thruster_joy_gain')),
                        'thruster_model': {
                            'name': 'proportional',
                            'max_thrust': float(LaunchConfiguration('max_thrust')),
                            'params': {
                                'gain': float(LaunchConfiguration('thruster_rotor_gain'))
                            }
                        },
                        'gain_roll': [float(x) for x in LaunchConfiguration('gain_roll').get().split(',')],
                        'gain_pitch': [float(x) for x in LaunchConfiguration('gain_pitch').get().split(',')],
                        'gain_yaw': [float(x) for x in LaunchConfiguration('gain_yaw').get().split(',')],
                        'thruster_topic': LaunchConfiguration('thruster_topic'),
                        'fin_topic_prefix': LaunchConfiguration('fin_topic_prefix'),
                        'fin_topic_suffix': LaunchConfiguration('fin_topic_suffix'),
                    }]
                ),
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joystick',
                    parameters=[{
                        'autorepeat_rate': 10,
                        'dev': f'/dev/input/js{LaunchConfiguration("joy_id")}'
                    }]
                ),
            ],
            condition=UnlessCondition(LaunchConfiguration('use_param_file'))
        ),
    ])
