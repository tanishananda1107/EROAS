#!/usr/bin/env python3
"""
Launch file for rexrov default demo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration."""
    namespace = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    joy_id = LaunchConfiguration('joy_id').perform(context)
    axis_yaw = LaunchConfiguration('axis_yaw').perform(context)
    axis_x = LaunchConfiguration('axis_x').perform(context)
    axis_y = LaunchConfiguration('axis_y').perform(context)
    axis_z = LaunchConfiguration('axis_z').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)

    actions = []

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
            'mode': 'default',
            'namespace': namespace,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items(),
    ))

    # Include thruster_manager
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': namespace,
            'model_name': 'rexrov',
        }.items(),
    ))

    # Acceleration control node
    actions.append(Node(
        package='uuv_control_cascaded_pid',
        executable='AccelerationControl.py',
        name='acceleration_control',
        output='screen',
        parameters=[{
            'tf_prefix': f'{namespace}/'
        }]
    ))

    # Velocity control node
    actions.append(Node(
        package='uuv_control_cascaded_pid',
        executable='VelocityControl.py',
        name='velocity_control',
        output='screen',
        remappings=[
            ('odom', f'/{namespace}/pose_gt'),
            ('cmd_accel', f'/{namespace}/cmd_accel'),
        ]
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
            'uuv_name': namespace,
            'joy_id': joy_id,
            'output_topic': 'cmd_vel',
            'message_type': 'twist',
            'axis_yaw': axis_yaw,
            'axis_x': axis_x,
            'axis_y': axis_y,
            'axis_z': axis_z,
        }.items(),
    ))

    # RViz if requested
    if launch_rviz == '1':
        actions.append(Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('uuv_gazebo'),
                'rviz',
                'rexrov_default.rviz'
            ])]
        ))

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='rexrov', description='Namespace'),
        DeclareLaunchArgument('x', default_value='0', description='X position'),
        DeclareLaunchArgument('y', default_value='0', description='Y position'),
        DeclareLaunchArgument('z', default_value='-70', description='Z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),
        DeclareLaunchArgument('joy_id', default_value='0', description='Joystick ID'),
        DeclareLaunchArgument('axis_yaw', default_value='0', description='Axis for yaw'),
        DeclareLaunchArgument('axis_x', default_value='4', description='Axis for x'),
        DeclareLaunchArgument('axis_y', default_value='3', description='Axis for y'),
        DeclareLaunchArgument('axis_z', default_value='1', description='Axis for z'),
        DeclareLaunchArgument('launch_rviz', default_value='1', description='Launch RViz'),

        OpaqueFunction(function=launch_setup),
    ])
