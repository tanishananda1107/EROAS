#!/usr/bin/env python3
"""
Launch file for ROV PID controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Set up the launch configuration."""
    uuv_name = LaunchConfiguration('uuv_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    saturation = LaunchConfiguration('saturation').perform(context)
    gui_on = LaunchConfiguration('gui_on').perform(context)
    use_params_file = LaunchConfiguration('use_params_file').perform(context)
    use_ned_frame = LaunchConfiguration('use_ned_frame').perform(context)
    Kp = LaunchConfiguration('Kp').perform(context)
    Kd = LaunchConfiguration('Kd').perform(context)
    Ki = LaunchConfiguration('Ki').perform(context)

    actions = []

    # Start thruster manager
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'launch',
                'thruster_manager.launch.py'
            ])
        ),
        launch_arguments={
            'uuv_name': uuv_name,
            'model_name': model_name,
            'output_dir': PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'config',
                model_name
            ]),
            'config_file': PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'config',
                model_name,
                'thruster_manager.yaml'
            ]),
            'tam_file': PathJoinSubstitution([
                FindPackageShare('uuv_thruster_manager'),
                'config',
                model_name,
                'TAM.yaml'
            ]),
        }.items(),
    ))

    # Trajectory marker publisher if gui_on
    if gui_on == 'true':
        actions.append(Node(
            package='uuv_control_utils',
            executable='trajectory_marker_publisher.py',
            name='trajectory_marker_publisher',
            output='screen',
            namespace=uuv_name,
            remappings=[
                ('trajectory', 'dp_controller/trajectory'),
                ('waypoints', 'dp_controller/waypoints'),
            ]
        ))

    # Controller node
    controller_node = Node(
        package='uuv_trajectory_control',
        executable='rov_pid_controller.py',
        name='rov_pid_controller',
        output='screen',
        namespace=uuv_name,
        remappings=[
            ('odom', 'pose_gt_ned' if use_ned_frame == 'true' else 'pose_gt'),
            ('trajectory', 'dp_controller/trajectory'),
            ('input_trajectory', 'dp_controller/input_trajectory'),
            ('waypoints', 'dp_controller/waypoints'),
            ('error', 'dp_controller/error'),
            ('reference', 'dp_controller/reference'),
            ('thruster_output', 'thruster_manager/input_stamped'),
        ],
        parameters=[{
            'saturation': saturation,
            'Kp': [float(x) for x in Kp.split(',')],
            'Kd': [float(x) for x in Kd.split(',')],
            'Ki': [float(x) for x in Ki.split(',')],
            'inertial_frame_id': 'world_ned' if use_ned_frame == 'true' else 'world',
        }],
        condition=IfCondition(condition='true')
    )

    if use_params_file == 'true':
        actions.append(controller_node)
    else:
        actions.append(controller_node)

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('uuv_name', description='Vehicle namespace'),
        DeclareLaunchArgument('model_name', default_value='rexrov', description='Vehicle model name'),
        DeclareLaunchArgument('saturation', default_value='1200', description='Thruster saturation'),
        DeclareLaunchArgument('gui_on', default_value='true', description='Enable GUI markers'),
        DeclareLaunchArgument('use_params_file', default_value='false', description='Use params file'),
        DeclareLaunchArgument('use_ned_frame', default_value='false', description='Use NED frame'),
        DeclareLaunchArgument('Kp', default_value='11993.888,11993.888,11993.888,19460.069,19460.069,19460.069', description='Proportional gains'),
        DeclareLaunchArgument('Kd', default_value='9077.459,9077.459,9077.459,18880.925,18880.925,18880.925', description='Derivative gains'),
        DeclareLaunchArgument('Ki', default_value='321.417,321.417,321.417,2096.951,2096.951,2096.951', description='Integral gains'),

        OpaqueFunction(function=launch_setup),
    ])
