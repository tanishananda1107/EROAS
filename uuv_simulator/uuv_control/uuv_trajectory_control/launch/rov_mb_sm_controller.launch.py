#!/usr/bin/env python3
"""
Launch file for ROV MB-SM controller.
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
    lambda_val = LaunchConfiguration('lambda').perform(context)
    rho_constant = LaunchConfiguration('rho_constant').perform(context)
    k = LaunchConfiguration('k').perform(context)
    c = LaunchConfiguration('c').perform(context)
    adapt_slope = LaunchConfiguration('adapt_slope').perform(context)
    rho_0 = LaunchConfiguration('rho_0').perform(context)
    drift_prevent = LaunchConfiguration('drift_prevent').perform(context)

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
        executable='rov_mb_sm_controller.py',
        name='rov_mb_smcontroller',
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
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'config',
                'models',
                model_name,
                'params.yaml'
            ]),
            {
                'saturation': saturation,
                'lambda': [float(x) for x in lambda_val.split(',')],
                'rho_constant': [float(x) for x in rho_constant.split(',')],
                'k': [float(x) for x in k.split(',')],
                'c': [float(x) for x in c.split(',')],
                'adapt_slope': [float(x) for x in adapt_slope.split(',')],
                'rho_0': [float(x) for x in rho_0.split(',')],
                'drift_prevent': drift_prevent,
                'inertial_frame_id': 'world_ned' if use_ned_frame == 'true' else 'world',
            }
        ],
        condition=IfCondition(condition='true')
    )

    if use_params_file == 'true':
        # Load from params file instead
        controller_node.parameters = [
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'config',
                'controllers',
                'mb_sm',
                model_name,
                'params.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('uuv_trajectory_control'),
                'config',
                'models',
                model_name,
                'params.yaml'
            ]),
            {
                'saturation': saturation,
                'inertial_frame_id': 'world_ned' if use_ned_frame == 'true' else 'world',
            }
        ]
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
        DeclareLaunchArgument('lambda', default_value='10,10,10,10,10,10', description='Lambda parameter'),
        DeclareLaunchArgument('rho_constant', default_value='10000,10000,10000,10000,10000,10000', description='Rho constant'),
        DeclareLaunchArgument('k', default_value='500,500,500,500,500,500', description='K parameter'),
        DeclareLaunchArgument('c', default_value='50,50,50,1,1,1', description='C parameter'),
        DeclareLaunchArgument('adapt_slope', default_value='100,10,10', description='Adapt slope'),
        DeclareLaunchArgument('rho_0', default_value='3000,3000,8000,1500,1500,8000', description='Rho 0'),
        DeclareLaunchArgument('drift_prevent', default_value='0.03', description='Drift prevent'),

        OpaqueFunction(function=launch_setup),
    ])
